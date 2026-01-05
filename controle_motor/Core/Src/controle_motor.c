#include "controle_motor.h"
#include "math.h"
#include "tim.h"
#include "main.h"
#include "stm32c0xx_hal.h"

// ============================================================================
// DEFINIÇÕES DE MEMÓRIA FLASH (ESTRUTURA 128-BIT / 16 BYTES)
// ============================================================================

// Estrutura para organizar os dados na RAM antes de gravar
typedef struct {
    // --- Bloco 1 (64 bits / 8 Bytes) ---
    float    saved_freq;    // Bytes 0-3
    uint8_t  saved_P10;     // Byte 4
    uint8_t  saved_P11;     // Byte 5
    uint8_t  saved_P20;     // Byte 6
    uint8_t  saved_P21;     // Byte 7

    // --- Bloco 2 (64 bits / 8 Bytes) ---
    uint8_t  saved_P35;     // Byte 8
    uint8_t  saved_P42;     // Byte 9
    uint8_t  saved_P12;     // Byte 10
    uint8_t  saved_locked;  // Byte 11
    uint32_t padding;       // Bytes 12-15 (Reservado/Zero)
} SystemParams;

// ============================================================================
// 1. VARIÁVEIS GLOBAIS
// ============================================================================

// Inputs
volatile uint8_t P00 = 0;   // Comando de Sistema (7=Lock, 101=Reset)
volatile uint8_t P01 = 0;
volatile uint32_t P10 = 15; // Aceleração
volatile uint32_t P11 = 5;  // Desaceleração
volatile uint8_t P12 = 1;   // Memorização (Agora controla se o SAVE é permitido)
volatile uint8_t P20 = 1;   // Freq Mínima
volatile uint8_t P21 = 90;  // Freq Máxima
volatile uint8_t P35 = 0;   // Boost de Torque (0-9)
volatile uint8_t P42 = 10;  // Freq PWM / Torque

// Estado de Segurança
volatile bool params_locked = false;

volatile bool cmd_ligar_motor = false;
volatile float cmd_frequencia_alvo = 0.0f;

// Outputs
volatile float f_atual = 0.0f;
volatile uint32_t debug_isr_cnt = 0;

// Variáveis de Backup (Para impor o bloqueio)
static uint32_t backup_P10 = 15;
static uint32_t backup_P11 = 5;
static uint8_t  backup_P12 = 1;
static uint8_t  backup_P20 = 1;
static uint8_t  backup_P21 = 90;
static uint8_t  backup_P35 = 0;
static uint8_t  backup_P42 = 10;

// Variáveis de Memória (Shadow Registers - Execução)
static uint8_t P20_run = 1;
static uint8_t P21_run = 90;
static uint32_t P10_run = 15;
static uint32_t P11_run = 5;

// Controle de Estado
static bool last_motor_state = false;
static uint8_t last_P42 = 0;

// Variáveis de Controle da Rampa
volatile float ramp_inc_up = 0.0f;
volatile float ramp_inc_down = 0.0f;
volatile float inv_fs_2pi = 0.0f;

// Variáveis Dinâmicas de Hardware
volatile uint32_t current_arr = 99;
volatile float amp_max_atual = 50.0f;
volatile float freq_ISR = 10000.0f;

// Ângulos
static float theta_u = 0.0f;
static float theta_v = 2.094395f;
static float theta_w = 4.188790f;

static const float TWO_PI = 6.283185307f;

// ============================================================================
// 2. INICIALIZAÇÃO
// ============================================================================
void motor_init(void) {

    // 1. Define valores Padrão de Fábrica na RAM (caso Flash esteja vazia)
    P10 = 15; P11 = 5; P12 = 1;
    P20 = 1;  P21 = 90;
    P35 = 0;  P42 = 10;
    cmd_frequencia_alvo = 5.0f;
    params_locked = false;

    // 2. Tenta carregar da Flash. Se houver dados salvos, SOBRESCREVE os padrões.
    load_flash_data();

    // 3. Inicializa backups e variáveis de execução
    backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
    backup_P20 = P20; backup_P21 = P21;
    backup_P35 = P35; backup_P42 = P42;

    P10_run = P10;
    P11_run = P11;
    P20_run = P20;
    P21_run = P21;

    // Inicializa lógica P42
    if (P42 <= 7) P42 = 5; else if (P42 <= 12) P42 = 10; else P42 = 15;
    last_P42 = 0; // Força atualização no primeiro loop

    f_atual = (float) P20;
    motor_task();
}

// ============================================================================
// 3. MOTOR TASK (Loop Lento)
// ============================================================================
void motor_task(void) {

    // --- 3.1 LÓGICA DE COMANDO P00 (SISTEMA) ---
    if (P00 != 0) {
        // COMANDO 7 (007): Alternar Bloqueio
        if (P00 == 7) {
            params_locked = !params_locked;
            write_flash_data(); // Salva estado do bloqueio
        }
        // COMANDO 101: Reset de Fábrica
        else if (P00 == 101) {
            // Restaura valores default na RAM
            P10 = 15; P11 = 5;  P12 = 1;
            P20 = 1;  P21 = 90;
            P35 = 0;  P42 = 10;
            cmd_frequencia_alvo = 5.0f;
            params_locked = false;

            // Atualiza backups
            backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
            backup_P20 = P20; backup_P21 = P21;
            backup_P35 = P35; backup_P42 = P42;

            // Grava defaults na Flash (Reset Definitivo)
            write_flash_data();
        }
        P00 = 0; // Auto-clear
    }

    // --- 3.2 LÓGICA DE BLOQUEIO E VALIDAÇÃO ---
    if (params_locked) {
        // Se bloqueado, força valores de backup
        if (P10 != backup_P10) P10 = backup_P10;
        if (P11 != backup_P11) P11 = backup_P11;
        if (P12 != backup_P12) P12 = backup_P12;
        if (P20 != backup_P20) P20 = backup_P20;
        if (P21 != backup_P21) P21 = backup_P21;
        if (P35 != backup_P35) P35 = backup_P35;
        if (P42 != backup_P42) P42 = backup_P42;
    } else {
        // Se desbloqueado, valida limites e atualiza backups
        if (P10 < 5) P10 = 5;   if (P10 > 60) P10 = 60;
        if (P11 < 5) P11 = 5;   if (P11 > 60) P11 = 60;
        if (P20 < 1) P20 = 1;   if (P20 > 24) P20 = 24;
        if (P21 < 23) P21 = 23; if (P21 > 90) P21 = 90;
        if (P35 > 9) P35 = 9;

        if (cmd_frequencia_alvo < P20) cmd_frequencia_alvo = P20;
        if (cmd_frequencia_alvo > P21) cmd_frequencia_alvo = P21;

        // Atualiza backups
        backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
        backup_P20 = P20; backup_P21 = P21;
        backup_P35 = P35; backup_P42 = P42;
    }

    // --- 3.3 LÓGICA DO P42 (HARDWARE UPDATE) ---
    if (P42 != last_P42) {
        if (P42 <= 7) P42 = 5; else if (P42 <= 12) P42 = 10; else P42 = 15;
        atualiza_P42();
        last_P42 = P42;
    }

    // --- 3.4 TRANSIÇÃO DE ESTADOS ---
    // BORDA DE SUBIDA (Ligar)
    if (!last_motor_state && cmd_ligar_motor) {
        P20_run = P20; P21_run = P21;
        P10_run = P10; P11_run = P11;
    }

    // --- 3.5 ROTINA DE SALVAMENTO AUTOMÁTICO ---
    // Verifica se houve alteração nos parâmetros enquanto o motor está PARADO
    if (!cmd_ligar_motor) {
        // Lê Flash atual para comparar
        SystemParams flash_data;
        uint64_t *pFlash = (uint64_t*)FLASH_USER_ADDR;
        uint64_t dw1 = pFlash[0];
        uint64_t dw2 = pFlash[1];

        // Reconstrói struct a partir dos 2 DoubleWords lidos
        // Nota: Copiamos byte a byte para evitar problemas de alinhamento
        uint8_t* pDst = (uint8_t*)&flash_data;
        *(uint64_t*)(pDst) = dw1;
        *(uint64_t*)(pDst+8) = dw2;

        // Verifica se algo mudou na RAM em relação à Flash
        bool changed = false;

        if (fabs(flash_data.saved_freq - cmd_frequencia_alvo) > 0.1f) changed = true;
        if (flash_data.saved_P10 != (uint8_t)P10) changed = true;
        if (flash_data.saved_P11 != (uint8_t)P11) changed = true;
        if (flash_data.saved_P20 != P20) changed = true;
        if (flash_data.saved_P21 != P21) changed = true;
        if (flash_data.saved_P35 != P35) changed = true;
        if (flash_data.saved_P42 != P42) changed = true;
        if (flash_data.saved_P12 != P12) changed = true;
        if (flash_data.saved_locked != (params_locked ? 1 : 0)) changed = true;

        // Salva se houver mudança e P12 permitir (P12=1)
        if (changed && P12 == 1) {
            write_flash_data();
        }
    }

    last_motor_state = cmd_ligar_motor;

    // --- 3.6 EXECUÇÃO ---
    inv_fs_2pi = TWO_PI / freq_ISR;
    calcula_rampa();

    if (cmd_ligar_motor) {
        if (cmd_frequencia_alvo < P20) cmd_frequencia_alvo = P20;
        if (cmd_frequencia_alvo > P21) cmd_frequencia_alvo = P21;

        if ((TIM3->DIER & TIM_DIER_UIE) == 0) {
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            HAL_TIM_Base_Start_IT(&htim3);
        }
    }
    else {
        if (f_atual <= 0.1f) {
            HAL_TIM_Base_Stop_IT(&htim3);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        }
    }
}

// ============================================================================
// FUNÇÃO AUXILIAR: ATUALIZA HARDWARE
// ============================================================================
void atualiza_P42(void) {
    if (P42 == 5) current_arr = 199;
    else if (P42 == 15) current_arr = 66;
    else { P42 = 10; current_arr = 99; }

    amp_max_atual = (float)(current_arr + 1) / 2.0f;
    freq_ISR = 1000000.0f / (float)(current_arr + 1);

    __HAL_TIM_SET_AUTORELOAD(&htim1, current_arr);
    __HAL_TIM_SET_AUTORELOAD(&htim3, current_arr);
}

// ============================================================================
// FUNÇÕES DE FLASH (2 x DOUBLEWORD = 128 Bits)
// ============================================================================

void load_flash_data(void) {
    SystemParams ram_params;

    // Lê 2 DoubleWords (16 bytes) da Flash
    uint64_t *pFlash = (uint64_t*)FLASH_USER_ADDR;
    uint64_t dw1 = pFlash[0];
    uint64_t dw2 = pFlash[1];

    // Mapeia para struct
    uint8_t* pDst = (uint8_t*)&ram_params;
    *(uint64_t*)(pDst) = dw1;
    *(uint64_t*)(pDst+8) = dw2;

    // Se P12 salvo for 0xFF (flash virgem), ignora e mantem defaults
    if (ram_params.saved_P12 == 0xFF) {
        return; // Flash vazia, usa defaults definidos no motor_init
    }

    // Se Flash válida, carrega para variáveis globais
    if (!isnan(ram_params.saved_freq)) cmd_frequencia_alvo = ram_params.saved_freq;
    P10 = (uint32_t)ram_params.saved_P10;
    P11 = (uint32_t)ram_params.saved_P11;
    P20 = ram_params.saved_P20;
    P21 = ram_params.saved_P21;
    P35 = ram_params.saved_P35;
    P42 = ram_params.saved_P42;
    P12 = ram_params.saved_P12;

    params_locked = (ram_params.saved_locked == 1) ? true : false;
}

void write_flash_data(void) {
    __disable_irq();
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError;
    uint32_t page_number = (FLASH_USER_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page      = page_number;
    EraseInitStruct.NbPages   = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {

        // 1. Prepara Struct na RAM
        SystemParams data;
        data.saved_freq   = cmd_frequencia_alvo;
        data.saved_P10    = (uint8_t)P10;
        data.saved_P11    = (uint8_t)P11;
        data.saved_P20    = P20;
        data.saved_P21    = P21;
        data.saved_P35    = P35;
        data.saved_P42    = P42;
        data.saved_P12    = P12;
        data.saved_locked = params_locked ? 1 : 0;
        data.padding      = 0;

        // 2. Grava Bloco 1 (Bytes 0-7)
        uint64_t dw1 = *(uint64_t*)((uint8_t*)&data);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_ADDR, dw1);

        // 3. Grava Bloco 2 (Bytes 8-15)
        uint64_t dw2 = *(uint64_t*)((uint8_t*)&data + 8);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_ADDR + 8, dw2);
    }

    HAL_FLASH_Lock();
    __enable_irq();
}

// ============================================================================
// CÁLCULO DA RAMPA
// ============================================================================
void calcula_rampa(){
    float t_acc = (float)P10_run; if (t_acc < 0.1f) t_acc = 0.1f;
    float t_dec = (float)P11_run; if (t_dec < 0.1f) t_dec = 0.1f;
    float f_max_ref = (float)cmd_frequencia_alvo;

    if (last_P42 == 15){
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 6.7f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 6.7f);
    } else if (last_P42 == 10){
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 3.9f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 3.9f);
    } else {
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 2.2f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 2.2f);
    }

    if (cmd_ligar_motor) {
        float diferenca = f_atual - cmd_frequencia_alvo;
        if (diferenca < 0.0f) diferenca = -diferenca;
        if (diferenca < 0.1f) f_atual = cmd_frequencia_alvo;
    }

    if (f_atual > (cmd_frequencia_alvo - 0.1f) && cmd_ligar_motor == false){
        f_atual = cmd_frequencia_alvo - 0.15f;
    }
}

// ============================================================================
// 4. SPWM (Interrupção)
// ============================================================================
void spwm(void) {
    debug_isr_cnt++;

    float alvo = 0.0f;
    float step = 0.0f;
    float amp = 0.0f;

    if (cmd_ligar_motor) {
        alvo = cmd_frequencia_alvo;
        if (alvo > (float)P21_run) alvo = (float)P21_run;
        if (alvo < (float)P20_run) alvo = (float)P20_run;

        if (f_atual < alvo) {
            step = ramp_inc_up;
            f_atual += step;
            if (f_atual > alvo) f_atual = alvo;
        }
        else if (f_atual > alvo) {
            step = ramp_inc_down;
            f_atual -= step;
            if (f_atual < alvo) f_atual = alvo;
        }
    }
    else {
        alvo = 0.0f;
        step = ramp_inc_down;
        if (f_atual > 0.0f) {
            f_atual -= step;
            if (f_atual < 0.0f) f_atual = 0.0f;
        }
    }

    P01 = (uint8_t)f_atual;

    if (!cmd_ligar_motor && f_atual <= 0.1f) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        HAL_TIM_Base_Stop_IT(&htim3);
        return;
    }

    float inc = 0.0f;
    if (last_P42 == 15)      inc = f_atual * inv_fs_2pi * 6.45f;
    else if (last_P42 == 10) inc = f_atual * inv_fs_2pi * 4.31f;
    else                     inc = f_atual * inv_fs_2pi * 2.16f;

    theta_u += inc;
    if (theta_u >= TWO_PI) theta_u -= TWO_PI;
    theta_v = theta_u + 2.094395f;
    if (theta_v >= TWO_PI) theta_v -= TWO_PI;
    theta_w = theta_u + 4.188790f;
    if (theta_w >= TWO_PI) theta_w -= TWO_PI;

    float prop_vf = f_atual * 0.01666f;
    if (prop_vf > 1.0f) prop_vf = 1.0f;

    // Boost Torque P35
    if (f_atual < 20.0f){
        amp = amp_max_atual * (1.0f + (P35/10.0f));
    } else {
        amp = amp_max_atual;
    }

    float amp_atual = prop_vf * amp;
    if (amp_atual > amp_max_atual) amp_atual = amp_max_atual;

    float offset = amp_max_atual - amp_atual;

    uint16_t p1 = (uint16_t)((amp_atual * (sinf(theta_u) + 1.0f)) + offset);
    uint16_t p2 = (uint16_t)((amp_atual * (sinf(theta_v) + 1.0f)) + offset);
    uint16_t p3 = (uint16_t)((amp_atual * (sinf(theta_w) + 1.0f)) + offset);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
}
