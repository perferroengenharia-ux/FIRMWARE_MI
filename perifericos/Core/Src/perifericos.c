/*
 * perifericos.c
 *
 *  Created on: Nov 13, 2025
 *      Author: murilo
 *      Esse algoritmoo é reponsável pelas variáveis e funções de controle dos periféricos do Inversor ICW-xx.v1
 *      Os parâmetros que são parte dessa categoria, são:
 *
 *      P30 - OK
 *      P31 - OK
 *      P32 - OK
 *      P33 -
 *      P80 - OK
 *      P81 - OK
 *      P82 - OK
 *      P83 - OK
 *      P84 - OK
 *      P85 - OK
 *      P86 - OK
 *
 *      Sinalizar conforme implementar e testar cada um dos parâmetros, será considerado finalizado quando todos os itens
 *      estiverem devidamente testados.
 *
 *      Na Main é necessário ter uma explicação de como usar corretamente essas funções, e configurações necessárias (timer, GPIOs, etc)
 */
#include "perifericos.h"
#include "gpio.h"
#include "tim.h"
#include "stdint.h"
#include "stdbool.h"
#include "main.h"
/*===========================================================PARÂMETROS======================================*/
// --- Leitura por polling dos botões (detecção de borda simples) ---
static uint8_t last_onoff  = 0;
static uint8_t last_bomba  = 0;
static uint8_t last_swing  = 0;
static uint8_t last_dreno  = 0;
static uint8_t last_exaustor = 0;

volatile uint32_t P30			= 0;				// TEMPO MOLHADO
volatile uint32_t P31			= 1;				// TEMPO SECO
volatile uint8_t  P32 			= 60;				// VELOCIDADE DE SECAGEM
volatile uint8_t  P33           = 0;				// SENTIDO DE GIRO AO SECAR
volatile uint8_t  P51 			= 0;			    // SENTIDO DE ROTAÇÃO DO MOTOR
volatile uint8_t  P80 			= 1;				// DRENO 		*
volatile uint8_t  P81			= 1;				// SWING		*	0 (DESLIGADO), 1 (NA), 2 (NF)
volatile uint8_t  P82 			= 1;				// BOMBA		*
volatile uint32_t P83			= 1;				// TEMPO DE DRENO LIGADO
volatile uint32_t P84			= 1;				// TEMPO DE ENCHIMENTO
volatile uint8_t  P85 			= 1;				// SENSOR
volatile uint8_t  P86 			= 0;				//EXAUSTÃO		*	0 (DESLIGADO), 1 MODO SEM TIMER
/*===========================================================VARIÁVEIS======================================*/
//----------------Variáveis externas---------------------------------------
volatile uint32_t fatual		= 45;		// Deve definir como extern quando for aplicar no inversor
volatile uint32_t fsec			= 0;		// Deve definir como extern quando for aplicar no inversor
volatile bool motor_sentido = 0;   // sentido atual aplicado ao motor
volatile bool inversao_pendente = false; // usado quando P33=1

volatile uint32_t tm            = 0;		// Contador de tempo molhado
volatile uint32_t ts     	    = 0;		// Contador de tempo seco
volatile uint32_t td     		= 0;		// Contador de tempo dreno ligado
volatile uint32_t td_off 		= 0;		// Contador de tempo de enchimento
volatile uint32_t tsensor 		= 5;		// Tempo de estabilidade do sensor

// --- Filtro de estabilidade do sensor de nível (delay de 5s) ---
volatile uint32_t sensor_delay_counter = 0;  // contador em "ticks" do perifericos_tick
static uint8_t    sensor_estavel       = 0;  // valor filtrado (0/1)
static uint8_t    sensor_ultimo_raw    = 0;  // última leitura bruta

volatile bool ligar_bomba;
volatile bool desligar_bomba;
volatile bool ligar_swing;
volatile bool desligar_swing;
volatile bool ligar_dreno;
volatile bool desligar_dreno;
volatile bool ligar_motor;
volatile bool desligar_motor;

volatile bool flag_onoff 		= false;
volatile bool flag_bomba 		= false;
volatile bool flag_swing 		= false;
volatile bool flag_exaustor     = false;

volatile bool dreno_solicitado  = false;
volatile bool dreno_pendente  	= false;
volatile bool bomba_habilitada  = true;
volatile bool exaustor_pendente = false;
static  volatile bool motor_desligado	= false;

volatile Estado estado_atual	= IDLE;
/*===========================================================FUNÇÕES======================================*/
void perifericos_read_buttons(void)
{
    uint8_t onoff_now = (HAL_GPIO_ReadPin(ONOFF_BTN_GPIO_Port, ONOFF_BTN_Pin) == GPIO_PIN_SET);
    uint8_t bomba_now = (HAL_GPIO_ReadPin(BOMBA_BTN_GPIO_Port, BOMBA_BTN_Pin) == GPIO_PIN_SET);
    uint8_t swing_now = (HAL_GPIO_ReadPin(SWING_BTN_GPIO_Port, SWING_BTN_Pin) == GPIO_PIN_SET);
    uint8_t dreno_now = (HAL_GPIO_ReadPin(DRENO_BTN_GPIO_Port, DRENO_BTN_Pin) == GPIO_PIN_SET);
    uint8_t exaustor_now = (HAL_GPIO_ReadPin(EXAUSTOR_BTN_GPIO_Port, EXAUSTOR_BTN_Pin) == GPIO_PIN_SET);

    // borda de subida: 0 -> 1
    if (onoff_now && !last_onoff)  { flag_onoff       = true; }
    if (bomba_now && !last_bomba)  { flag_bomba       = true; }
    if (swing_now && !last_swing)  { flag_swing       = true; }
    if (dreno_now && !last_dreno)  { dreno_solicitado = true; }
    if (exaustor_now && !last_exaustor) { flag_exaustor = true; }

    // salva estados
    last_onoff = onoff_now;
    last_bomba = bomba_now;
    last_swing = swing_now;
    last_dreno = dreno_now;
    last_exaustor = exaustor_now;
}

//--------------------------DEFINE O FUNCIONAMENTO DOS PERIFÉRICOS
void perifericos_set (void){

	ligar_motor 	= true;
	desligar_motor = false;

	/*====================DRENO===============*/
	if (P80 == 0)
	{
		ligar_dreno    = false;
		desligar_dreno = false;
	}

	else if (P80 == 1)
	{
		ligar_dreno    = true;
		desligar_dreno = false;
	}
	else if (P80 == 2)
	{
		ligar_dreno    = false;
		desligar_dreno = true;
	}
	/*====================SWING===============*/
	if (P81 == 0)
	{
		ligar_swing    = false;
		desligar_swing = false;
	}
	else if (P81 == 1)
	{
		ligar_swing    = true;
		desligar_swing = false;
	}
	else if (P81 == 2)
	{
		ligar_swing    = false;
		desligar_swing = true;
	}
	/*====================BOMBA===============*/
	if (P82 == 0)
	{
		ligar_bomba    = false;
		desligar_bomba = false;
	}
	else if (P82 == 1)
	{
		ligar_bomba    = true;
		desligar_bomba = false;
	}
	else if (P82 == 2)
	{
		ligar_bomba    = false;
		desligar_bomba = true;
	}
	//Inicia com todos os periféricos desligados
	HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
	HAL_GPIO_WritePin(SWING_GPIO_Port, SWING_Pin, desligar_swing);
	HAL_GPIO_WritePin(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);
}

//--------------------------LEITURA DO SENSOR--------------------------------------------------------------
uint8_t sensor_read (void)
{
    // Retorno padronizado:
    // 1 = nível OK
    // 0 = falta de água

    if (P85 == 0)
    {
        // Sem sensor → sempre considera nível baixo
        return 0;
    }

    GPIO_PinState raw = HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin);

    if (P85 == 1)
    {
        // NA : pino alto = nível OK
        return (raw == GPIO_PIN_SET) ? 1 : 0;
    }
    else // P85 == 2 → NF (invertido)
    {
        // NF: pino baixo = nível OK
        return (raw == GPIO_PIN_RESET) ? 1 : 0;
    }
}

//--------------------------CALLBACK DO TIMER--------------------------------------------------------------
void perifericos_tick (void){

	if (tm > 0) 	tm--;
	if (ts > 0) 	ts--;
	if (td > 0) 	td--;
	if (td_off > 0) td_off--;
	if (sensor_delay_counter > 0) sensor_delay_counter--;

}
//-------------------------FUNÇÃO PRINCIPAL---------------------------------------------------------------
void perifericos_task (void)
{
    // lê botões por polling e seta flags
    perifericos_read_buttons();

    // define quando o equipamento é considerado "ligado"
    bool equipamento_ligado =
        (estado_atual == BOMBA_ON      ||
         estado_atual == VENTILADOR_ON ||
         estado_atual == OFF           ||
         estado_atual == BOMBA_OFF     ||
         estado_atual == EXAUSTOR_ON);

    uint8_t sensor_raw = sensor_read();
    if (sensor_raw != sensor_ultimo_raw)
    {
        // Houve mudança de nível: reinicia o contador de estabilidade
        sensor_ultimo_raw      = sensor_raw;
        sensor_delay_counter   = tsensor;     // 5 ticks do perifericos_tick (5s se tick=1s)
        // Enquanto o contador > 0, mantemos sensor_estavel no valor antigo
    }
    else
    {
        // Sensor manteve o mesmo valor
        if (sensor_delay_counter == 0)
        {
            // Já está estável há 5s → atualiza o valor filtrado
            sensor_estavel = sensor_raw;
        }
        // Se ainda > 0, estamos "esperando": sensor_estavel não muda
    }

    if (flag_swing)
    {
        // independente de aceitar ou não, sempre limpa a flag
        if (equipamento_ligado && P81 != 0)
        {
            // só permite alterar SWING se o equipamento estiver ligado
            HAL_GPIO_TogglePin(SWING_GPIO_Port, SWING_Pin);
        }

        flag_swing = false;
    }

    if (flag_bomba)
    {
        flag_bomba = false;

        // Só permite atuação se:
        // - bomba está configurada (P82 != 0)
        // - estamos no estado VENTILADOR_ON (já passou ciclo de ligar)
        // - o equipamento está ligado
        // - sensor indica água presente
        if ((P82 != 0) &&
            (estado_atual == VENTILADOR_ON) &&
            equipamento_ligado &&
            (sensor_read() == 1))
        {
            // Toggle lógico: usuário força ligar/desligar a bomba
            bomba_habilitada = !bomba_habilitada;
        }
    }

    /* =========================================================
     * @Brief	:	ROTINA DE LIGAR
     * =========================================================*/
    switch (estado_atual){
    case IDLE:

    	flag_exaustor = false;

        // 1) DRENO pode ser acionado em IDLE se P80 != 0
        if (dreno_solicitado && (P80 != 0))
        {
            dreno_solicitado = false;
            dreno_pendente   = false;   // é um dreno "isolado", não pós-desligamento

            // Garante tudo desligado
            HAL_GPIO_WritePin(MOTOR_GPIO_Port,  MOTOR_Pin,  desligar_motor);
            HAL_GPIO_WritePin(BOMBA_GPIO_Port,  BOMBA_Pin,  desligar_bomba);
            HAL_GPIO_WritePin(SWING_GPIO_Port,  SWING_Pin,  desligar_swing);

            // Liga DRENO pelo tempo P83
            HAL_GPIO_WritePin(DRENO_GPIO_Port, DRENO_Pin, ligar_dreno);
            td          = P83 * 60;           // contador já existe
            estado_atual = DRENO_ON;
            break; // não processa ONOFF neste ciclo
        }

        if (flag_onoff)
        {
            flag_onoff = false;
            bomba_habilitada = true;

            if (P82 == 0)
            {
                // Sem bomba: ONOFF só liga/desliga motor, sem tm/ts
                HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);
                motor_desligado = false;
                estado_atual = VENTILADOR_ON;   // ventilação simples
            }
            else
            {
                if (sensor_read() == 1)
                {
                    // Nível OK → inicia ciclo molhado
                    HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, ligar_bomba);

                    tm = P30 * 60; // tempo molhado
                    estado_atual = BOMBA_ON;
                }
                else
                {
                    // liga o motor imediatamente
                    HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);
                    motor_desligado = false;
                    // Nível baixo → apenas entra em ON aguardando sensor
                    estado_atual = VENTILADOR_ON;
                }
            }
        }
        break;

    case BOMBA_ON:

    	flag_exaustor = false;
    	dreno_solicitado = false;

        // Se o usuário apertar ONOFF aqui, ignora:
        if (flag_onoff)
        {
            flag_onoff = false;   // limpa, mas NÃO muda de estado
        }

        if (P82 == 0)
        {
            // Bomba desativada → não deveria estar aqui
            estado_atual = VENTILADOR_ON;
            break;
        }

        // 1) Se apertar ONOFF em qualquer momento -> OFF
        if (flag_onoff)
        {
            flag_onoff = false;
            estado_atual = OFF;
            break;
        }

        // 2) Controle da bomba pelo sensor
        if (sensor_estavel == 1)
        {
            // Nível OK -> bomba ligada
            HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, ligar_bomba);

        	if (flag_bomba)
        	{
        		HAL_GPIO_TogglePin(BOMBA_BTN_GPIO_Port, BOMBA_Pin);
        		flag_bomba = false;
        	}
        }
        else
        {
            // Sem água -> bomba desligada, mas NÃO mexe no motor
            HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
        }

        // 3) Quando o tempo molhado (tm) chegar a zero,
        //    liga o motor e vai para VENTILADOR_ON
        if (tm == 0)
        {
            // Liga o motor
            HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);
            motor_desligado = false;
            // Obs.: a bomba já está sendo controlada pelo sensor logo acima.
            // Se ainda tiver água (sensor==1), ela estará ligada; se não tiver, estará desligada.

            // Avança para o estado de ventilação
            estado_atual = VENTILADOR_ON;
        }

        break;

    case VENTILADOR_ON:

        // PEDIDO DE EXAUSTOR COM EQUIPAMENTO LIGADO
        if (flag_exaustor && (P86 != 0))
        {
            // Não iniciamos exaustão direto; primeiro desligamos normalmente
            flag_exaustor     = false;
            exaustor_pendente = true;   // marca que, após desligar, vai para exaustão

            // Inicia rotina de desligar (igual apertar ONOFF)
            estado_atual = OFF;
            break;
        }

        // PEDIDO DE DRENO COM EQUIPAMENTO LIGADO
        if (dreno_solicitado && (P80 != 0))
        {
            // Não vamos iniciar o dreno agora,
            // primeiro fazemos o desligamento normal:
            dreno_solicitado = false;
            dreno_pendente   = true;   // marca para drenar após desligar

            // Equivalente a apertar ONOFF para iniciar rotina de desligar
            flag_onoff   = false;
            estado_atual = OFF;
            break;
        }

        // Se apertar ONOFF, inicia rotina de desligar
        if (flag_onoff)
        {
            flag_onoff   = false;
            estado_atual = OFF;
            break;
        }

        // Motor permanece ligado o tempo todo neste estado
        HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);

        if (P82 == 0)
        {
            // Sem bomba: mantém sempre desligada
            HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
        }

        else
        {
            if (!bomba_habilitada)
            {
                // Usuário forçou bomba OFF pelo botão
                HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
            }
            else
            {
                if (sensor_estavel == 1)
                {
                    HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, ligar_bomba);
                }
                else
                {
                    HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
                }
            }
        }
        break;

    case OFF:

    	flag_exaustor = false;
        // Ignora novos cliques de dreno durante o desligamento
        dreno_solicitado = false;

    	if (flag_onoff) flag_onoff;
        // Ao entrar em OFF:
        // 1) desliga a bomba imediatamente
        HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
        // garante que o swing também fique desligado
        HAL_GPIO_WritePin(SWING_GPIO_Port, SWING_Pin, desligar_swing);

        if (P82 == 0)
        {
            // Sem bomba → não faz sentido ts, desliga motor direto
            HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);
            motor_desligado = false;
            estado_atual = VENTILADOR_OFF;
        }

        else
        {
            if (!bomba_habilitada)
            {
                // Usuário já tinha forçado a bomba OFF pelo botão
                // → não faz sentido secar, desliga motor direto
                HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);
                motor_desligado = false;
                estado_atual = VENTILADOR_OFF;
            }

            else if (sensor_estavel == 0)
            {
                // Já não tem água -> não precisa secar painel
                // desliga motor imediatamente
                HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);
                motor_desligado = false;
                // Vai direto para VENTILADOR_OFF (que depois volta pro IDLE)
                estado_atual = VENTILADOR_OFF;
            }
            else
            {
                // Ainda tem água no painel -> faz secagem com ts
                ts = P31 * 60;

                // Define a frequência de secagem (fsec)
                // Se P32 == 0 -> usa a frequência atual (fatual)
                // Se P32 != 0 -> usa o valor configurado em P32
                if (P32 == 0)
                {
                    fsec = fatual;
                }
                else
                {
                    fsec = P32;   // aqui você garante que P32 foi validado entre fmin..fmax em outro lugar
                }

                if (P33 == 1)
                {
                    // Precisa inverter o giro antes de secar
                    inversao_pendente = true;

                    // Desliga motor e espera parar
                    HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);
                    motor_desligado = false;

                    estado_atual = EXAUSTAO_INVERSAO;
                }
                else
                {
                    // Secagem normal
                    motor_sentido = P51;  // garante sentido normal
                    estado_atual = BOMBA_OFF;
                }

            }
        }
        break;

    case EXAUSTAO_INVERSAO:

        // Garantir motor desligado
        HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);

        // Espera o motor parar mecanicamente
        if (!motor_desligado) break;

        // Agora inverter sentido
        motor_sentido = !P51;

        ts = P31 * 60;
        // Liga motor no sentido invertido
        HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);

        // Agora podemos começar a secagem (ts já está setado)
        estado_atual = EXAUSTAO_SECAR;
        break;

    case EXAUSTAO_SECAR:

        // Secagem com sentido invertido
        if (ts > 0)
        {
            HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);
        }
        else
        {
            // Acabou TS → desliga
            HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);
            motor_desligado = false;

            inversao_pendente = false;

            estado_atual = VENTILADOR_OFF;
        }
        break;

    case BOMBA_OFF:
    {
    	flag_exaustor = false;
        // Ignora novos cliques de dreno durante o desligamento
        dreno_solicitado = false;

    	if (flag_onoff) flag_onoff = false;
        // Garante bomba OFF durante toda a secagem
        HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);

        // Enquanto ts > 0, mantém o motor ligado para secar o painel
        if (ts > 0)
        {
            HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);
        }
        else
        {
            // Acabou o tempo de secagem: desliga o motor
            HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);
            motor_desligado = false;
            // Segue para o próximo estado da rotina de desligamento
            estado_atual = VENTILADOR_OFF;
        }
    }
    break;

    case VENTILADOR_OFF:

        // Nenhum botão é válido aqui
        if (flag_onoff)       flag_onoff       = false;
        if (dreno_solicitado) dreno_solicitado = false;
        if (flag_bomba)       flag_bomba       = false;
        if (flag_swing)       flag_swing       = false;
        // garante que o swing também fique desligado
        HAL_GPIO_WritePin(SWING_GPIO_Port, SWING_Pin, desligar_swing);

        if (dreno_pendente && (P80 != 0))
        {
            dreno_pendente = false;   // vamos atender agora

            // Liga DRENO pelo tempo P83
            HAL_GPIO_WritePin(DRENO_GPIO_Port, DRENO_Pin, ligar_dreno);
            td          = P83 * 60;
            estado_atual = DRENO_ON;
        }
        else if (exaustor_pendente && (P86 != 0))
        {
            // Espera a variável motor_desligado indicar que o motor
            // realmente parou mecanicamente
            if (motor_desligado)
            {
                exaustor_pendente = false;

                // Liga somente o motor no modo exaustão
                HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);
                // bomba fica sempre OFF neste modo
                HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
                motor_desligado = false;
                estado_atual = EXAUSTOR_ON;
            }
        }
        else
        {
            // Sem dreno pendente → volta direto para IDLE
            estado_atual = IDLE;
            // sempre voltar ao sentido nominal
            motor_sentido = P51;
        }
    break;

    case DRENO_ON:

        // Nenhum botão é válido durante o ciclo de dreno
        if (flag_onoff)       flag_onoff       = false;
        if (dreno_solicitado) dreno_solicitado = false;
        if (flag_bomba)       flag_bomba       = false;
        if (flag_swing)       flag_swing       = false;
        if (flag_exaustor)    flag_exaustor    = false;

        // Equipamento "principal" fica desligado
        HAL_GPIO_WritePin(MOTOR_GPIO_Port,  MOTOR_Pin,  desligar_motor);
        HAL_GPIO_WritePin(BOMBA_GPIO_Port,  BOMBA_Pin,  desligar_bomba);
        HAL_GPIO_WritePin(SWING_GPIO_Port,  SWING_Pin,  desligar_swing);

        if (td == 0)
        {
            // Tempo P83 acabou → desliga dreno
            HAL_GPIO_WritePin(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);

            td_off       = P84 * 60;         // tempo com dreno desligado
            estado_atual = DRENO_OFF_DELAY;
        }
        break;


    case DRENO_OFF_DELAY:

        // Nenhum botão é válido aqui também
        if (flag_onoff)       flag_onoff       = false;
        if (dreno_solicitado) dreno_solicitado = false;
        if (flag_bomba)       flag_bomba       = false;
        if (flag_swing)       flag_swing       = false;
        if (flag_exaustor)    flag_exaustor    = false;

        // Mantém tudo desligado enquanto conta P84
        HAL_GPIO_WritePin(MOTOR_GPIO_Port,  MOTOR_Pin,  desligar_motor);
        HAL_GPIO_WritePin(BOMBA_GPIO_Port,  BOMBA_Pin,  desligar_bomba);
        HAL_GPIO_WritePin(SWING_GPIO_Port,  SWING_Pin,  desligar_swing);

        if (td_off == 0)
        {
            // Fim do ciclo de dreno → volta para IDLE
            estado_atual = IDLE;
        }
        break;
    case EXAUSTOR_ON:

        // Modo exaustão:
        // - Motor ligado (sentido reverso é físico)
        // - Bomba sempre desligada
        // - Swing pode funcionar normalmente (já permitido pelo equipamento_ligado)

    	// Sentido invertido = !P51
    	motor_sentido = !P51;
    	HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor);
        HAL_GPIO_WritePin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);

        // Dreno, ONOFF e Bomba não têm efeito aqui
        if (dreno_solicitado) dreno_solicitado = false;
        if (flag_onoff)       flag_onoff       = false;
        if (flag_bomba)       flag_bomba       = false;

        // Se apertar EXAUSTOR novamente, desliga motor e aguarda motor_desligado
        if (flag_exaustor)
        {
            flag_exaustor = false;

            HAL_GPIO_WritePin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor);
            motor_desligado = false;
            // Agora espera motor_desligado == true para voltar ao IDLE
            estado_atual = EXAUSTOR_WAIT;
        }
        break;

    case EXAUSTOR_WAIT:

        // Garante tudo desligado
        HAL_GPIO_WritePin(MOTOR_GPIO_Port,  MOTOR_Pin,  desligar_motor);
        HAL_GPIO_WritePin(BOMBA_GPIO_Port,  BOMBA_Pin,  desligar_bomba);
        HAL_GPIO_WritePin(SWING_GPIO_Port,  SWING_Pin,  desligar_swing);
        HAL_GPIO_WritePin(DRENO_GPIO_Port,  DRENO_Pin,  desligar_dreno);

        // Nenhum botão é válido aqui
        if (flag_onoff)       flag_onoff       = false;
        if (flag_bomba)       flag_bomba       = false;
        if (flag_swing)       flag_swing       = false;
        if (dreno_solicitado) dreno_solicitado = false;
        if (flag_exaustor)    flag_exaustor    = false;

        // Quando a variável indicar motor totalmente parado,
        // voltamos ao estado IDLE
        if (motor_desligado)
        {
        	motor_sentido = P51;   // **restaurar direção nominal**
            estado_atual = IDLE;
        }
        break;
    }

}
