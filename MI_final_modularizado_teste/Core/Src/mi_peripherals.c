#include "mi_peripherals.h"
#include "mi_sensors.h"

static void set_output_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    HAL_GPIO_WritePin(port, pin, state);
}

void mi_periph_set_modes(void) {
    ligar_motor_pin = GPIO_PIN_SET;
    desligar_motor_pin = GPIO_PIN_RESET;

    /* BOMBA: P82=1 -> NA, P82=2 -> NF, P82=0 -> bloqueada logicamente */
    ligar_bomba    = (P82 == 2u) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    desligar_bomba = (P82 == 2u) ? GPIO_PIN_SET   : GPIO_PIN_RESET;

    /* SWING: 0=desabilitado, 1=NA, 2=NF */
    if (P81 == 0u) {
        ligar_swing    = GPIO_PIN_RESET;
        desligar_swing = GPIO_PIN_RESET;
    } else if (P81 == 2u) {
        ligar_swing    = GPIO_PIN_RESET;
        desligar_swing = GPIO_PIN_SET;
    } else {
        ligar_swing    = GPIO_PIN_SET;
        desligar_swing = GPIO_PIN_RESET;
    }

    /* DRENO: a polaridade física pode ser mantida padrão; P80 define o modo e não a polaridade */
    ligar_dreno    = GPIO_PIN_SET;
    desligar_dreno = GPIO_PIN_RESET;
}

uint8_t mi_sensor_read(void) {
    if (P85 == 0u) {
        return 0u; /* sensor desabilitado = sem confirmação de água */
    }

    GPIO_PinState raw = HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin);
    if (P85 == 1u) {
        return (raw == GPIO_PIN_SET) ? 1u : 0u; /* NA */
    }
    return (raw == GPIO_PIN_RESET) ? 1u : 0u;   /* NF */
}

void mi_periph_tick_1s(void) {
    uint8_t sensor_raw = mi_sensor_read();

    if (sensor_raw != sensor_ultimo_raw) {
        sensor_ultimo_raw = sensor_raw;
        sensor_delay_counter = tsensor;
    } else {
        if (sensor_delay_counter > 0u) {
            sensor_delay_counter--;
        } else {
            sensor_estavel = sensor_raw;
        }
    }
}

void mi_comm_safe_stop(void) {
    remote_start_latched = false;
    cmd_ligar_motor = false;
    set_output_pin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
    set_output_pin(SWING_GPIO_Port, SWING_Pin, desligar_swing);
    set_output_pin(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);
    if (f_atual <= 0.1f) {
        set_output_pin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor_pin);
    }
}

void mi_comm_apply_received_command(void) {
    uint8_t btn = g_cmd.buttons;

    if (btn & MI_BTN_BIT_START) remote_start_latched = true;
    if (btn & MI_BTN_BIT_STOP)  remote_start_latched = false;

    remote_system_on    = ((g_cmd.aux_flags & MI_AUX_BIT_SYSTEM_ON) != 0u);
    remote_bomba_cmd    = ((g_cmd.aux_flags & MI_AUX_BIT_BOMBA) != 0u);
    remote_swing_cmd    = ((g_cmd.aux_flags & MI_AUX_BIT_SWING) != 0u);
    remote_exaustao_cmd = ((g_cmd.aux_flags & MI_AUX_BIT_EXAUSTAO) != 0u);
    remote_dreno_status = (g_cmd.dreno_status == 0u) ? MI_DRENO_IDLE :
                          (g_cmd.dreno_status == 1u) ? MI_DRENO_AGUARDANDO_LED : MI_DRENO_EM_CURSO;

    if (!remote_system_on) {
        remote_start_latched = false;
    }

    P51 = (g_cmd.direction != 0u) ? 1u : 0u;
    cmd_frequencia_alvo = ((float)g_cmd.target_freq_centi_hz) / 100.0f;
}

void mi_periph_apply_remote(void) {
    bool dreno_ativo = (remote_dreno_status != MI_DRENO_IDLE) && (P80 != 0u);
    bool temp_trip = g_sensors.temp_fault;
    bool permite_motor = hardware_comms_ok && handshake_done &&
                         !g_cmd.e08_active &&
                         !temp_trip &&
                         remote_system_on && remote_start_latched &&
                         !dreno_ativo;
    bool motor_ativo = permite_motor || (f_atual > 0.1f);

    /* P85=0 bloqueia confirmação de água e, portanto, climatizar */
    bool bomba_permitida = remote_system_on && remote_bomba_cmd && !remote_exaustao_cmd && !dreno_ativo && (P82 != 0u) && (sensor_estavel == 1u);
    bool swing_permitido = motor_ativo && remote_swing_cmd && !dreno_ativo && (P81 != 0u) && !remote_exaustao_cmd;

    if (!hardware_comms_ok || !handshake_done || g_cmd.e08_active || temp_trip) {
        dreno_ativo = false;
        bomba_permitida = false;
        swing_permitido = false;
        permite_motor = false;
        motor_ativo = false;
    }

    if (dreno_ativo) {
        set_output_pin(DRENO_GPIO_Port, DRENO_Pin, ligar_dreno);
        set_output_pin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
        set_output_pin(SWING_GPIO_Port, SWING_Pin, desligar_swing);
        cmd_ligar_motor = false;
    } else {
        set_output_pin(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);
        set_output_pin(BOMBA_GPIO_Port, BOMBA_Pin, bomba_permitida ? ligar_bomba : desligar_bomba);
        set_output_pin(SWING_GPIO_Port, SWING_Pin, swing_permitido ? ligar_swing : desligar_swing);
        cmd_ligar_motor = permite_motor;
    }

    motor_reverse = remote_exaustao_cmd ? !((bool)P51) : ((bool)P51);

    if (cmd_ligar_motor || f_atual > 0.1f) {
        set_output_pin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor_pin);
        motor_desligado = false;
    } else {
        set_output_pin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor_pin);
        motor_desligado = true;
    }
}

void mi_periph_init_runtime(void) {
    mi_periph_set_modes();
}
