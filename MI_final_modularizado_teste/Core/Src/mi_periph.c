#include "mi_periph.h"
#include "mi_state.h"
#include "mi_config.h"
#include "main.h"

static void mi_set_output(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

void mi_periph_set_modes(void)
{
    ligar_motor_pin = GPIO_PIN_SET;
    desligar_motor_pin = GPIO_PIN_RESET;

    ligar_bomba    = (P82 == 2u) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    desligar_bomba = (P82 == 2u) ? GPIO_PIN_SET   : GPIO_PIN_RESET;

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

    ligar_dreno    = GPIO_PIN_SET;
    desligar_dreno = GPIO_PIN_RESET;
}

uint8_t mi_periph_sensor_read(void)
{
    if (P85 == 0u) {
        return 0u;
    }

    GPIO_PinState raw = HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin);
    if (P85 == 1u) {
        return (raw == GPIO_PIN_SET) ? 1u : 0u;
    }

    return (raw == GPIO_PIN_RESET) ? 1u : 0u;
}

void mi_periph_tick_1s(void)
{
    uint8_t sensor_raw = mi_periph_sensor_read();

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

void mi_periph_safe_stop(void)
{
    remote_start_latched = false;
    cmd_ligar_motor = false;

    mi_set_output(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
    mi_set_output(SWING_GPIO_Port, SWING_Pin, desligar_swing);
    mi_set_output(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);

    if (f_atual <= 0.1f) {
        mi_set_output(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor_pin);
    }
}

void mi_periph_apply_remote(void)
{
    bool dreno_ativo = (remote_dreno_status != DRENO_IDLE) && (P80 != 0u);
    bool sensor_ok = (P85 != 0u) && (sensor_estavel == 1u);

    bool permite_motor =
        hardware_comms_ok &&
        handshake_done &&
        !g_cmd.e08_active &&
        remote_system_on &&
        remote_start_latched &&
        !dreno_ativo;

    bool motor_ativo = permite_motor || (f_atual > 0.1f);

    bool bomba_permitida =
        remote_system_on &&
        remote_bomba_cmd &&
        !remote_exaustao_cmd &&
        !dreno_ativo &&
        (P82 != 0u) &&
        sensor_ok;

    bool swing_permitido =
        motor_ativo &&
        remote_swing_cmd &&
        !remote_exaustao_cmd &&
        !dreno_ativo &&
        (P81 != 0u);

    if (!hardware_comms_ok || !handshake_done || g_cmd.e08_active) {
        dreno_ativo = false;
        bomba_permitida = false;
        swing_permitido = false;
        permite_motor = false;
        motor_ativo = false;
    }

    if (dreno_ativo) {
        mi_set_output(DRENO_GPIO_Port, DRENO_Pin, ligar_dreno);
        mi_set_output(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
        mi_set_output(SWING_GPIO_Port, SWING_Pin, desligar_swing);
        cmd_ligar_motor = false;
    } else {
        mi_set_output(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);
        mi_set_output(BOMBA_GPIO_Port, BOMBA_Pin, bomba_permitida ? ligar_bomba : desligar_bomba);
        mi_set_output(SWING_GPIO_Port, SWING_Pin, swing_permitido ? ligar_swing : desligar_swing);
        cmd_ligar_motor = permite_motor;
    }

    motor_reverse = remote_exaustao_cmd ? !((bool)P51) : ((bool)P51);

    if (cmd_ligar_motor || f_atual > 0.1f) {
        mi_set_output(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor_pin);
        motor_desligado = false;
    } else {
        mi_set_output(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor_pin);
        motor_desligado = true;
    }
}
