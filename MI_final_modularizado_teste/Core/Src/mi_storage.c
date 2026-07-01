#include "mi_storage.h"
#include "mi_state.h"
#include "mi_config.h"
#include <string.h>
#include <math.h>

void mi_storage_load(void)
{
    const mi_system_params_t *flash_data = (const mi_system_params_t *)MI_FLASH_USER_ADDR;

    if (flash_data->saved_P12 == 0xFFu || flash_data->saved_P10 == 0xFFu) {
        return;
    }

    if (!isnan(flash_data->saved_freq_hz)) {
        cmd_frequencia_alvo = flash_data->saved_freq_hz;
    }

    P10 = (uint32_t)flash_data->saved_P10;
    P11 = (uint32_t)flash_data->saved_P11;
    P20 = flash_data->saved_P20;
    P21 = flash_data->saved_P21;
    P35 = flash_data->saved_P35;
    P42 = flash_data->saved_P42;
    P12 = flash_data->saved_P12;
    params_locked = (flash_data->saved_locked == 1u);
}

void mi_storage_write(void)
{
    mi_system_params_t data;
    uint32_t page_error = 0u;

    data.saved_freq_hz = cmd_frequencia_alvo;
    data.saved_P10 = (uint8_t)P10;
    data.saved_P11 = (uint8_t)P11;
    data.saved_P20 = P20;
    data.saved_P21 = P21;
    data.saved_P35 = P35;
    data.saved_P42 = P42;
    data.saved_P12 = P12;
    data.saved_locked = params_locked ? 1u : 0u;
    data.padding = 0u;

    __disable_irq();
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
#if defined(FLASH_TYPEERASE_PAGES)
#if defined(STM32F301x8) || defined(STM32F3)
    erase.PageAddress = MI_FLASH_USER_ADDR;
#else
    erase.PageAddress = MI_FLASH_USER_ADDR;
#endif
#endif
    erase.NbPages = 1u;

    if (HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK) {
        const uint8_t *p = (const uint8_t *)&data;
        uint32_t addr = MI_FLASH_USER_ADDR;

        for (uint32_t i = 0u; i < sizeof(data); i += 2u) {
            uint16_t half = 0xFFFFu;
            if ((i + 1u) < sizeof(data)) {
                half = (uint16_t)p[i] | ((uint16_t)p[i + 1u] << 8);
            } else {
                half = (uint16_t)p[i] | 0xFF00u;
            }

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, half) != HAL_OK) {
                break;
            }
        }
    }

    HAL_FLASH_Lock();
    __enable_irq();
}
