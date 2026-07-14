#include "flash_storage.h"

#include "iwdg.h"
#include "stm32f4xx_hal.h"

#define FLASH_STORAGE_ADDR_A   0x08040000UL
#define FLASH_STORAGE_ADDR_B   0x08060000UL
#define FLASH_STORAGE_SECTOR_A FLASH_SECTOR_6
#define FLASH_STORAGE_SECTOR_B FLASH_SECTOR_7

static uint32_t watchdog_saved_prescaler;
static uint32_t watchdog_saved_reload;

uint8_t FlashStorage_WatchdogEnterMaintenance(void)
{
    watchdog_saved_prescaler = hiwdg.Init.Prescaler;
    watchdog_saved_reload    = hiwdg.Init.Reload;
    hiwdg.Init.Prescaler     = IWDG_PRESCALER_256;
    hiwdg.Init.Reload        = 4095U;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        hiwdg.Init.Prescaler = watchdog_saved_prescaler;
        hiwdg.Init.Reload    = watchdog_saved_reload;
        return 0U;
    }
    return 1U;
}

uint8_t FlashStorage_WatchdogExitMaintenance(void)
{
    hiwdg.Init.Prescaler = watchdog_saved_prescaler;
    hiwdg.Init.Reload    = watchdog_saved_reload;
    return (HAL_IWDG_Init(&hiwdg) == HAL_OK) ? 1U : 0U;
}

const uint8_t *FlashStorage_SlotData(uint8_t slot)
{
    uint32_t address = (slot == 0U) ? FLASH_STORAGE_ADDR_A : FLASH_STORAGE_ADDR_B;
    return (const uint8_t *)address;
}

uint8_t FlashStorage_Unlock(void)
{
    HAL_IWDG_Refresh(&hiwdg);
    return (HAL_FLASH_Unlock() == HAL_OK) ? 1U : 0U;
}

void FlashStorage_Lock(void)
{
    (void)HAL_FLASH_Lock();
    HAL_IWDG_Refresh(&hiwdg);
}

uint8_t FlashStorage_EraseSlot(uint8_t slot)
{
    FLASH_EraseInitTypeDef erase        = {0};
    uint32_t               sector_error = 0UL;
    HAL_StatusTypeDef      status;

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = (slot == 0U) ? FLASH_STORAGE_SECTOR_A : FLASH_STORAGE_SECTOR_B;
    erase.NbSectors    = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    status             = HAL_FLASHEx_Erase(&erase, &sector_error);
    return (status == HAL_OK && sector_error == 0xFFFFFFFFUL) ? 1U : 0U;
}

uint8_t FlashStorage_ProgramWord(uint8_t slot, size_t offset, uint32_t word)
{
    uint32_t address = ((slot == 0U) ? FLASH_STORAGE_ADDR_A : FLASH_STORAGE_ADDR_B) + (uint32_t)offset;
    return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word) == HAL_OK) ? 1U : 0U;
}
