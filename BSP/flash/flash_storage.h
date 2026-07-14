#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stddef.h>
#include <stdint.h>

uint8_t        FlashStorage_WatchdogEnterMaintenance(void);
uint8_t        FlashStorage_WatchdogExitMaintenance(void);
const uint8_t *FlashStorage_SlotData(uint8_t slot);
uint8_t        FlashStorage_Unlock(void);
void           FlashStorage_Lock(void);
uint8_t        FlashStorage_EraseSlot(uint8_t slot);
uint8_t        FlashStorage_ProgramWord(uint8_t slot, size_t offset, uint32_t word);

#endif
