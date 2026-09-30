#include "flash_program.h"

#include "stm32f1xx_hal.h"
#include "dem_event_adapter.h"
#include "boot_watchdog.h"

#include <string.h>

bool FlashProgram_IsRangeValid(uint32_t address, uint32_t length)
{
    return ImageLayout_IsRangeInsideSlot(IMAGE_SLOT_A_ID,address,length)||
           ImageLayout_IsRangeInsideSlot(IMAGE_SLOT_B_ID,address,length);
}

bool FlashProgram_IsRangeValidForSlot(uint8_t slot_id,uint32_t address,uint32_t length)
{
    return ImageLayout_IsRangeInsideSlot(slot_id,address,length);
}

FlashProgram_StatusType FlashProgram_EraseSlot(uint8_t slot_id)
{
    FLASH_EraseInitTypeDef erase_init = {0};
    uint32_t page_error = 0U;
    HAL_StatusTypeDef status;

    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    if (!ImageLayout_IsSlotId(slot_id)) {
        return FLASH_PROGRAM_INVALID_ARG;
    }
    HAL_FLASH_Unlock();
    status = HAL_OK;
    erase_init.NbPages = 1U;
    for (uint32_t page = 0U; page < IMAGE_SLOT_PAGE_COUNT; page++) {
        erase_init.PageAddress = ImageLayout_GetSlotBase(slot_id) +
                                 page * IMAGE_FLASH_PAGE_SIZE;
        if (HAL_FLASHEx_Erase(&erase_init, &page_error) != HAL_OK) {
            status = HAL_ERROR;
            break;
        }
        BootWatchdog_Refresh();
    }
    HAL_FLASH_Lock();

    DemEventAdapter_ReportFlashEraseResult(status == HAL_OK,
                                           HAL_GetTick());

    return (status == HAL_OK) ? FLASH_PROGRAM_OK :
                                FLASH_PROGRAM_HAL_ERROR;
}

FlashProgram_StatusType FlashProgram_Verify(uint32_t address,
                                            const uint8_t *data,
                                            uint32_t length)
{
    if (!FlashProgram_IsRangeValid(address, length) ||
        ((data == NULL) && (length > 0U))) {
        return FLASH_PROGRAM_INVALID_ARG;
    }
    if (length == 0U) {
        /* Preserve the legacy no-op result without fabricating a PASSED test. */
        return FLASH_PROGRAM_OK;
    }

    if (memcmp((const void *)address, data, length) != 0) {
        DemEventAdapter_ReportFlashVerifyResult(false, HAL_GetTick());
        return FLASH_PROGRAM_VERIFY_ERROR;
    }

    DemEventAdapter_ReportFlashVerifyResult(true, HAL_GetTick());

    return FLASH_PROGRAM_OK;
}

FlashProgram_StatusType FlashProgram_Write(uint32_t address,
                                           const uint8_t *data,
                                           uint32_t length)
{
    uint32_t offset;

    if (!FlashProgram_IsRangeValid(address, length) ||
        ((data == NULL) && (length > 0U))) {
        return FLASH_PROGRAM_INVALID_ARG;
    }
    if ((length & 1U) != 0U) {
        return FLASH_PROGRAM_ALIGNMENT_ERROR;
    }
    if (length == 0U) {
        /* Preserve the legacy no-op result without fabricating a PASSED test. */
        return FLASH_PROGRAM_OK;
    }

    HAL_FLASH_Unlock();

    for (offset = 0U; offset < length; offset += 2U) {
        uint16_t halfword = (uint16_t)data[offset] |
                            ((uint16_t)data[offset + 1U] << 8);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              address + offset,
                              halfword) != HAL_OK) {
            HAL_FLASH_Lock();
            DemEventAdapter_ReportFlashProgramResult(false, HAL_GetTick());
            return FLASH_PROGRAM_HAL_ERROR;
        }

        if (*(volatile uint16_t *)(address + offset) != halfword) {
            HAL_FLASH_Lock();
            DemEventAdapter_ReportFlashVerifyResult(false, HAL_GetTick());
            return FLASH_PROGRAM_VERIFY_ERROR;
        }
        if ((offset & 0x3FU) == 0U) BootWatchdog_Refresh();
    }

    HAL_FLASH_Lock();
    DemEventAdapter_ReportFlashProgramResult(true, HAL_GetTick());
    DemEventAdapter_ReportFlashVerifyResult(true, HAL_GetTick());
    return FLASH_PROGRAM_OK;
}

FlashProgram_StatusType FlashProgram_Read(uint32_t address,
                                          uint8_t *buffer,
                                          uint32_t length)
{
    if (!FlashProgram_IsRangeValid(address, length) ||
        ((buffer == NULL) && (length > 0U))) {
        return FLASH_PROGRAM_INVALID_ARG;
    }

    if (length > 0U) {
        memcpy(buffer, (const void *)address, length);
    }

    return FLASH_PROGRAM_OK;
}
