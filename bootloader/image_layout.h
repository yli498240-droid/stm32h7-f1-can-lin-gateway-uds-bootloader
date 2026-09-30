#ifndef IMAGE_LAYOUT_H
#define IMAGE_LAYOUT_H

#include <stdbool.h>
#include <stdint.h>

#define IMAGE_FLASH_BASE                 0x08000000U
#define IMAGE_FLASH_END_EXCLUSIVE        0x08040000U
#define IMAGE_FLASH_PAGE_SIZE            0x00000800U

#define IMAGE_SLOT_A_ID                  0x00U
#define IMAGE_SLOT_B_ID                  0x01U
#define IMAGE_SLOT_NONE                  0xFFU

#define IMAGE_SLOT_A_BASE                0x08008000U
#define IMAGE_SLOT_A_END_EXCLUSIVE       0x08023800U
#define IMAGE_SLOT_B_BASE                0x08023800U
#define IMAGE_SLOT_B_END_EXCLUSIVE       0x0803F000U
#define IMAGE_SLOT_SIZE                  0x0001B800U
#define IMAGE_SLOT_PAGE_COUNT            55U

#define APP_IMAGE_DESCRIPTOR_OFFSET      0x00000200U
#define APP_IMAGE_DESCRIPTOR_SIZE        32U

#define IMAGE_METADATA_PAGE_A_ADDRESS    0x0803F000U
#define IMAGE_METADATA_PAGE_B_ADDRESS    0x0803F800U
#define IMAGE_METADATA_PAGE_SIZE         0x00000800U
#define IMAGE_METADATA_END_EXCLUSIVE     0x08040000U

static inline bool ImageLayout_IsSlotId(uint8_t slot_id)
{
    return (slot_id == IMAGE_SLOT_A_ID) ||
           (slot_id == IMAGE_SLOT_B_ID);
}

static inline uint32_t ImageLayout_GetSlotBase(uint8_t slot_id)
{
    return (slot_id == IMAGE_SLOT_A_ID) ? IMAGE_SLOT_A_BASE :
           (slot_id == IMAGE_SLOT_B_ID) ? IMAGE_SLOT_B_BASE : 0U;
}

static inline uint32_t ImageLayout_GetSlotEndExclusive(uint8_t slot_id)
{
    return (slot_id == IMAGE_SLOT_A_ID) ? IMAGE_SLOT_A_END_EXCLUSIVE :
           (slot_id == IMAGE_SLOT_B_ID) ? IMAGE_SLOT_B_END_EXCLUSIVE : 0U;
}

static inline uint32_t ImageLayout_GetDescriptorAddress(uint8_t slot_id)
{
    uint32_t base = ImageLayout_GetSlotBase(slot_id);
    return (base == 0U) ? 0U : (base + APP_IMAGE_DESCRIPTOR_OFFSET);
}

static inline uint8_t ImageLayout_GetInactiveSlot(uint8_t slot_id)
{
    return (slot_id == IMAGE_SLOT_A_ID) ? IMAGE_SLOT_B_ID :
           (slot_id == IMAGE_SLOT_B_ID) ? IMAGE_SLOT_A_ID : IMAGE_SLOT_NONE;
}

static inline bool ImageLayout_IsRangeInsideSlot(uint8_t slot_id,
                                                  uint32_t address,
                                                  uint32_t length)
{
    uint32_t base = ImageLayout_GetSlotBase(slot_id);
    uint32_t end = ImageLayout_GetSlotEndExclusive(slot_id);

    return (base != 0U) && (length != 0U) &&
           (address >= base) && (address < end) &&
           (length <= (end - address));
}

#endif /* IMAGE_LAYOUT_H */
