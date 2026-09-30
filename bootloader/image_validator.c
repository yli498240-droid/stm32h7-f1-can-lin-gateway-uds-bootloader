#include "image_validator.h"
#include "boot_watchdog.h"

#include <stddef.h>

bool ImageValidator_IsImageRangeValid(uint32_t image_base,
                                      uint32_t image_size)
{
    return ImageValidator_IsImageRangeValidForSlot(IMAGE_SLOT_A_ID,
                                                   image_base,
                                                   image_size) ||
           ImageValidator_IsImageRangeValidForSlot(IMAGE_SLOT_B_ID,
                                                   image_base,
                                                   image_size);
}

bool ImageValidator_IsImageRangeValidForSlot(uint8_t slot_id,
                                             uint32_t image_base,
                                             uint32_t image_size)
{
    return (image_base == ImageLayout_GetSlotBase(slot_id)) &&
           ImageLayout_IsRangeInsideSlot(slot_id, image_base, image_size);
}

bool ImageValidator_ValidateVectorForImage(uint32_t initial_msp,
                                           uint32_t reset_handler,
                                           uint32_t image_base,
                                           uint32_t image_size)
{
    uint32_t reset_address = reset_handler & ~1U;
    uint32_t image_end;
    bool msp_valid;
    bool reset_valid;

    if (!ImageValidator_IsImageRangeValid(image_base, image_size)) {
        return false;
    }

    image_end = image_base + image_size;
    msp_valid = (initial_msp >= IMAGE_SRAM_START) &&
                (initial_msp < IMAGE_SRAM_END_EXCLUSIVE) &&
                ((initial_msp & 0x3U) == 0U);
    reset_valid = ((reset_handler & 1U) == 1U) &&
                  (reset_address >= image_base) &&
                  (reset_address < image_end);

    return msp_valid && reset_valid;
}

bool ImageValidator_ValidateVectorValues(uint32_t initial_msp,
                                         uint32_t reset_handler)
{
    return ImageValidator_ValidateVectorForImage(initial_msp,
                                                 reset_handler,
                                                 IMAGE_SLOT_A_BASE,
                                                 IMAGE_SLOT_SIZE);
}

bool ImageValidator_ReadAndValidateSlotVector(uint8_t slot_id,
                                              uint32_t image_size,
                                              ImageValidator_VectorType *vector)
{
    ImageValidator_VectorType local_vector;
    uint32_t image_base = ImageLayout_GetSlotBase(slot_id);

    if (!ImageValidator_IsImageRangeValidForSlot(slot_id,
                                                 image_base,
                                                 image_size)) {
        return false;
    }

    local_vector.initial_msp =
        *(volatile const uint32_t *)image_base;
    local_vector.reset_handler =
        *(volatile const uint32_t *)(image_base + 4U);

    if (vector != NULL) {
        *vector = local_vector;
    }

    return ImageValidator_ValidateVectorForImage(local_vector.initial_msp,
                                                 local_vector.reset_handler,
                                                 image_base,
                                                 image_size);
}

uint32_t ImageValidator_CalculateCrc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t index;

    if ((data == NULL) && (length > 0U)) {
        return 0U;
    }

    for (index = 0U; index < length; index++) {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            } else {
                crc >>= 1U;
            }
        }
        if ((index & 0x3FFU) == 0U) BootWatchdog_Refresh();
    }

    return ~crc;
}

bool ImageValidator_VerifyCrc32(const uint8_t *data,
                                uint32_t length,
                                uint32_t expected_crc,
                                uint32_t *actual_crc)
{
    uint32_t calculated_crc =
        ImageValidator_CalculateCrc32(data, length);

    if (actual_crc != NULL) {
        *actual_crc = calculated_crc;
    }

    return calculated_crc == expected_crc;
}
