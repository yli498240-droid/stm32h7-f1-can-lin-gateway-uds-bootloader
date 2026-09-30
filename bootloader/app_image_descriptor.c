#include "app_image_descriptor.h"

#include "image_validator.h"

#include <stddef.h>
#include <string.h>

typedef char AppImageDescriptor_SizeMustBe32Bytes[
    (sizeof(AppImageDescriptorType) == APP_IMAGE_DESCRIPTOR_SIZE) ? 1 : -1];

static bool AppImageDescriptor_IsCoveredByImage(uint32_t image_base,
                                                uint32_t image_size)
{
    uint32_t image_end;
    uint32_t descriptor_address;
    uint8_t slot_id;

    if (!ImageValidator_IsImageRangeValid(image_base, image_size)) {
        return false;
    }

    slot_id = (image_base == IMAGE_SLOT_A_BASE) ? IMAGE_SLOT_A_ID :
              (image_base == IMAGE_SLOT_B_BASE) ? IMAGE_SLOT_B_ID :
              IMAGE_SLOT_NONE;
    descriptor_address = ImageLayout_GetDescriptorAddress(slot_id);

    image_end = image_base + image_size;
    return (descriptor_address >= image_base) &&
           (descriptor_address < image_end) &&
           (APP_IMAGE_DESCRIPTOR_SIZE <=
            (image_end - descriptor_address));
}

bool AppImageDescriptor_ReadAndValidate(uint32_t image_base,
                                        uint32_t image_size,
                                        AppImageDescriptorType *descriptor)
{
    AppImageDescriptorType local_descriptor;
    uint32_t actual_crc;

    if (!AppImageDescriptor_IsCoveredByImage(image_base, image_size)) {
        return false;
    }

    memcpy(&local_descriptor,
           (const void *)(image_base + APP_IMAGE_DESCRIPTOR_OFFSET),
           sizeof(local_descriptor));

    actual_crc = ImageValidator_CalculateCrc32(
        (const uint8_t *)&local_descriptor,
        (uint32_t)offsetof(AppImageDescriptorType, descriptor_crc32));

    if ((local_descriptor.magic != APP_IMAGE_DESCRIPTOR_MAGIC) ||
        (local_descriptor.format_version !=
         APP_IMAGE_DESCRIPTOR_FORMAT_VERSION) ||
        (local_descriptor.descriptor_size != APP_IMAGE_DESCRIPTOR_SIZE) ||
        (local_descriptor.image_target != APP_IMAGE_TARGET_RC_WINDOW) ||
        (local_descriptor.app_base != image_base) ||
        (local_descriptor.reserved0 != 0U) ||
        (local_descriptor.reserved1 != 0U) ||
        (local_descriptor.descriptor_crc32 != actual_crc)) {
        return false;
    }

    if (descriptor != NULL) {
        *descriptor = local_descriptor;
    }

    return true;
}
