#ifndef IMAGE_METADATA_H
#define IMAGE_METADATA_H

#include <stdbool.h>
#include <stdint.h>
#include "image_layout.h"

#define IMAGE_METADATA_MAGIC 0x4D31314DU
#define IMAGE_METADATA_FORMAT_VERSION_V1 1U
#define IMAGE_METADATA_FORMAT_VERSION_V2 2U
#define IMAGE_METADATA_RECORD_SIZE 64U
#define IMAGE_METADATA_COMMIT_MARKER 0xA55AU
#define IMAGE_METADATA_INITIAL_SEQUENCE 1U
#define IMAGE_METADATA_MAX_BOOT_ATTEMPTS 3U
#define IMAGE_METADATA_FLAGS_ALLOWED_MASK 0x00U

typedef enum { IMAGE_SLOT_STATE_EMPTY=0, IMAGE_SLOT_STATE_UPDATE_IN_PROGRESS=1,
 IMAGE_SLOT_STATE_PENDING=2, IMAGE_SLOT_STATE_CONFIRMED=3,
 IMAGE_SLOT_STATE_VALID_ROLLBACK=4, IMAGE_SLOT_STATE_INVALID=5 } ImageMetadata_SlotStateType;
typedef enum { IMAGE_ROLLBACK_NONE=0, IMAGE_ROLLBACK_PENDING_VECTOR_INVALID=1,
 IMAGE_ROLLBACK_PENDING_DESCRIPTOR_INVALID=2, IMAGE_ROLLBACK_PENDING_CRC_INVALID=3,
 IMAGE_ROLLBACK_BOOT_ATTEMPTS_EXCEEDED=4, IMAGE_ROLLBACK_UPDATE_INTERRUPTED=5,
 IMAGE_ROLLBACK_CONFIRMED_IMAGE_INVALID=6 } ImageMetadata_RollbackReasonType;
typedef enum { IMAGE_METADATA_SCAN_V2_RECORD=0, IMAGE_METADATA_SCAN_V1_RECORD,
 IMAGE_METADATA_ABSENT_LEGACY, IMAGE_METADATA_CORRUPT } ImageMetadata_ScanStateType;
typedef enum { IMAGE_METADATA_OK=0, IMAGE_METADATA_INVALID_ARGUMENT=-1,
 IMAGE_METADATA_ERASE_ERROR=-2, IMAGE_METADATA_PROGRAM_ERROR=-3,
 IMAGE_METADATA_VERIFY_ERROR=-4 } ImageMetadata_StatusType;

typedef struct {
 uint32_t magic; uint16_t format_version; uint16_t record_size; uint32_t sequence;
 uint32_t slot_a_image_size; uint32_t slot_a_crc32; uint32_t slot_b_image_size; uint32_t slot_b_crc32;
 uint8_t slot_a_version[4]; uint8_t slot_b_version[4];
 uint8_t slot_a_state; uint8_t slot_b_state; uint8_t active_slot; uint8_t confirmed_slot;
 uint8_t pending_slot; uint8_t boot_attempt_count; uint8_t max_boot_attempts;
 uint8_t rollback_reason; uint8_t last_boot_slot; uint8_t flags; uint16_t reserved16;
 uint32_t reserved32[2]; uint32_t metadata_crc32; uint16_t commit_marker; uint16_t padding;
} ImageMetadata_RecordType;

typedef struct { uint32_t sequence, image_base, image_size, image_crc32;
 uint8_t software_version[4]; uint32_t update_state; } ImageMetadata_LegacyInfoType;
typedef struct { ImageMetadata_ScanStateType state; ImageMetadata_RecordType record;
 ImageMetadata_LegacyInfoType legacy; uint32_t active_page_address; } ImageMetadata_ScanResultType;

bool ImageMetadata_IsSequenceNewer(uint32_t candidate,uint32_t reference);
void ImageMetadata_Scan(ImageMetadata_ScanResultType *result);
ImageMetadata_StatusType ImageMetadata_CommitRecord(const ImageMetadata_RecordType *desired);
ImageMetadata_StatusType ImageMetadata_MigrateV1(const ImageMetadata_ScanResultType *scan);
ImageMetadata_StatusType ImageMetadata_MarkUpdateInProgress(uint8_t slot_id,uint32_t image_size);
ImageMetadata_StatusType ImageMetadata_MarkPending(uint8_t slot_id,uint32_t image_size,uint32_t image_crc32,const uint8_t software_version[4]);
ImageMetadata_StatusType ImageMetadata_MarkCandidateInvalid(uint8_t slot_id,ImageMetadata_RollbackReasonType reason);
ImageMetadata_StatusType ImageMetadata_IncrementPendingAttempt(uint8_t slot_id,ImageMetadata_RecordType *committed);
ImageMetadata_StatusType ImageMetadata_ConfirmPending(uint8_t slot_id,uint32_t expected_sequence);
ImageMetadata_StatusType ImageMetadata_PromoteRollback(uint8_t failed_slot,uint8_t rollback_slot);
uint8_t ImageMetadata_GetSlotState(const ImageMetadata_RecordType *record,uint8_t slot_id);
uint32_t ImageMetadata_GetSlotSize(const ImageMetadata_RecordType *record,uint8_t slot_id);
uint32_t ImageMetadata_GetSlotCrc(const ImageMetadata_RecordType *record,uint8_t slot_id);
const uint8_t *ImageMetadata_GetSlotVersion(const ImageMetadata_RecordType *record,uint8_t slot_id);

#endif
