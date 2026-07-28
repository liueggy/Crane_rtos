#ifndef VISION_SURVEY_H
#define VISION_SURVEY_H

#include "k230_link.h"
#include "world_map.h"

#include <stdint.h>

typedef enum
{
  VISION_SURVEY_IDLE = 0,
  VISION_SURVEY_NUMBER_A,
  VISION_SURVEY_NUMBER_B,
  VISION_SURVEY_BEAN_C,
  VISION_SURVEY_COMPLETE,
  VISION_SURVEY_NEEDS_CALIBRATION,
  VISION_SURVEY_INVALID_RESULT
} VisionSurveyState;

typedef struct
{
  uint8_t number_at_slot[WORLD_NUMBER_SLOT_COUNT];
  uint8_t bean_at_slot[WORLD_BEAN_SLOT_COUNT];
  uint8_t number_valid_mask;
  uint8_t bean_valid_mask;
  uint16_t last_sequence;
} VisionSurveyMap;

void VisionSurvey_Init(void);
uint8_t VisionSurvey_Begin(void);
void VisionSurvey_Reset(void);
VisionSurveyState VisionSurvey_GetState(void);
WorldScanPoseId VisionSurvey_GetRequiredPose(void);
const VisionSurveyMap *VisionSurvey_GetMap(void);

/*
 * 将一次稳定扫描结果写入全局槽位。slot_indices由扫描姿态标定层提供，
 * 每个目标对应0..4数字槽或0..2豆子槽；后续可替换为多帧投票器。
 */
uint8_t VisionSurvey_AcceptResult(const K230VisionResult *result,
                                  const uint8_t *slot_indices,
                                  uint8_t slot_count);
uint8_t VisionSurvey_IsComplete(void);

#endif
