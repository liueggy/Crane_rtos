#include "vision_survey.h"

#include <string.h>

#define NUMBER_COMPLETE_MASK 0x1FU
#define BEAN_COMPLETE_MASK   0x07U

static VisionSurveyState g_state;
static VisionSurveyMap g_map;

static uint8_t IsNumberSemantic(uint8_t semantic)
{
  return (semantic >= K230_SEMANTIC_NUMBER_1) &&
         (semantic <= K230_SEMANTIC_NUMBER_5);
}

static uint8_t IsBeanSemantic(uint8_t semantic)
{
  return (semantic == K230_SEMANTIC_BEAN_L) ||
         (semantic == K230_SEMANTIC_BEAN_H) ||
         (semantic == K230_SEMANTIC_BEAN_B);
}

void VisionSurvey_Init(void)
{
  VisionSurvey_Reset();
}

void VisionSurvey_Reset(void)
{
  memset(&g_map, 0, sizeof(g_map));
  g_state = VISION_SURVEY_IDLE;
}

uint8_t VisionSurvey_Begin(void)
{
  VisionSurvey_Reset();
  if (!WorldMap_IsSurveyCalibrated())
  {
    g_state = VISION_SURVEY_NEEDS_CALIBRATION;
    return 0U;
  }
  g_state = VISION_SURVEY_NUMBER_A;
  return 1U;
}

VisionSurveyState VisionSurvey_GetState(void)
{
  return g_state;
}

WorldScanPoseId VisionSurvey_GetRequiredPose(void)
{
  if (g_state == VISION_SURVEY_NUMBER_B) return WORLD_SCAN_NUMBER_B;
  if (g_state == VISION_SURVEY_BEAN_C) return WORLD_SCAN_BEAN_C;
  return WORLD_SCAN_NUMBER_A;
}

const VisionSurveyMap *VisionSurvey_GetMap(void)
{
  return &g_map;
}

uint8_t VisionSurvey_AcceptResult(const K230VisionResult *result,
                                  const uint8_t *slot_indices,
                                  uint8_t slot_count)
{
  if ((result == 0) || (slot_indices == 0) || !result->valid ||
      (result->count == 0U) ||
      (result->sequence == g_map.last_sequence) ||
      (slot_count < result->count)) return 0U;

  if ((g_state == VISION_SURVEY_NUMBER_A) ||
      (g_state == VISION_SURVEY_NUMBER_B))
  {
    if (result->task != K230_TASK_NUMBER) return 0U;
    for (uint8_t i = 0U; i < result->count; ++i)
    {
      uint8_t slot = slot_indices[i];
      if ((slot >= WORLD_NUMBER_SLOT_COUNT) ||
          !IsNumberSemantic(result->targets[i].semantic)) continue;
      g_map.number_at_slot[slot] = result->targets[i].semantic;
      g_map.number_valid_mask |= (uint8_t)(1U << slot);
    }
    g_state = (g_state == VISION_SURVEY_NUMBER_A) ?
              VISION_SURVEY_NUMBER_B : VISION_SURVEY_BEAN_C;
  }
  else if (g_state == VISION_SURVEY_BEAN_C)
  {
    if (result->task != K230_TASK_BEAN) return 0U;
    for (uint8_t i = 0U; i < result->count; ++i)
    {
      uint8_t slot = slot_indices[i];
      if ((slot >= WORLD_BEAN_SLOT_COUNT) ||
          !IsBeanSemantic(result->targets[i].semantic)) continue;
      g_map.bean_at_slot[slot] = result->targets[i].semantic;
      g_map.bean_valid_mask |= (uint8_t)(1U << slot);
    }

    /* 识别到两个不同豆类时，用固定三类集合补齐剩余槽位。 */
    if ((g_map.bean_valid_mask == 0x03U) ||
        (g_map.bean_valid_mask == 0x05U) ||
        (g_map.bean_valid_mask == 0x06U))
    {
      uint8_t seen = 0U;
      uint8_t missing_slot = 0U;
      for (uint8_t slot = 0U; slot < WORLD_BEAN_SLOT_COUNT; ++slot)
      {
        if (g_map.bean_valid_mask & (uint8_t)(1U << slot))
        {
          uint8_t code = g_map.bean_at_slot[slot];
          if (code == K230_SEMANTIC_BEAN_L) seen |= 0x01U;
          else if (code == K230_SEMANTIC_BEAN_H) seen |= 0x02U;
          else if (code == K230_SEMANTIC_BEAN_B) seen |= 0x04U;
        }
        else missing_slot = slot;
      }
      if ((seen == 0x03U) || (seen == 0x05U) || (seen == 0x06U))
      {
        g_map.bean_at_slot[missing_slot] =
            (seen == 0x03U) ? K230_SEMANTIC_BEAN_B :
            (seen == 0x05U) ? K230_SEMANTIC_BEAN_H : K230_SEMANTIC_BEAN_L;
        g_map.bean_valid_mask |= (uint8_t)(1U << missing_slot);
      }
    }
  }

  g_map.last_sequence = result->sequence;
  if (VisionSurvey_IsComplete()) g_state = VISION_SURVEY_COMPLETE;
  return 1U;
}

uint8_t VisionSurvey_IsComplete(void)
{
  if ((g_map.number_valid_mask != NUMBER_COMPLETE_MASK) ||
      (g_map.bean_valid_mask != BEAN_COMPLETE_MASK)) return 0U;

  uint8_t number_seen = 0U;
  uint8_t bean_seen = 0U;
  for (uint8_t i = 0U; i < WORLD_NUMBER_SLOT_COUNT; ++i)
  {
    uint8_t number = g_map.number_at_slot[i];
    if ((number < 1U) || (number > 5U) ||
        (number_seen & (uint8_t)(1U << (number - 1U)))) return 0U;
    number_seen |= (uint8_t)(1U << (number - 1U));
  }
  for (uint8_t i = 0U; i < WORLD_BEAN_SLOT_COUNT; ++i)
  {
    uint8_t bean = g_map.bean_at_slot[i];
    uint8_t bit = (bean == K230_SEMANTIC_BEAN_L) ? 0x01U :
                  (bean == K230_SEMANTIC_BEAN_H) ? 0x02U :
                  (bean == K230_SEMANTIC_BEAN_B) ? 0x04U : 0U;
    if ((bit == 0U) || (bean_seen & bit)) return 0U;
    bean_seen |= bit;
  }
  return (number_seen == NUMBER_COMPLETE_MASK) && (bean_seen == BEAN_COMPLETE_MASK);
}
