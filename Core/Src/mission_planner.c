#include "mission_planner.h"

#include "k230_link.h"

#include <string.h>

static MissionTransportTask g_tasks[MISSION_TRANSPORT_TASK_COUNT];
static uint8_t g_ready;

static WorldSlotId BeanWorldSlot(uint8_t slot)
{
  static const WorldSlotId map[WORLD_BEAN_SLOT_COUNT] = {
    WORLD_SLOT_BEAN_TOP_LEFT,
    WORLD_SLOT_BEAN_OFFSET,
    WORLD_SLOT_BEAN_TOP_RIGHT,
  };
  return map[slot];
}

static WorldSlotId NumberWorldSlot(uint8_t slot)
{
  static const WorldSlotId map[WORLD_NUMBER_SLOT_COUNT] = {
    WORLD_SLOT_NUMBER_OFFSET_RIGHT,
    WORLD_SLOT_NUMBER_BOTTOM_RIGHT,
    WORLD_SLOT_NUMBER_BOTTOM_CENTER,
    WORLD_SLOT_NUMBER_BOTTOM_LEFT,
    WORLD_SLOT_NUMBER_OFFSET_LEFT,
  };
  return map[slot];
}

void MissionPlanner_Init(void)
{
  memset(g_tasks, 0, sizeof(g_tasks));
  g_ready = 0U;
}

uint8_t MissionPlanner_Build(const VisionSurveyMap *survey)
{
  uint8_t number_seen = 0U;
  uint8_t bean_seen = 0U;
  MissionPlanner_Init();
  if ((survey == 0) || (survey->number_valid_mask != 0x1FU) ||
      (survey->bean_valid_mask != 0x07U)) return 0U;
  for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
  {
    uint8_t number = survey->number_at_slot[slot];
    if ((number < 1U) || (number > 5U) ||
        (number_seen & (uint8_t)(1U << (number - 1U)))) return 0U;
    number_seen |= (uint8_t)(1U << (number - 1U));
  }
  for (uint8_t slot = 0U; slot < WORLD_BEAN_SLOT_COUNT; ++slot)
  {
    uint8_t bean = survey->bean_at_slot[slot];
    uint8_t bit = (bean == K230_SEMANTIC_BEAN_L) ? 0x01U :
                  (bean == K230_SEMANTIC_BEAN_H) ? 0x02U :
                  (bean == K230_SEMANTIC_BEAN_B) ? 0x04U : 0U;
    if ((bit == 0U) || (bean_seen & bit)) return 0U;
    bean_seen |= bit;
  }

  /* 任务固定按物理箱A/B/C，而不是按豆类语义排序。 */
  for (uint8_t task = 0U; task < MISSION_TRANSPORT_TASK_COUNT; ++task)
  {
    uint8_t drop_found = 0U;
    uint8_t bean = survey->bean_at_slot[task];
    uint8_t target_number = (bean == K230_SEMANTIC_BEAN_H) ? 1U :
                            (bean == K230_SEMANTIC_BEAN_L) ? 2U :
                            (bean == K230_SEMANTIC_BEAN_B) ? 3U : 0U;
    if (target_number == 0U) return 0U;
    g_tasks[task].bean_semantic = bean;
    g_tasks[task].pickup_slot = BeanWorldSlot(task);
    g_tasks[task].target_number = target_number;
    for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
    {
      if (survey->number_at_slot[slot] == target_number)
      {
        g_tasks[task].drop_slot = NumberWorldSlot(slot);
        drop_found = 1U;
      }
    }
    if (!drop_found) return 0U;
  }
  g_ready = 1U;
  return 1U;
}

uint8_t MissionPlanner_IsReady(void)
{
  return g_ready;
}

const MissionTransportTask *MissionPlanner_GetTask(uint8_t index)
{
  return (g_ready && (index < MISSION_TRANSPORT_TASK_COUNT)) ? &g_tasks[index] : 0;
}

