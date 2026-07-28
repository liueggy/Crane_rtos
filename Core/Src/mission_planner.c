#include "mission_planner.h"

#include "k230_link.h"

#include <string.h>

static MissionTransportTask g_tasks[MISSION_TRANSPORT_TASK_COUNT];
static uint8_t g_ready;

static WorldSlotId BeanWorldSlot(uint8_t slot)
{
  static const WorldSlotId map[WORLD_BEAN_SLOT_COUNT] = {
    WORLD_SLOT_BEAN_TOP_LEFT,
    WORLD_SLOT_BEAN_TOP_RIGHT,
    WORLD_SLOT_BEAN_OFFSET,
  };
  return map[slot];
}

static WorldSlotId NumberWorldSlot(uint8_t slot)
{
  static const WorldSlotId map[WORLD_NUMBER_SLOT_COUNT] = {
    WORLD_SLOT_NUMBER_BOTTOM_LEFT,
    WORLD_SLOT_NUMBER_BOTTOM_CENTER,
    WORLD_SLOT_NUMBER_BOTTOM_RIGHT,
    WORLD_SLOT_NUMBER_OFFSET_LEFT,
    WORLD_SLOT_NUMBER_OFFSET_RIGHT,
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
  static const uint8_t beans[MISSION_TRANSPORT_TASK_COUNT] = {
    K230_SEMANTIC_BEAN_H, /* 黄豆 -> 1 */
    K230_SEMANTIC_BEAN_L, /* 绿豆 -> 2 */
    K230_SEMANTIC_BEAN_B, /* 白芸豆 -> 3 */
  };
  MissionPlanner_Init();
  if ((survey == 0) || !VisionSurvey_IsComplete()) return 0U;

  for (uint8_t task = 0U; task < MISSION_TRANSPORT_TASK_COUNT; ++task)
  {
    uint8_t pickup_found = 0U;
    uint8_t drop_found = 0U;
    g_tasks[task].bean_semantic = beans[task];
    g_tasks[task].target_number = (uint8_t)(task + 1U);

    for (uint8_t slot = 0U; slot < WORLD_BEAN_SLOT_COUNT; ++slot)
    {
      if (survey->bean_at_slot[slot] == beans[task])
      {
        g_tasks[task].pickup_slot = BeanWorldSlot(slot);
        pickup_found = 1U;
      }
    }
    for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
    {
      if (survey->number_at_slot[slot] == (uint8_t)(task + 1U))
      {
        g_tasks[task].drop_slot = NumberWorldSlot(slot);
        drop_found = 1U;
      }
    }
    if (!pickup_found || !drop_found) return 0U;
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

