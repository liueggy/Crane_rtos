#include "mission_planner.h"

#include "k230_link.h"

#include <string.h>

static MissionTransportTask g_tasks[MISSION_TRANSPORT_TASK_COUNT];
static uint8_t g_ready;
static uint8_t g_task_count;
static uint8_t g_skipped_bean_mask;
static uint8_t g_completed_bean_mask;
static MissionPlanIssue g_issue;
static uint8_t g_issue_mask;
static MissionTaskSkipReason g_skip_reasons[WORLD_BEAN_SLOT_COUNT];

static uint8_t CountBeanBits(uint8_t mask)
{
  uint8_t count = 0U;
  mask &= 0x07U;
  while (mask != 0U)
  {
    count = (uint8_t)(count + (mask & 0x01U));
    mask >>= 1U;
  }
  return count;
}

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
  g_task_count = 0U;
  g_skipped_bean_mask = 0U;
  g_completed_bean_mask = 0U;
  g_issue = MISSION_PLAN_OK;
  g_issue_mask = 0U;
  memset(g_skip_reasons, 0, sizeof(g_skip_reasons));
}

static uint8_t Build(const VisionSurveyMap *survey, uint8_t completed_mask)
{
  static const uint8_t priority[MISSION_TRANSPORT_TASK_COUNT] = {0U, 2U, 1U};
  uint8_t bean_counts[3] = {0U, 0U, 0U};
  uint8_t number_counts[3] = {0U, 0U, 0U};
  MissionPlanner_Init();
  if (survey == 0) return 0U;
  g_completed_bean_mask = (uint8_t)(completed_mask & 0x07U);

  /* 正式比赛必须先建立三种豆类和数字1/2/3的唯一映射，禁止部分任务先抓。 */
  for (uint8_t slot = 0U; slot < WORLD_BEAN_SLOT_COUNT; ++slot)
  {
    uint8_t semantic = survey->bean_at_slot[slot];
    if ((survey->bean_valid_mask & (uint8_t)(1U << slot)) == 0U)
    {
      g_issue = MISSION_PLAN_BEAN_MISSING;
      g_issue_mask |= (uint8_t)(1U << slot);
      continue;
    }
    if (semantic == K230_SEMANTIC_BEAN_H) ++bean_counts[0];
    else if (semantic == K230_SEMANTIC_BEAN_L) ++bean_counts[1];
    else if (semantic == K230_SEMANTIC_BEAN_B) ++bean_counts[2];
    else
    {
      g_issue = MISSION_PLAN_BEAN_MISSING;
      g_issue_mask |= (uint8_t)(1U << slot);
    }
  }
  for (uint8_t semantic = 0U; semantic < 3U; ++semantic)
  {
    if (bean_counts[semantic] == 0U)
    {
      if (g_issue == MISSION_PLAN_OK) g_issue = MISSION_PLAN_BEAN_MISSING;
      g_issue_mask |= (uint8_t)(1U << semantic);
    }
    else if (bean_counts[semantic] != 1U)
    {
      g_issue = MISSION_PLAN_BEAN_CONFLICT;
      g_issue_mask |= (uint8_t)(1U << semantic);
    }
  }
  for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
  {
    uint8_t number = survey->number_at_slot[slot];
    if (((survey->number_valid_mask & (uint8_t)(1U << slot)) != 0U) &&
        (number >= 1U) && (number <= 3U)) ++number_counts[number - 1U];
  }
  for (uint8_t number = 0U; number < 3U; ++number)
  {
    if (number_counts[number] == 0U)
    {
      if ((g_issue == MISSION_PLAN_OK) ||
          (g_issue == MISSION_PLAN_NUMBER_MISSING))
        g_issue = MISSION_PLAN_NUMBER_MISSING;
      g_issue_mask |= (uint8_t)(1U << number);
    }
    else if (number_counts[number] != 1U)
    {
      if ((g_issue == MISSION_PLAN_OK) ||
          (g_issue == MISSION_PLAN_NUMBER_MISSING) ||
          (g_issue == MISSION_PLAN_NUMBER_CONFLICT))
        g_issue = MISSION_PLAN_NUMBER_CONFLICT;
      g_issue_mask |= (uint8_t)(1U << number);
    }
  }
  if (g_issue != MISSION_PLAN_OK)
  {
    g_ready = 1U;
    return 1U;
  }

  /* 两侧箱优先：物理A、C可信任务先执行，最后才是凸出的B箱。 */
  for (uint8_t order = 0U; order < MISSION_TRANSPORT_TASK_COUNT; ++order)
  {
    uint8_t bean_slot = priority[order];
    uint8_t drop_matches = 0U;
    WorldSlotId drop_slot = WORLD_SLOT_COUNT;
    uint8_t bean = survey->bean_at_slot[bean_slot];
    uint8_t bean_matches = 0U;
    uint8_t target_number = (bean == K230_SEMANTIC_BEAN_H) ? 1U :
                            (bean == K230_SEMANTIC_BEAN_L) ? 2U :
                            (bean == K230_SEMANTIC_BEAN_B) ? 3U : 0U;
    if (g_completed_bean_mask & (uint8_t)(1U << bean_slot)) continue;
    if (((survey->bean_valid_mask & (uint8_t)(1U << bean_slot)) == 0U) ||
        (target_number == 0U))
    {
      g_skipped_bean_mask |= (uint8_t)(1U << bean_slot);
      g_skip_reasons[bean_slot] = MISSION_SKIP_BEAN_UNKNOWN;
      continue;
    }
    for (uint8_t other = 0U; other < WORLD_BEAN_SLOT_COUNT; ++other)
      if ((survey->bean_valid_mask & (uint8_t)(1U << other)) &&
          (survey->bean_at_slot[other] == bean)) ++bean_matches;
    if (bean_matches != 1U)
    {
      g_skipped_bean_mask |= (uint8_t)(1U << bean_slot);
      g_skip_reasons[bean_slot] = MISSION_SKIP_BEAN_CONFLICT;
      continue;
    }
    for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
    {
      if ((survey->number_valid_mask & (uint8_t)(1U << slot)) &&
          (survey->number_at_slot[slot] == target_number))
      {
        drop_slot = NumberWorldSlot(slot);
        ++drop_matches;
      }
    }
    if (drop_matches != 1U)
    {
      g_skipped_bean_mask |= (uint8_t)(1U << bean_slot);
      g_skip_reasons[bean_slot] = (drop_matches == 0U) ?
          MISSION_SKIP_NUMBER_UNKNOWN : MISSION_SKIP_NUMBER_CONFLICT;
      continue;
    }
    g_tasks[g_task_count].bean_semantic = bean;
    g_tasks[g_task_count].physical_bean_slot = bean_slot;
    g_tasks[g_task_count].pickup_slot = BeanWorldSlot(bean_slot);
    g_tasks[g_task_count].target_number = target_number;
    g_tasks[g_task_count].drop_slot = drop_slot;
    ++g_task_count;
  }
  if ((uint8_t)(g_task_count + CountBeanBits(g_completed_bean_mask)) !=
      MISSION_TRANSPORT_TASK_COUNT)
  {
    g_issue = MISSION_PLAN_TASK_COUNT_INVALID;
    g_issue_mask = (uint8_t)(0x07U & (uint8_t)~g_completed_bean_mask);
    g_task_count = 0U;
  }
  g_ready = 1U;
  return 1U;
}

uint8_t MissionPlanner_Build(const VisionSurveyMap *survey)
{
  return Build(survey, 0U);
}

uint8_t MissionPlanner_RebuildRemaining(const VisionSurveyMap *survey,
                                        uint8_t completed_bean_mask)
{
  return Build(survey, completed_bean_mask);
}

uint8_t MissionPlanner_IsReady(void)
{
  return g_ready;
}

const MissionTransportTask *MissionPlanner_GetTask(uint8_t index)
{
  return (g_ready && (index < g_task_count)) ? &g_tasks[index] : 0;
}

uint8_t MissionPlanner_GetTaskCount(void) { return g_task_count; }
uint8_t MissionPlanner_GetSkippedBeanMask(void) { return g_skipped_bean_mask; }
uint8_t MissionPlanner_GetCompletedBeanMask(void) { return g_completed_bean_mask; }
uint8_t MissionPlanner_HasCompleteThreeTasks(void)
{
  return g_ready && (g_issue == MISSION_PLAN_OK) &&
         ((uint8_t)(g_task_count + CountBeanBits(g_completed_bean_mask)) ==
          MISSION_TRANSPORT_TASK_COUNT);
}
MissionPlanIssue MissionPlanner_GetIssue(void) { return g_issue; }
uint8_t MissionPlanner_GetIssueMask(void) { return g_issue_mask; }

MissionTaskSkipReason MissionPlanner_GetSkipReason(uint8_t physical_bean_slot)
{
  return (physical_bean_slot < WORLD_BEAN_SLOT_COUNT) ?
         g_skip_reasons[physical_bean_slot] : MISSION_SKIP_BEAN_UNKNOWN;
}

void MissionPlanner_MarkTaskCompleted(uint8_t index)
{
  if (index < g_task_count)
    g_completed_bean_mask |= (uint8_t)(1U << g_tasks[index].physical_bean_slot);
}
