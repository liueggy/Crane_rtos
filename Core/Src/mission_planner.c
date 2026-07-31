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
static uint8_t g_inferred_bean_mask;
static uint8_t g_inferred_number_mask;
static uint32_t g_prng_state;

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
  g_inferred_bean_mask = 0U;
  g_inferred_number_mask = 0U;
}

static uint32_t NextRandom(void)
{
  g_prng_state = g_prng_state * 1664525UL + 1013904223UL;
  return g_prng_state;
}

static void Shuffle(uint8_t *values, uint8_t count)
{
  while (count > 1U)
  {
    uint8_t index = (uint8_t)(NextRandom() % count);
    uint8_t last = (uint8_t)(count - 1U);
    uint8_t value = values[index];
    values[index] = values[last];
    values[last] = value;
    count = last;
  }
}

static uint8_t Build(const VisionSurveyMap *survey, uint8_t completed_mask)
{
  static const uint8_t priority[MISSION_TRANSPORT_TASK_COUNT] = {0U, 2U, 1U};
  uint8_t beans[WORLD_BEAN_SLOT_COUNT] = {0U};
  uint8_t numbers[WORLD_NUMBER_SLOT_COUNT] = {0U};
  uint8_t bean_used = 0U;
  uint8_t number_used = 0U;
  uint8_t free_slots[WORLD_NUMBER_SLOT_COUNT];
  uint8_t missing[WORLD_NUMBER_SLOT_COUNT];
  uint8_t free_count = 0U;
  uint8_t missing_count = 0U;
  MissionPlanner_Init();
  if (survey == 0) return 0U;
  g_completed_bean_mask = (uint8_t)(completed_mask & 0x07U);
  g_prng_state = 0x6D2B79F5UL ^ survey->last_sequence ^
                 ((uint32_t)survey->number_valid_mask << 8U) ^
                 ((uint32_t)survey->bean_valid_mask << 16U);

  /* 先锁定互不冲突的可靠豆类，重复或缺失项留给后续唯一补全。 */
  for (uint8_t slot = 0U; slot < WORLD_BEAN_SLOT_COUNT; ++slot)
  {
    uint8_t semantic = survey->bean_at_slot[slot];
    uint8_t bit = (semantic == K230_SEMANTIC_BEAN_H) ? 0x01U :
                  (semantic == K230_SEMANTIC_BEAN_L) ? 0x02U :
                  (semantic == K230_SEMANTIC_BEAN_B) ? 0x04U : 0U;
    if ((survey->bean_valid_mask & (uint8_t)(1U << slot)) && bit &&
        ((bean_used & bit) == 0U))
    {
      beans[slot] = semantic;
      bean_used |= bit;
    }
    else g_inferred_bean_mask |= (uint8_t)(1U << slot);
  }
  for (uint8_t slot = 0U; slot < WORLD_BEAN_SLOT_COUNT; ++slot)
  {
    if (beans[slot] == 0U) free_slots[free_count++] = slot;
  }
  if ((bean_used & 0x01U) == 0U) missing[missing_count++] = K230_SEMANTIC_BEAN_H;
  if ((bean_used & 0x02U) == 0U) missing[missing_count++] = K230_SEMANTIC_BEAN_L;
  if ((bean_used & 0x04U) == 0U) missing[missing_count++] = K230_SEMANTIC_BEAN_B;
  Shuffle(missing, missing_count);
  Shuffle(free_slots, free_count);
  for (uint8_t i = 0U; i < free_count; ++i) beans[free_slots[i]] = missing[i];

  free_count = 0U;
  missing_count = 0U;
  /* 数字4/5的可靠箱位同样保留，缺少的1/2/3只分配给剩余物理箱。 */
  for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
  {
    uint8_t number = survey->number_at_slot[slot];
    uint8_t bit = ((number >= 1U) && (number <= 5U)) ?
                  (uint8_t)(1U << (number - 1U)) : 0U;
    if ((survey->number_valid_mask & (uint8_t)(1U << slot)) && bit &&
        ((number_used & bit) == 0U))
    {
      numbers[slot] = number;
      number_used |= bit;
    }
    else g_inferred_number_mask |= (uint8_t)(1U << slot);
  }
  for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
    if (numbers[slot] == 0U) free_slots[free_count++] = slot;
  for (uint8_t number = 1U; number <= 5U; ++number)
    if ((number_used & (uint8_t)(1U << (number - 1U))) == 0U)
      missing[missing_count++] = number;
  Shuffle(missing, missing_count);
  Shuffle(free_slots, free_count);
  for (uint8_t i = 0U; i < free_count; ++i) numbers[free_slots[i]] = missing[i];

  if (g_inferred_bean_mask != 0U)
  {
    g_issue = MISSION_PLAN_BEAN_MISSING;
    g_issue_mask |= g_inferred_bean_mask;
  }
  if (g_inferred_number_mask != 0U)
  {
    if (g_issue == MISSION_PLAN_OK) g_issue = MISSION_PLAN_NUMBER_MISSING;
    g_issue_mask |= (uint8_t)(g_inferred_number_mask << 3U);
  }

  /* 两侧箱优先：物理A、C先执行，最后才是凸出的B箱。 */
  for (uint8_t order = 0U; order < MISSION_TRANSPORT_TASK_COUNT; ++order)
  {
    uint8_t bean_slot = priority[order];
    WorldSlotId drop_slot = WORLD_SLOT_COUNT;
    uint8_t drop_physical_slot = WORLD_NUMBER_SLOT_COUNT;
    uint8_t bean = beans[bean_slot];
    uint8_t target_number = (bean == K230_SEMANTIC_BEAN_H) ? 1U :
                            (bean == K230_SEMANTIC_BEAN_L) ? 2U :
                            (bean == K230_SEMANTIC_BEAN_B) ? 3U : 0U;
    if (g_completed_bean_mask & (uint8_t)(1U << bean_slot)) continue;
    for (uint8_t slot = 0U; slot < WORLD_NUMBER_SLOT_COUNT; ++slot)
    {
      if (numbers[slot] == target_number)
      {
        drop_slot = NumberWorldSlot(slot);
        drop_physical_slot = slot;
        break;
      }
    }
    g_tasks[g_task_count].bean_semantic = bean;
    g_tasks[g_task_count].physical_bean_slot = bean_slot;
    g_tasks[g_task_count].pickup_slot = BeanWorldSlot(bean_slot);
    g_tasks[g_task_count].target_number = target_number;
    g_tasks[g_task_count].drop_slot = drop_slot;
    g_tasks[g_task_count].bean_inferred =
        (g_inferred_bean_mask & (uint8_t)(1U << bean_slot)) ? 1U : 0U;
    g_tasks[g_task_count].drop_inferred =
        (drop_physical_slot < WORLD_NUMBER_SLOT_COUNT) &&
        (g_inferred_number_mask & (uint8_t)(1U << drop_physical_slot)) ? 1U : 0U;
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
  return g_ready &&
         ((uint8_t)(g_task_count + CountBeanBits(g_completed_bean_mask)) ==
          MISSION_TRANSPORT_TASK_COUNT);
}
MissionPlanIssue MissionPlanner_GetIssue(void) { return g_issue; }
uint8_t MissionPlanner_GetIssueMask(void) { return g_issue_mask; }
uint8_t MissionPlanner_GetInferredBeanMask(void) { return g_inferred_bean_mask; }
uint8_t MissionPlanner_GetInferredNumberMask(void) { return g_inferred_number_mask; }

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
