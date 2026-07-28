#include "box_calibration.h"

#include "app_config.h"
#include "stepper_axis.h"
#include "world_map.h"

static const uint32_t k_jog_steps[] = {10U, 50U, 100U, 500U, 1000U};
static const WorldSlotId k_number_slots[WORLD_NUMBER_SLOT_COUNT] = {
  WORLD_SLOT_NUMBER_BOTTOM_LEFT,
  WORLD_SLOT_NUMBER_BOTTOM_CENTER,
  WORLD_SLOT_NUMBER_BOTTOM_RIGHT,
  WORLD_SLOT_NUMBER_OFFSET_LEFT,
  WORLD_SLOT_NUMBER_OFFSET_RIGHT,
};

static uint8_t g_selected_slot;
static uint8_t g_step_index;
static uint8_t g_reference_valid;
static uint8_t g_saved_mask;

void BoxCalibration_Init(void)
{
  g_selected_slot = 0U;
  g_step_index = 2U;
  g_reference_valid = 0U;
  /* 五个数字箱X坐标已有编译期标定值，仍需回零后才可作为绝对位置使用。 */
  g_saved_mask = (uint8_t)((1U << WORLD_NUMBER_SLOT_COUNT) - 1U);
}

void BoxCalibration_SelectSlot(uint8_t slot)
{
  if ((slot < WORLD_NUMBER_SLOT_COUNT) &&
      !StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) g_selected_slot = slot;
}

void BoxCalibration_AdjustStep(int8_t direction)
{
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X)) return;
  if ((direction > 0) && (g_step_index + 1U <
      (uint8_t)(sizeof(k_jog_steps) / sizeof(k_jog_steps[0])))) ++g_step_index;
  else if ((direction < 0) && (g_step_index > 0U)) --g_step_index;
}

uint8_t BoxCalibration_Jog(uint8_t reverse)
{
  uint32_t pulses = k_jog_steps[g_step_index];
  int32_t current = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_X) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_Z)) return 0U;

  if (g_reference_valid)
  {
    if (reverse)
    {
      if (current <= 0) return 0U;
      if ((uint32_t)current < pulses) pulses = (uint32_t)current;
    }
    else
    {
      if (current >= (int32_t)STEPPER_X_SAFE_TRAVEL_PULSES) return 0U;
      if ((uint32_t)((int32_t)STEPPER_X_SAFE_TRAVEL_PULSES - current) < pulses)
        pulses = (uint32_t)((int32_t)STEPPER_X_SAFE_TRAVEL_PULSES - current);
    }
  }
  return (pulses != 0U) &&
         (StepperAxis_MovePulses(STEPPER_AXIS_X, pulses, reverse) == HAL_OK);
}

uint8_t BoxCalibration_SetXZero(void)
{
  if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_X) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_Z) ||
      !StepperAxis_ResetPositionPulses(STEPPER_AXIS_X)) return 0U;
  g_reference_valid = 1U;
  return 1U;
}

uint8_t BoxCalibration_SaveCurrent(void)
{
  int32_t x = StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
  if (!g_reference_valid || StepperAxis_IsEnabled(STEPPER_AXIS_X) ||
      (x < 0) || (x > (int32_t)STEPPER_X_SAFE_TRAVEL_PULSES) ||
      !WorldMap_SetSlotX(k_number_slots[g_selected_slot], x)) return 0U;
  g_saved_mask |= (uint8_t)(1U << g_selected_slot);
  return 1U;
}

uint8_t BoxCalibration_GetSelectedSlot(void)
{
  return g_selected_slot;
}

uint32_t BoxCalibration_GetJogStep(void)
{
  return k_jog_steps[g_step_index];
}

int32_t BoxCalibration_GetCurrentX(void)
{
  return StepperAxis_GetPositionPulses(STEPPER_AXIS_X);
}

uint8_t BoxCalibration_IsReferenceValid(void)
{
  return g_reference_valid;
}

uint8_t BoxCalibration_IsSelectedSaved(void)
{
  return (g_saved_mask & (uint8_t)(1U << g_selected_slot)) ? 1U : 0U;
}
