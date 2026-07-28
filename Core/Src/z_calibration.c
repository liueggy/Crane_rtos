#include "z_calibration.h"

#include "app_config.h"
#include "stepper_axis.h"
#include "world_map.h"

static const uint32_t k_jog_steps[] = {10U, 50U, 100U, 500U, 1000U};

static uint8_t g_selected_target;
static uint8_t g_step_index;
static uint8_t g_reference_valid;
static uint8_t g_saved_mask;
static int32_t g_saved_z[Z_CALIBRATION_TARGET_COUNT];

void ZCalibration_Init(void)
{
  static const int32_t calibrated_z[Z_CALIBRATION_TARGET_COUNT] = {
    BEAN_PICKUP_Z_LEVEL_1_PULSES,
    BEAN_PICKUP_Z_LEVEL_2_PULSES,
    BEAN_PICKUP_Z_LEVEL_3_PULSES,
    NUMBER_DROP_Z_PULSES,
  };
  g_selected_target = Z_CALIBRATION_PICKUP_1;
  g_step_index = 2U;
  g_reference_valid = 0U;
  g_saved_mask = (uint8_t)((1U << Z_CALIBRATION_TARGET_COUNT) - 1U);
  for (uint8_t i = 0U; i < Z_CALIBRATION_TARGET_COUNT; ++i)
    g_saved_z[i] = calibrated_z[i];
}

void ZCalibration_SelectTarget(uint8_t target)
{
  if ((target < Z_CALIBRATION_TARGET_COUNT) &&
      !StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) g_selected_target = target;
}

void ZCalibration_AdjustStep(int8_t direction)
{
  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z)) return;
  if ((direction > 0) &&
      (g_step_index + 1U < (uint8_t)(sizeof(k_jog_steps) / sizeof(k_jog_steps[0]))))
    ++g_step_index;
  else if ((direction < 0) && (g_step_index > 0U))
    --g_step_index;
}

uint8_t ZCalibration_Jog(uint8_t reverse)
{
  uint32_t pulses = k_jog_steps[g_step_index];
  int32_t current = StepperAxis_GetPositionPulses(STEPPER_AXIS_Z);

  if (StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z) ||
      StepperAxis_IsEnabled(STEPPER_AXIS_X)) return 0U;

  if (g_reference_valid)
  {
    if (reverse)
    {
      if (current <= 0) return 0U;
      if ((uint32_t)current < pulses) pulses = (uint32_t)current;
    }
    else
    {
      if (current >= (int32_t)STEPPER_Z_TRAVEL_PULSES) return 0U;
      if ((uint32_t)((int32_t)STEPPER_Z_TRAVEL_PULSES - current) < pulses)
        pulses = (uint32_t)((int32_t)STEPPER_Z_TRAVEL_PULSES - current);
    }
  }

  return (pulses != 0U) &&
         (StepperAxis_MovePulses(STEPPER_AXIS_Z, pulses, reverse) == HAL_OK);
}

uint8_t ZCalibration_SetBottomReference(void)
{
  if (StepperAxis_IsEnabled(STEPPER_AXIS_X)) return 0U;
  /* 开机即触底时没有运动轴可自动归属，标定页明确将遮挡认作Z下限。 */
  if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_Z) &&
      !StepperAxis_ArmPhotoLimitEscape(STEPPER_AXIS_Z, 0U)) return 0U;
  if (!StepperAxis_IsPhotoLimitOwnedBy(STEPPER_AXIS_Z) ||
      !StepperAxis_SetPositionPulses(STEPPER_AXIS_Z,
                                    (int32_t)STEPPER_Z_TRAVEL_PULSES)) return 0U;
  g_reference_valid = 1U;
  /* 从已确认的触底点输出完整反向行程，到达Z=0后再向下寻找工作点。 */
  return (StepperAxis_MovePulses(STEPPER_AXIS_Z,
                                 STEPPER_Z_TRAVEL_PULSES, 1U) == HAL_OK) ? 1U : 0U;
}

uint8_t ZCalibration_SaveCurrent(void)
{
  int32_t z = StepperAxis_GetPositionPulses(STEPPER_AXIS_Z);
  if (!g_reference_valid || StepperAxis_IsPulseMoveActive(STEPPER_AXIS_Z) ||
      (z < 0) || (z > (int32_t)STEPPER_Z_TRAVEL_PULSES)) return 0U;
  g_saved_z[g_selected_target] = z;
  g_saved_mask |= (uint8_t)(1U << g_selected_target);
  return 1U;
}

uint8_t ZCalibration_GetSelectedTarget(void)
{
  return g_selected_target;
}

uint32_t ZCalibration_GetJogStep(void)
{
  return k_jog_steps[g_step_index];
}

int32_t ZCalibration_GetCurrentZ(void)
{
  return StepperAxis_GetPositionPulses(STEPPER_AXIS_Z);
}

int32_t ZCalibration_GetSavedZ(uint8_t target)
{
  return (target < Z_CALIBRATION_TARGET_COUNT) ?
         g_saved_z[target] : WORLD_MAP_UNCALIBRATED;
}

uint8_t ZCalibration_IsReferenceValid(void)
{
  return g_reference_valid;
}

uint8_t ZCalibration_IsSelectedSaved(void)
{
  return (g_saved_mask & (uint8_t)(1U << g_selected_target)) ? 1U : 0U;
}
