#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define APP_MOTOR_COUNT 4U /* 四个轮毂电机共用同一套控制参数模型。 */

/*
 * 世界坐标约定：场地右上角为XY原点，+X向左，+Y向数字区；+Z向下。
 * 步进轴reverse=0表示沿对应坐标正方向运动。触底点是Z最大值，不是Z零点。
 * 2026-07-26实机标定。软行程生效前必须先通过光电门建立可信坐标。
 */
#define STEPPER_X_TRAVEL_PULSES          33924U
#define STEPPER_X_TRAVEL_MM_X10          16930U
#define STEPPER_X_SAFE_MARGIN_PULSES       500U
#define STEPPER_X_SAFE_TRAVEL_PULSES     33424U
#define STEPPER_Z_TRAVEL_PULSES           5784U
#define STEPPER_Z_TRAVEL_MM_X10           2810U
#define STEPPER_Z_SAFE_MARGIN_PULSES       200U
#define STEPPER_Z_SAFE_TRAVEL_PULSES      5584U

/* 识别阶段夹爪离地75mm即可随底盘移动，无需升到Z轴机械最高点。 */
#define RECOGNITION_CLEARANCE_MM_X10        750U
#define RECOGNITION_LIFT_PULSES \
  ((STEPPER_Z_TRAVEL_PULSES * RECOGNITION_CLEARANCE_MM_X10 + \
    STEPPER_Z_TRAVEL_MM_X10 / 2U) / STEPPER_Z_TRAVEL_MM_X10)
#define RECOGNITION_TRAVEL_Z_PULSES \
  (STEPPER_Z_TRAVEL_PULSES - RECOGNITION_LIFT_PULSES)

/*
 * 数字区五个物理箱位的X轴实机标定值（2026-07-27）。
 * 编号表示物理槽位而非箱体数字：1~3为底部左/中/右，4~5为侧面左/右。
 * 两个侧面箱位分别以对应端光电门刚好遮挡的位置为准。
 */
#define NUMBER_SLOT_BOTTOM_LEFT_X_PULSES   24400
#define NUMBER_SLOT_BOTTOM_CENTER_X_PULSES 16300
#define NUMBER_SLOT_BOTTOM_RIGHT_X_PULSES   8500
#define NUMBER_SLOT_OFFSET_LEFT_X_PULSES   ((int32_t)STEPPER_X_TRAVEL_PULSES)
#define NUMBER_SLOT_OFFSET_RIGHT_X_PULSES  0

/* 豆子区三个物理箱位：左、右为实测值，中间凸出箱与数字区中箱共线。 */
#define BEAN_SLOT_TOP_LEFT_X_PULSES        26500
#define BEAN_SLOT_OFFSET_X_PULSES          NUMBER_SLOT_BOTTOM_CENTER_X_PULSES
#define BEAN_SLOT_TOP_RIGHT_X_PULSES        6700

/*
 * Z轴抓放高度实机标定值（+Z向下）。
 * 豆子点位1=中间凸出箱，点位2=顶部右箱，点位3=顶部左箱。
 */
#define BEAN_PICKUP_Z_LEVEL_1_PULSES        2500
/* 3号遥控点位实机避险：较原3600上移约15.06mm（310脉冲）。 */
#define BEAN_PICKUP_Z_LEVEL_2_PULSES        3290
#define BEAN_PICKUP_Z_LEVEL_3_PULSES        4300
#define NUMBER_DROP_Z_PULSES                3300
/* 比最高抓取层再高约9.7mm，给横移和张爪留出余量。 */
#define BEAN_PICKUP_PREP_MARGIN_PULSES        200
#define BEAN_PICKUP_PREP_Z_PULSES \
  (BEAN_PICKUP_Z_LEVEL_1_PULSES - BEAN_PICKUP_PREP_MARGIN_PULSES)

/* 槽位动作使用规定逻辑角度，舵机驱动层会自动叠加6度安装偏移。 */
#define SLOT_YAW_INITIAL_DEGREES                0U
#define NUMBER_BOTTOM_ROW_YAW_DEGREES          90U

/* 运行期可替换的机器人参数快照。 */
typedef struct
{
  float speed_kp[APP_MOTOR_COUNT];
  float speed_ki[APP_MOTOR_COUNT];
  float speed_feedforward[APP_MOTOR_COUNT];
  /* 各轮闭环目标速度补偿；正反转时均增加对应速度绝对值。 */
  float motor_target_trim_rpm[APP_MOTOR_COUNT];
  float speed_sync_kp;
  float acceleration_rpm_s;
  float deceleration_rpm_s;
  float encoder_counts_per_output_rev;
  /* 驱动轮实测滚动周长，用于把编码器累计计数换算为底盘里程。 */
  float wheel_circumference_mm;
  float speed_filter_alpha;
  float maximum_rpm;
  uint16_t pwm_max;
  uint16_t pwm_deadband;
  uint16_t motor_control_period_ms;
  uint16_t speed_measurement_period_ms;
  uint16_t ui_refresh_period_ms;
  uint16_t key_debounce_ms;
  uint16_t servo_min_us[2];
  uint16_t servo_max_us[2];
  uint16_t servo_travel_degrees[2];
  uint16_t servo_angle_offset_degrees[2];
  uint16_t servo_command_min_degrees[2];
  uint16_t servo_command_max_degrees[2];
  uint16_t servo_initial_degrees[2];
  uint16_t gripper_closed_degrees;
  uint16_t gripper_open_degrees;
  uint16_t gripper_release_degrees;
  int8_t motor_polarity[APP_MOTOR_COUNT];
  int8_t encoder_polarity[APP_MOTOR_COUNT];
} AppConfig;

void AppConfig_Init(void);
/* 获取一致的参数副本，供控制任务和显示任务读取。 */
void AppConfig_GetSnapshot(AppConfig *snapshot);
/* 校验并原子替换当前参数；返回 1 表示成功。 */
uint8_t AppConfig_Replace(const AppConfig *candidate);
/* 恢复编译期默认参数。 */
void AppConfig_LoadDefaults(void);

#endif
