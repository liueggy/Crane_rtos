#ifndef BEAN_SEQUENCE_DEMO_H
#define BEAN_SEQUENCE_DEMO_H

#include <stdint.h>

typedef enum
{
  BEAN_SEQUENCE_DEMO_IDLE = 0,
  BEAN_SEQUENCE_DEMO_REFERENCE,
  BEAN_SEQUENCE_DEMO_MOVE_TO_B,
  BEAN_SEQUENCE_DEMO_PICK_B,
  BEAN_SEQUENCE_DEMO_MOVE_TO_AC,
  BEAN_SEQUENCE_DEMO_ALIGN_AC,
  BEAN_SEQUENCE_DEMO_PICK_C,
  BEAN_SEQUENCE_DEMO_PICK_A,
  BEAN_SEQUENCE_DEMO_HOLD,
  BEAN_SEQUENCE_DEMO_RETURN_DESCEND,
  BEAN_SEQUENCE_DEMO_RELEASE,
  BEAN_SEQUENCE_DEMO_RETURN_RAISE,
  BEAN_SEQUENCE_DEMO_COMPLETE,
  BEAN_SEQUENCE_DEMO_FAULT,
} BeanSequenceDemoState;

void BeanSequenceDemo_Init(void);
void BeanSequenceDemo_Process(void);
void BeanSequenceDemo_Start(void);
void BeanSequenceDemo_Abort(void);

BeanSequenceDemoState BeanSequenceDemo_GetState(void);
uint8_t BeanSequenceDemo_IsRunning(void);
/* 0=A、1=B、2=C，用于OLED显示当前稳定性测试箱位。 */
uint8_t BeanSequenceDemo_GetCurrentPosition(void);

#endif
