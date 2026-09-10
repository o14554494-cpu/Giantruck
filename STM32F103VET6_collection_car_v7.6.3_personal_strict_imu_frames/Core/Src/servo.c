/**
  ******************************************************************************
  * @file    servo.c
  * @brief   MG995 后舱门舵机驱动
  *
  * 使用 TIM1_CH1(PA8) 输出 50Hz(20ms 周期) 舵机 PWM。
  * 定时器分频 72MHz/72=1MHz，一个计数=1us；周期 19999 → 20ms。
  * 占空比比较值即脉冲宽度微秒数(500~2500)。
  *
  * 注意：
  *   MG995 工作电流较大，必须用外部 5~6V 电源单独给舵机供电，
  *   电源负极与 STM32 GND 共地；信号线接 PA8。
  ******************************************************************************
  */
#include "main.h"
#include "servo.h"

static TIM_HandleTypeDef hservo_tim;
static uint8_t servo_running;
static uint32_t servo_start_tick;

typedef struct
{
  uint16_t angle_deg;
  uint16_t duration_ms;
} ServoHatchPoint_t;

/* One-shot unload path: 90° -> 0° (inward 90°) -> 180° (outward 180°), hold. */
static const ServoHatchPoint_t hatch_path[] =
{
  { SERVO_HATCH_INITIAL_DEG, 0U },
  { SERVO_HATCH_CCW_DEG,     SERVO_HATCH_MOVE90_MS },
  { SERVO_HATCH_CW_DEG,      SERVO_HATCH_SWEEP180_MS }
};
#define HATCH_PATH_POINTS (sizeof(hatch_path) / sizeof(hatch_path[0]))

static void Servo_SetPulseUs(uint16_t pulse_us)
{
  if (pulse_us < SERVO_MIN_PULSE_US)
  {
    pulse_us = SERVO_MIN_PULSE_US;
  }
  else if (pulse_us > SERVO_MAX_PULSE_US)
  {
    pulse_us = SERVO_MAX_PULSE_US;
  }
  __HAL_TIM_SET_COMPARE(&hservo_tim, TIM_CHANNEL_1, (uint32_t)pulse_us);
}

static uint16_t Servo_AngleToPulseUs(uint16_t angle_deg)
{
  uint32_t pulse_us;

  if (angle_deg > 180U)
  {
    angle_deg = 180U;
  }
  /* 0°=500us, 180°=2500us, 线性映射 */
  pulse_us = SERVO_MIN_PULSE_US +
             ((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) *
              angle_deg) / 180U;
  return (uint16_t)pulse_us;
}

static void Servo_UpdateHatch(uint32_t now_ms)
{
  const uint32_t phase = now_ms - servo_start_tick;
  uint32_t segment_start = 0U;
  uint32_t index;

  for (index = 1U; index < HATCH_PATH_POINTS; index++)
  {
    const uint32_t segment_end =
        segment_start + hatch_path[index].duration_ms;
    if (phase <= segment_end)
    {
      const uint16_t from = hatch_path[index - 1U].angle_deg;
      const uint16_t to = hatch_path[index].angle_deg;
      const uint32_t duration = hatch_path[index].duration_ms;
      const int32_t delta = (int32_t)to - (int32_t)from;
      const int32_t local = (int32_t)(phase - segment_start);
      uint16_t angle;

      if (duration == 0U)
      {
        angle = to;
      }
      else
      {
        angle = (uint16_t)((int32_t)from +
                           (delta * local) / (int32_t)duration);
      }
      Servo_SetPulseUs(Servo_AngleToPulseUs(angle));
      return;
    }
    segment_start = segment_end;
  }

  /* Sequence completed: keep the hatch at the outward 180-degree endpoint
   * while the chassis shakes. Servo_Stop() closes it after unloading. */
  Servo_SetPulseUs(Servo_AngleToPulseUs(SERVO_HATCH_CW_DEG));
}

void Servo_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef oc = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA8 -> TIM1_CH1 */
  gpio.Pin = GPIO_PIN_8;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  hservo_tim.Instance = TIM1;
  hservo_tim.Init.Prescaler = 71;              /* 72MHz/72 = 1MHz -> 1us */
  hservo_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
  hservo_tim.Init.Period = 19999;              /* 20ms -> 50Hz */
  hservo_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  hservo_tim.Init.RepetitionCounter = 0;
  hservo_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&hservo_tim) != HAL_OK)
  {
    Error_Handler();
  }

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = SERVO_NEUTRAL_PULSE_US;           /* 启动即回到中位 */
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&hservo_tim, &oc, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&hservo_tim, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  servo_running = 0U;
  servo_start_tick = 0U;
}

void Servo_StartEject(void)
{
  if (servo_running != 0U)
  {
    return; /* 已运行时不重置相位，避免扫摆跳动 */
  }
  servo_running = 1U;
  servo_start_tick = HAL_GetTick();

#if (SERVO_MODE == SERVO_MODE_CONTINUOUS)
  Servo_SetPulseUs(SERVO_CR_SPEED_PULSE_US);
#else
  Servo_SetPulseUs(Servo_AngleToPulseUs(SERVO_HATCH_INITIAL_DEG));
#endif
}

void Servo_Stop(void)
{
  servo_running = 0U;
  Servo_SetPulseUs(SERVO_NEUTRAL_PULSE_US);
}

void Servo_Update(uint32_t now_ms)
{
  if (servo_running == 0U)
  {
    return;
  }
#if (SERVO_MODE == SERVO_MODE_SWEEP)
  Servo_UpdateHatch(now_ms);
#else
  (void)now_ms;
#endif
}

uint8_t Servo_IsRunning(void)
{
  return servo_running;
}
