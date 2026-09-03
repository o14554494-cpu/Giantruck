#include "main.h"
#include "brush_feedback.h"

static uint8_t adc_ready;
static uint32_t adc_poll_tick;

static uint8_t BrushEncoder_ReadAB(void)
{
  return (uint8_t)((HAL_GPIO_ReadPin(BRUSH_ENCODER_A_PORT, BRUSH_ENCODER_A_PIN) != GPIO_PIN_RESET ? 2U : 0U) |
                  (HAL_GPIO_ReadPin(BRUSH_ENCODER_B_PORT, BRUSH_ENCODER_B_PIN) != GPIO_PIN_RESET ? 1U : 0U));
}

/* Bounded, startup-only waits. ADC1 is exclusively owned by this module.
 * Direct CMSIS configuration avoids adding missing HAL ADC vendor files.
 * RM0008 ADC: PCLK2/6=12 MHz, single channel, 239.5-cycle sample, software start. */
static uint8_t BrushAdc_WaitClear(uint32_t mask)
{
  uint32_t start = HAL_GetTick();
  while ((ADC1->CR2 & mask) != 0U)
    if ((uint32_t)(HAL_GetTick() - start) >= 20U) return 0U;
  return 1U;
}

void BrushFeedback_HardwareInit(void)
{
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_AFIO_CLK_ENABLE();
  BRUSH_ENCODER_GPIO_CLOCK_ENABLE();
  HAL_NVIC_DisableIRQ(BRUSH_ENCODER_IRQn);
  gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
  gpio.Pull = BRUSH_ENCODER_PULL;
  gpio.Pin = BRUSH_ENCODER_A_PIN;
  HAL_GPIO_Init(BRUSH_ENCODER_A_PORT, &gpio);
  gpio.Pin = BRUSH_ENCODER_B_PIN;
  HAL_GPIO_Init(BRUSH_ENCODER_B_PORT, &gpio);
  BrushFeedback_Reset(HAL_GetTick(), BrushEncoder_ReadAB());
  __HAL_GPIO_EXTI_CLEAR_IT(BRUSH_ENCODER_A_PIN | BRUSH_ENCODER_B_PIN);
  HAL_NVIC_SetPriority(BRUSH_ENCODER_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(BRUSH_ENCODER_IRQn);

  BRUSH_CURRENT_GPIO_CLOCK_ENABLE();
  gpio.Pin = BRUSH_CURRENT_PIN;
  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BRUSH_CURRENT_PORT, &gpio);
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_ADC1_FORCE_RESET();
  __HAL_RCC_ADC1_RELEASE_RESET();
  MODIFY_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_DIV6);
  ADC1->CR1 = 0U;
  ADC1->CR2 = ADC_CR2_EXTSEL | ADC_CR2_CONT;
  ADC1->SQR1 = ADC1->SQR2 = 0U;
  ADC1->SQR3 = BRUSH_CURRENT_ADC_CHANNEL;
#if BRUSH_CURRENT_ADC_CHANNEL < 10
  ADC1->SMPR2 = 7U << (BRUSH_CURRENT_ADC_CHANNEL * 3U);
#else
  ADC1->SMPR1 = 7U << ((BRUSH_CURRENT_ADC_CHANNEL - 10U) * 3U);
#endif
  ADC1->CR2 |= ADC_CR2_ADON;
  HAL_Delay(1U); /* ADC power-up stabilization; never called while driving. */
  ADC1->CR2 |= ADC_CR2_RSTCAL;
  if (!BrushAdc_WaitClear(ADC_CR2_RSTCAL)) return;
  ADC1->CR2 |= ADC_CR2_CAL;
  if (!BrushAdc_WaitClear(ADC_CR2_CAL)) return;
  ADC1->CR2 |= ADC_CR2_EXTTRIG | ADC_CR2_SWSTART;
  adc_ready = 1U;
  adc_poll_tick = HAL_GetTick();
}

void BrushFeedback_HardwarePoll(void)
{
  uint32_t now = HAL_GetTick();
  if (adc_ready && (uint32_t)(now - adc_poll_tick) >= 10U)
  {
    adc_poll_tick = now;
    if ((ADC1->SR & ADC_SR_EOC) != 0U)
      BrushFeedback_RecordAdc((uint16_t)(ADC1->DR & 4095U), now);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == BRUSH_ENCODER_A_PIN || pin == BRUSH_ENCODER_B_PIN)
    BrushFeedback_EncoderEdge(BrushEncoder_ReadAB());
}
