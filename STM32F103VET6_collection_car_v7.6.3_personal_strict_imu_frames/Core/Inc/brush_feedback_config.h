#ifndef BRUSH_FEEDBACK_CONFIG_H
#define BRUSH_FEEDBACK_CONFIG_H

/* Hardware wiring. Keep .ioc GPIO/EXTI settings consistent when editing.
 * PB12/PB15 share EXTI15_10; changes to another IRQ group also need it.c. */
#define BRUSH_ENCODER_A_PORT GPIOB
#define BRUSH_ENCODER_A_PIN GPIO_PIN_12
#define BRUSH_ENCODER_B_PORT GPIOB
#define BRUSH_ENCODER_B_PIN GPIO_PIN_15
#define BRUSH_ENCODER_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()
#define BRUSH_ENCODER_IRQn EXTI15_10_IRQn
#define BRUSH_ENCODER_PULL GPIO_PULLUP
#define BRUSH_CURRENT_PORT GPIOA
#define BRUSH_CURRENT_PIN GPIO_PIN_4
#define BRUSH_CURRENT_GPIO_CLOCK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BRUSH_CURRENT_ADC_CHANNEL 4U /* ADC1_IN4, not the GPIO pin mask */

/* Actual x4 edge counts per BRUSH OUTPUT SHAFT revolution. 0 = unknown.
 * Teach temporarily with H,N, one manual revolution, T; then put CPR here
 * for persistence. No guessed encoder PPR or gearbox ratio is used. */
#ifndef BRUSH_COUNTS_PER_REV
#define BRUSH_COUNTS_PER_REV 0U
#endif
/* This car reports negative raw A/B counts under the forward brush command.
 * Correct feedback polarity only; this does not change H-bridge direction.
 * Keep A/B wiring unchanged with -1. Use +1 for the opposite wiring. */
#ifndef BRUSH_ENCODER_SIGN
#define BRUSH_ENCODER_SIGN (-1)
#endif
#ifndef BRUSH_AUTO_ENABLE_ON_BOOT
#define BRUSH_AUTO_ENABLE_ON_BOOT 0U /* Q arms pulse/RPM monitoring; E requires CPR; O disarms. */
#endif
#define BRUSH_SAMPLE_MS 100U
/* Initial bench-test thresholds, not a hardware reliability guarantee. */
#define BRUSH_ERROR_WINDOW_MS 1000U
#define BRUSH_ERROR_WINDOW_LIMIT 8U
#define BRUSH_ERROR_CONSECUTIVE_LIMIT 3U
#define BRUSH_ERROR_WARN_MS 1000U
#define BRUSH_STARTUP_GRACE_MS 1000U
#define BRUSH_STALL_RPM_X10 50 /* 5.0 rpm; tune for the real shaft */
#define BRUSH_STALL_CONFIRM_MS 600U
#define BRUSH_WRONG_DIRECTION_CONFIRM_MS 300U
#define BRUSH_HOLD_REPORT_MS 2000U
#define BRUSH_AUTO_MAX_ATTEMPTS 5U
#define BRUSH_HEALTHY_RESET_MS 10000U
#define BRUSH_REVERSE_MAX_MS 6000U
#define BRUSH_REVERSE_NO_PROGRESS_MS 800U

/* PA4 is always sampled, but current protection is opt-in. These values
 * refer to voltage AT THE MCU PIN after any divider/amplifier. */
#ifndef BRUSH_CURRENT_ENABLE
#define BRUSH_CURRENT_ENABLE 0U
#endif
#define BRUSH_ADC_VREF_MV 3300U
#ifndef BRUSH_CURRENT_ZERO_MV
#define BRUSH_CURRENT_ZERO_MV 1650U
#endif
#ifndef BRUSH_CURRENT_MV_PER_AMP
#define BRUSH_CURRENT_MV_PER_AMP 0U /* Unknown sensor: set its calibrated value. */
#endif
#ifndef BRUSH_CURRENT_STALL_MA
#define BRUSH_CURRENT_STALL_MA 0U /* Must be measured, not guessed. */
#endif
#define BRUSH_CURRENT_CONFIRM_MS 150U
#define BRUSH_ADC_STALE_MS 300U

#if BRUSH_ENCODER_SIGN != 1 && BRUSH_ENCODER_SIGN != -1
#error "BRUSH_ENCODER_SIGN must be +1 or -1"
#endif
#if BRUSH_COUNTS_PER_REV > 1000000U
#error "CPR exceeds supported limit"
#endif
#if BRUSH_AUTO_ENABLE_ON_BOOT && BRUSH_COUNTS_PER_REV < 4
#error "Calibrate BRUSH_COUNTS_PER_REV before enabling automatic boot protection"
#endif
#if BRUSH_CURRENT_ENABLE && (BRUSH_CURRENT_MV_PER_AMP == 0 || BRUSH_CURRENT_STALL_MA == 0)
#error "Current protection requires calibrated sensitivity and trip current"
#endif
#if BRUSH_CURRENT_ADC_CHANNEL > 15
#error "Use an external ADC1 channel 0..15 and its corresponding analog pin"
#endif
#endif
