// Mushroom testrun
// Board is Curtain Rev. 1

#include <stm32g0xx_hal.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define RELEASE
#ifndef RELEASE
#define _release_inline
static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}
#else
#define _release_inline inline
#define swv_printf(...)
#endif

TIM_HandleTypeDef tim3;

inline void run();

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_init.PLL.PLLM = RCC_PLLM_DIV1;  // VCO input 16 MHz (2.66 ~ 16 MHz)
  osc_init.PLL.PLLN = 8;              // VCO output 128 MHz (64 ~ 344 MHz)
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;  // PLLPCLK 64 MHz
  osc_init.PLL.PLLR = RCC_PLLR_DIV2;  // PLLRCLK 64 MHz
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 64 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // ======== GPIO (Act LEDs) ========
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOF, &gpio_init);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, 1);

  // ======== GPIO (Light strips) ========
  // GPIOA
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
           GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 |
           GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOA, &gpio_init);
  HAL_GPIO_WritePin(GPIOA, gpio_init.Pin, 0);

  // GPIOB
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
           GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
           GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOB, &gpio_init);
  HAL_GPIO_WritePin(GPIOB, gpio_init.Pin, 0);

  // GPIOC
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6 | GPIO_PIN_7,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOC, &gpio_init);
  HAL_GPIO_WritePin(GPIOC, gpio_init.Pin, 0);

  // GPIOD
  gpio_init = (GPIO_InitTypeDef){
    .Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_HIGH,
  };
  HAL_GPIO_Init(GPIOD, &gpio_init);
  HAL_GPIO_WritePin(GPIOD, gpio_init.Pin, 0);

  // ======== Timer ========
  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3 = (TIM_HandleTypeDef){
    .Instance = TIM3,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 80 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_Base_Init(&tim3);
  HAL_TIM_Base_Start(&tim3);

  inline uint32_t my_rand() {
    uint32_t seed = 2451023;
    seed = seed * 1103515245 + 12345;
    return seed & 0x7fffffff;
  }

  int count = 0;

  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, 0);

  uint32_t tick = HAL_GetTick();

  while (1) {
    __disable_irq();
    run();
    __enable_irq();

    uint32_t cur;
    while ((cur = HAL_GetTick()) - tick < 20)
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    tick = cur;

    if (++count % 50 == 0) {
      if (count == 100) count = 0;
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, count == 0 ? 0 : 1);
    }
  }
}

// Within tolerance of both WS2812 and WS2812B
// NOTE: When `_inv` is `~`, all outputs are inverted
// to match the external open-drain (NMOS with pull-up) output structure
// (maybe also set 1 instead of 0 at port initialisation?)
// This is not needed during testing
#define OUTPUT_BITS(_portA, _portB, _bitsA, _bitsB, _inv) do { \
  /* Output a bit vector for up to 16 lights */ \
  uint16_t bitsA = (_bitsA); \
  \
  while ((TIM3->SR & TIM_SR_UIF) == 0) { } \
  TIM3->SR = ~TIM_SR_UIF; \
  GPIO##_portA->ODR = (uint16_t)(_inv 0xffff); \
  GPIO##_portB->ODR = (uint16_t)(_inv 0xffff); \
  \
  uint16_t bitsB = (_bitsB); \
  \
  while (TIM3->CNT < 24) { } \
  GPIO##_portA->ODR = (uint16_t)(_inv bitsA); \
  GPIO##_portB->ODR = (uint16_t)(_inv bitsB); \
  \
  while (TIM3->CNT < 48) { } \
  GPIO##_portA->ODR = (uint16_t)(_inv 0x0000); \
  GPIO##_portB->ODR = (uint16_t)(_inv 0x0000); \
} while (0)

#pragma GCC optimize("O3")
void run()
{
  static uint32_t frame = 0, frame_subdiv = 0;
  if (++frame_subdiv == 4) {
    frame_subdiv = 0;
    frame++;
    if (frame == 9) frame = 0;
  }

#define N 30
  uint8_t buf[N][3];
  for (int i = 0, f = frame; i < N; i++, f = (f == 8 ? 0 : f + 1)) {
    uint8_t g = 0, r = 0, b = 0;

    static const uint8_t seq[9][3] = {
      {3, 0, 0},
      {2, 1, 0},
      {1, 2, 0},
      {0, 3, 0},
      {0, 2, 1},
      {0, 1, 2},
      {0, 0, 3},
      {1, 0, 2},
      {2, 0, 1},
    };
    if (frame_subdiv < 2) {
      g = seq[f][0] * 2 + 1;
      r = seq[f][1] * 2 + 1;
      b = seq[f][2] * 2 + 1;
    } else {
      uint8_t f1 = (f == 8 ? 0 : f + 1);
      g = seq[f][0] + seq[f1][0] + 1;
      r = seq[f][1] + seq[f1][1] + 1;
      b = seq[f][2] + seq[f1][2] + 1;
    }

    buf[i][0] = g;
    buf[i][1] = r;
    buf[i][2] = b;
  }

  TIM3->SR = ~TIM_SR_UIF;
  for (int i = 0; i < N; i++) {
    uint8_t g = buf[i][0];
    uint8_t r = buf[i][1];
    uint8_t b = buf[i][2];
    // G
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, (g & 4) == 0 ? 0x0000 : 0xffff, 0x0000,);
    OUTPUT_BITS(A, B, (g & 2) == 0 ? 0x0000 : 0xffff, 0x0000,);
    OUTPUT_BITS(A, B, (g & 1) == 0 ? 0x0000 : 0xffff, 0x0000,);
    // R
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, (r & 4) == 0 ? 0x0000 : 0xffff, 0x0000,);
    OUTPUT_BITS(A, B, (r & 2) == 0 ? 0x0000 : 0xffff, 0x0000,);
    OUTPUT_BITS(A, B, (r & 1) == 0 ? 0x0000 : 0xffff, 0x0000,);
    // B
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, 0x0000, 0x0000,);
    OUTPUT_BITS(A, B, (b & 4) == 0 ? 0x0000 : 0xffff, 0x0000,);
    OUTPUT_BITS(A, B, (b & 2) == 0 ? 0x0000 : 0xffff, 0x0000,);
    OUTPUT_BITS(A, B, (b & 1) == 0 ? 0x0000 : 0xffff, 0x0000,);
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void WWDG_IRQHandler() { while (1) { } }
void RTC_TAMP_IRQHandler() { while (1) { } }
void FLASH_IRQHandler() { while (1) { } }
void RCC_IRQHandler() { while (1) { } }
void EXTI0_1_IRQHandler() { while (1) { } }
void EXTI2_3_IRQHandler() { while (1) { } }
void EXTI4_15_IRQHandler() { while (1) { } }
void DMA1_Channel1_IRQHandler() { while (1) { } }
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void DMA1_Ch4_5_DMAMUX1_OVR_IRQHandler() { while (1) { } }
void ADC1_IRQHandler() { while (1) { } }
void TIM1_BRK_UP_TRG_COM_IRQHandler() { while (1) { } }
void TIM1_CC_IRQHandler() { while (1) { } }
void TIM3_IRQHandler() { while (1) { } }
void TIM14_IRQHandler() { while (1) { } }
void TIM16_IRQHandler() { while (1) { } }
void TIM17_IRQHandler() { while (1) { } }
void I2C1_IRQHandler() { while (1) { } }
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
void USART2_IRQHandler() { while (1) { } }
