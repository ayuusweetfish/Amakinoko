#include <stm32g0xx_hal.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// #define RELEASE
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

__attribute__ ((section(".RamFunc")))
static inline void spin_delay(uint32_t cycles)
{
  __asm__ volatile (
    "   cmp %[cycles], #5\n"
    "   ble 2f\n"
    "   sub %[cycles], #5\n"
    "   lsr %[cycles], #2\n"
    "1: sub %[cycles], #1\n"
    "   nop\n"
    "   bne 1b\n"   // 2 cycles if taken
    "2: \n"
    : [cycles] "+l" (cycles)
    : // No output
    : "cc"
  );
}
__attribute__ ((section(".RamFunc")))
static inline void delay_us(uint32_t us)
{
  spin_delay(us * 4);
}

static I2C_HandleTypeDef i2c1;

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  // Voltage scaling mostly affects clocks (HSE, PLL) and flash latency
  // Refer to RM0454 Rev. 5
  // - Ch. 4.1.4, "Dynamic voltage scaling management"
  // - Ch. 3.3.4, "FLASH read access latency"
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc_init.PLL.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // 16 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV4;     // 4 MHz
  clk_init.APB1CLKDivider = RCC_HCLK_DIV4;      // 4 MHz
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_0);

if (0) {
  // ======== I2C ========
  __HAL_RCC_I2C1_CLK_ENABLE();
  i2c1 = (I2C_HandleTypeDef){
    .Instance = I2C1,
    .Init = {
      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766
      // APB = 4 MHz      -- 0.25 us
      // PRESC = 1        -- 0.5 us
      // SCLH = SCLL = 7  -- 4 us  (125 kHz)
      // SCLDEL = SDADEL = 1
      // <PRESC>0<SCLDEL><SDADEL><SCLH>~<SCLL>~
      .Timing = 0x10110707,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c1);
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6 | GPIO_PIN_7,
    .Alternate = GPIO_AF6_I2C1,
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_PULLUP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  void check_device_ready(uint8_t addr, const char *name)
  {
    HAL_StatusTypeDef device_ready = HAL_I2C_IsDeviceReady(&i2c1, addr, 3, 1000);
    swv_printf("%s %u (%u)\n", name, (unsigned)device_ready, (unsigned)i2c1.ErrorCode);
  }
  check_device_ready(0b0100011 << 1, "BH1750FVI");
  check_device_ready(0b1000100 << 1, "SHT30");
  check_device_ready(0b1011100 << 1, "LPS22HH");
}

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_9,
    .Mode = GPIO_MODE_OUTPUT_OD,
    .Pull = GPIO_PULLUP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  bool parity = 0;
  while (1) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, parity ^= 1);
    swv_printf("hello\n");
    HAL_Delay(1000);
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
