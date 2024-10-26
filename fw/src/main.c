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
  spin_delay(us * 64);
}

static I2C_HandleTypeDef i2c1;
static TIM_HandleTypeDef tim3;

inline void run();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // 64 MHz
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;         // 64 MHz
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;          // 64 MHz
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  // ======== Timer ========
  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3 = (TIM_HandleTypeDef){
    .Instance = TIM3,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 80 - 1, // 800 kHz
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_Base_Init(&tim3);
  HAL_TIM_Base_Start(&tim3);

  // ======== I2C ========
  __HAL_RCC_I2C1_CLK_ENABLE();
  i2c1 = (I2C_HandleTypeDef){
    .Instance = I2C1,
    .Init = {
      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766
      // APB = 64 MHz     -- 0.015625 us
      // PRESC = 15       -- 0.25 us
      // SCLH = SCLL = 4  -- 1.25 us  (400 kHz)
      // SCLDEL = SDADEL = 1
      // <PRESC>0<SCLDEL><SDADEL><SCLH>~<SCLL>~
      .Timing = 0xF0110404,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c1);
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_6 | GPIO_PIN_7,
    .Alternate = GPIO_AF6_I2C1,
    .Mode = GPIO_MODE_AF_OD,
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

while (1) {
  unsigned r1;
  // Clock stretching disabled, high repeatability
  r1 = HAL_I2C_Master_Transmit(&i2c1, 0b1000100 << 1, (uint8_t []){0x24, 0x00}, 2, 1000);
  swv_printf("SHT30 write %u %u\n", r1, i2c1.ErrorCode);

  r1 = HAL_I2C_Master_Transmit(&i2c1, 0b0100011 << 1, (uint8_t []){0b00010011}, 1, 1000);
  swv_printf("BH1750FVI write %u %u\n", r1, i2c1.ErrorCode);

  // CTRL_REG2: IF_ADD_INC | ONE_SHOT
  r1 = HAL_I2C_Mem_Write(&i2c1, 0b1011100 << 1, 0x11, I2C_MEMADD_SIZE_8BIT, (uint8_t []){0b00010001}, 1, 1000);
  HAL_Delay(20);

  uint8_t buf[10] = { 0 };

  // STATUS
  r1 = HAL_I2C_Mem_Read(&i2c1, 0b1011100 << 1, 0x27, I2C_MEMADD_SIZE_8BIT, buf, 6, 1000);
  uint8_t status = buf[0] & 0x03; // T_DA | P_DA
  uint32_t reading_p =
    ((uint32_t)buf[1] <<  0) |
    ((uint32_t)buf[2] <<  8) |
    ((uint32_t)buf[3] << 16);
  uint32_t reading_t =
    ((uint32_t)buf[4] <<  0) |
    ((uint32_t)buf[5] <<  8);
  swv_printf("%u %u %u %08x %08x\n", r1, i2c1.ErrorCode, status, reading_p, reading_t);
  swv_printf("p = %u Pa\nt = %u.%02u degC\n",
    reading_p * 100 / 4096,
    reading_t / 100, reading_t % 100);

  r1 = HAL_I2C_Master_Receive(&i2c1, 0b0100011 << 1, buf, 2, 1000);
  uint16_t lx = ((((uint32_t)buf[0] << 8) | buf[1]) * 5 + 3) / 6;
  swv_printf("%u %u %02x %02x\n%u lx\n", r1, i2c1.ErrorCode, buf[0], buf[1], lx);

{
  r1 = HAL_I2C_Master_Receive(&i2c1, 0b1000100 << 1, buf, 6, 1000);
  uint32_t reading_t = (((uint32_t)buf[0] << 8) | buf[1]);
  uint32_t reading_h = (((uint32_t)buf[3] << 8) | buf[4]);
  uint32_t cent_degC = -4500 + reading_t * 17500 / 65535;
  uint32_t cent_hum = 10000 * reading_h / 65535;
  swv_printf("%u %u %u %u\n", r1, i2c1.ErrorCode, reading_t, reading_h);
  swv_printf("t = %u.%02u degC\nh = %3u.%02u %%\n",
    cent_degC / 100, cent_degC % 100,
    cent_hum / 100, cent_hum % 100);
}

  HAL_Delay(200);
}

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_9,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  uint32_t tick = HAL_GetTick();
  while (1) {
    __disable_irq();
    run();
    __enable_irq();

    uint32_t cur;
    while ((cur = HAL_GetTick()) - tick < 10)
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    tick = cur;
  }
}

// Within tolerance of both WS2812 and WS2812B
#define OUTPUT_BIT(_port, _bit, _set) do { \
  uint32_t bit = (_bit); \
  uint32_t wait = ((_set) ? 48 : 24); \
  while ((TIM3->SR & TIM_SR_UIF) == 0) { } \
  TIM3->SR = ~TIM_SR_UIF; \
  (_port)->BSRR = bit; \
  while (TIM3->CNT < wait) { } \
  (_port)->BSRR = bit << 16; \
} while (0)

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__ ((section(".RamFunc")))
void run()
{
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
  static const uint8_t seq_len = (sizeof seq) / (sizeof seq[0]);

  static uint32_t frame = 0, frame_subdiv = 0;
  if (++frame_subdiv == 8) {
    frame_subdiv = 0;
    frame++;
    if (frame == seq_len) frame = 0;
  }

#define N 48
  uint8_t buf[N][3];
  for (int i = 0, f = frame; i < N; i++, f = (f == seq_len - 1 ? 0 : f + 1)) {
    uint8_t g = 0, r = 0, b = 0;

    uint8_t f1 = (f == seq_len - 1 ? 0 : f + 1);
    g = (seq[f][0] * (8 - frame_subdiv) + seq[f1][0] * frame_subdiv) + 1;
    r = (seq[f][1] * (8 - frame_subdiv) + seq[f1][1] * frame_subdiv) + 1;
    b = (seq[f][2] * (8 - frame_subdiv) + seq[f1][2] * frame_subdiv) + 1;

    buf[i][0] = g * 5;
    buf[i][1] = r * 0;
    buf[i][2] = b * 5;
  }

  TIM3->SR = ~TIM_SR_UIF;
  for (int i = 0; i < N; i++) {
    // G
    uint8_t g = buf[i][0];
    OUTPUT_BIT(GPIOB, 1 << 9, 0);
    OUTPUT_BIT(GPIOB, 1 << 9, g & 64);
    OUTPUT_BIT(GPIOB, 1 << 9, g & 32);
    OUTPUT_BIT(GPIOB, 1 << 9, g & 16);
    OUTPUT_BIT(GPIOB, 1 << 9, g & 8);
    OUTPUT_BIT(GPIOB, 1 << 9, g & 4);
    OUTPUT_BIT(GPIOB, 1 << 9, g & 2);
    OUTPUT_BIT(GPIOB, 1 << 9, g & 1);
    // R
    uint8_t r = buf[i][1];
    OUTPUT_BIT(GPIOB, 1 << 9, 0);
    OUTPUT_BIT(GPIOB, 1 << 9, r & 64);
    OUTPUT_BIT(GPIOB, 1 << 9, r & 32);
    OUTPUT_BIT(GPIOB, 1 << 9, r & 16);
    OUTPUT_BIT(GPIOB, 1 << 9, r & 8);
    OUTPUT_BIT(GPIOB, 1 << 9, r & 4);
    OUTPUT_BIT(GPIOB, 1 << 9, r & 2);
    OUTPUT_BIT(GPIOB, 1 << 9, r & 1);
    // B
    uint8_t b = buf[i][2];
    OUTPUT_BIT(GPIOB, 1 << 9, 0);
    OUTPUT_BIT(GPIOB, 1 << 9, b & 64);
    OUTPUT_BIT(GPIOB, 1 << 9, b & 32);
    OUTPUT_BIT(GPIOB, 1 << 9, b & 16);
    OUTPUT_BIT(GPIOB, 1 << 9, b & 8);
    OUTPUT_BIT(GPIOB, 1 << 9, b & 4);
    OUTPUT_BIT(GPIOB, 1 << 9, b & 2);
    OUTPUT_BIT(GPIOB, 1 << 9, b & 1);
  }
}
#pragma GCC pop_options

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