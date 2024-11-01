// Board is Rev. 1

#include <stm32g0xx_hal.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "uxn.h"
#include "../../misc/mumu/mumu.h"

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

// Pull a set of pins to a given level and set them as input
static inline void pull_electrodes_port(GPIO_TypeDef *port, uint32_t pins, bool level)
{
  GPIO_InitTypeDef gpio_init = (GPIO_InitTypeDef){
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pin = pins,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(port, &gpio_init);
  HAL_GPIO_WritePin(port, pins, level);
  gpio_init.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(port, &gpio_init);
}

static inline void pull_electrodes(bool level)
{
  pull_electrodes_port(GPIOA,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
    level);
}

#define N_ELECTRODES 4
#define BTN_OUT_PORT GPIOC
#define BTN_OUT_PIN  GPIO_PIN_15

#pragma GCC push_options
#pragma GCC optimize("O3")
static inline void cap_sense(uint16_t cap_sum[N_ELECTRODES])
{
  struct record_t {
    uint16_t t;
    uint16_t v;
  } record[16];

  uint16_t cap[4] = { 0 };
  for (int i = 0; i < N_ELECTRODES; i++) cap_sum[i] = 0;

  static const uint16_t MASK[N_ELECTRODES] = {
    1 <<  0,
    1 <<  1,
    1 <<  2,
    1 <<  3,
  };
  static const uint16_t FULL_MASK =
    MASK[ 0] | MASK[ 1] | MASK[ 2] | MASK[ 3];

  inline void toggle(const bool level) {
    pull_electrodes(1 - level); // Pull to the opposite level before reading
    int n_records = 0;
    uint16_t last_v = (level == 1 ? ~FULL_MASK : FULL_MASK);
    record[n_records] = (struct record_t){.t = (uint16_t)-1, .v = last_v};
    __disable_irq();
    HAL_GPIO_WritePin(BTN_OUT_PORT, BTN_OUT_PIN, level);
    for (int i = 0; i < 100; i++) {
      uint32_t combined_v = GPIOA->IDR;
      uint16_t cur_v = (level == 1 ? (combined_v | last_v) : (combined_v & last_v));
      if (last_v != cur_v) n_records++;
      record[n_records] = (struct record_t){.t = i, .v = cur_v};
      last_v = cur_v;
    }
    __enable_irq();
    for (int j = 0; j < N_ELECTRODES; j++) cap[j] = 0xffff;
    for (int i = 1; i <= n_records; i++) {
      uint16_t t = record[i - 1].t + 1;
      uint16_t diff = record[i - 1].v ^ record[i].v;
      for (int j = 0; j < N_ELECTRODES; j++)
        if (diff & MASK[j]) cap[j] = t;
    }
    for (int j = 0; j < N_ELECTRODES; j++)
      if (cap_sum[j] == 0xffff || cap[j] == 0xffff) cap_sum[j] = 0xffff;
      else cap_sum[j] += cap[j];
  }
  for (int its = 0; its < 5; its++) {
    toggle(1);
    toggle(0);
  }

  // Maintain a base value for each of the buttons

  // The base value increases by 1 every 16 iterations (~ 0.5 second)
  static const uint32_t BASE_MULT = 16;
  static uint32_t base[N_ELECTRODES] = { UINT32_MAX };
  if (base[0] == UINT32_MAX) {
    for (int j = 0; j < N_ELECTRODES; j++) base[j] = cap_sum[j] * BASE_MULT;
  } else {
    for (int j = 0; j < N_ELECTRODES; j++) {
      base[j] += 1;
      uint32_t lower = cap_sum[j] * BASE_MULT;
      if (base[j] > lower)
        base[j] = lower + (base[j] - lower) * 3 / 4;
    }
  }
  for (int j = 0; j < N_ELECTRODES; j++) {
    uint32_t base_scaled = base[j] / BASE_MULT;
    cap_sum[j] = (cap_sum[j] > base_scaled ? cap_sum[j] - base_scaled : 0);
  }
}
#pragma GCC pop_options

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

  // ======== Capacitive touch sensing ========
  // BTN_OUT
  gpio_init = (GPIO_InitTypeDef){
    .Pin = BTN_OUT_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  };
  HAL_GPIO_Init(BTN_OUT_PORT, &gpio_init);
  HAL_GPIO_WritePin(BTN_OUT_PORT, BTN_OUT_PIN, 0);

  // For the electrodes, refer to `pull_electrodes()`

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

  bool check_device_ready(uint8_t addr, const char *name)
  {
    HAL_StatusTypeDef device_ready = HAL_I2C_IsDeviceReady(&i2c1, addr, 3, 1000);
    if (device_ready != HAL_OK) {
      swv_printf("%s %u (%u)\n", name, (unsigned)device_ready, (unsigned)i2c1.ErrorCode);
      return false;
    }
    return true;
  }
  check_device_ready(0b0100011 << 1, "BH1750FVI");
  check_device_ready(0b1000100 << 1, "SHT30");
  check_device_ready(0b1011100 << 1, "LPS22HH");

  void sensors_start()
  {
    unsigned result;

    // LPS22HH: One-shot
    // CTRL_REG2 = IF_ADD_INC | ONE_SHOT
    result = HAL_I2C_Mem_Write(&i2c1, 0b1011100 << 1, 0x11, I2C_MEMADD_SIZE_8BIT, (uint8_t []){0b00010001}, 1, 1000);
    if (result != HAL_OK) swv_printf("LPS22HH write %u %u\n", result, i2c1.ErrorCode);

    // SHT30: Clock stretching disabled, high repeatability
    result = HAL_I2C_Master_Transmit(&i2c1, 0b1000100 << 1, (uint8_t []){0x24, 0x00}, 2, 1000);
    if (result != HAL_OK) swv_printf("SHT30 write %u %u\n", result, i2c1.ErrorCode);

    // BH1750FVI: One Time L-Resolution Mode
    result = HAL_I2C_Master_Transmit(&i2c1, 0b0100011 << 1, (uint8_t []){0b00100011}, 1, 1000);
    if (result != HAL_OK) swv_printf("BH1750FVI write %u %u\n", result, i2c1.ErrorCode);
  }

  struct sensors_readings {
    uint32_t p;   // (LPS22HH) Pressure (Pa)
    uint32_t t1;  // (LPS22HH) Temperature (0.01 degC)
    uint32_t t2;  // (SHT30) Temperature (0.01 degC)
    uint32_t h;   // (SHT30) Humidity (0.01 %RH)
    uint32_t i;   // (BH1750FVI) Illuminance (lx)
  };

  bool sensors_read(struct sensors_readings *r)
  {
    unsigned result;
    uint8_t buf[10];

    // LPS22HH
    result = HAL_I2C_Mem_Read(&i2c1, 0b1011100 << 1, 0x27, I2C_MEMADD_SIZE_8BIT, buf, 6, 1000);
    if (result != HAL_OK) return false;
    uint8_t status = buf[0] & 0x03; // STATUS should have T_DA | P_DA set
    if (status != 0x03) return false;
    uint32_t reading_p  =
      ((uint32_t)buf[1] <<  0) |
      ((uint32_t)buf[2] <<  8) |
      ((uint32_t)buf[3] << 16);
    uint32_t reading_t1 =
      ((uint32_t)buf[4] <<  0) |
      ((uint32_t)buf[5] <<  8);
    r->p  = reading_p * 100 / 4096;
    r->t1 = reading_t1;

    // SHT30
    result = HAL_I2C_Master_Receive(&i2c1, 0b1000100 << 1, buf, 6, 1000);
    uint32_t reading_t2 = (((uint32_t)buf[0] << 8) | buf[1]);
    uint32_t reading_h  = (((uint32_t)buf[3] << 8) | buf[4]);
    r->t2 = -4500 + reading_t2 * 17500 / 65535;
    r->h  = 10000 * reading_h / 65535;

    // BH1750FVI
    result = HAL_I2C_Master_Receive(&i2c1, 0b0100011 << 1, buf, 2, 1000);
    if (result != HAL_OK) return false;
    r->i = ((((uint32_t)buf[0] << 8) | buf[1]) * 5 + 3) / 6;

    return true;
  }

if (1) {
  static mumu_vm_t m;

  static const uint32_t c[] = {
    0x22000001, // subn 0, 0, 1
    0x00000000, // sc 0
  };
  m.c = c;
  m.pc = 0;

  m.m[0] = 0xaa558800;
  swv_printf("running!\n");
  mumu_run(&m);
  swv_printf("%08x\n", m.m[0]); // 0xaa5587ff

  static uint32_t c2[100] = { 0 };
  for (int i = 0; i < 99; i++) c2[i] = 0x22000001;
  m.c = c2;
  m.m[0] = 0x00000000;
  uint32_t t0 = HAL_GetTick();
  for (int i = 0; i < 10000; i++) {
    m.pc = 0;
    mumu_run(&m);
  }
  swv_printf("%u\n", HAL_GetTick() - t0); // 961 = 1.04M instructions per second
  swv_printf("%08x\n", m.m[0]); // -99 * 10000 = 0xfff0e4d0
}

if (0) {
  uint8_t *r = uxn_instance()->ram;
  uint16_t pc = 0x100;
  r[pc++] = 0xA0;  // LIT2 02 00
  r[pc++] = 0x02;
  r[pc++] = 0x00;
  r[pc++] = 0x14;  // LDA
  r[pc++] = 0x18;  // ADD
  r[pc++] = 0xA0;  // LIT2 42 00 ( This wraps around, modulo 2048 = 0x800 )
  r[pc++] = 0x42;
  r[pc++] = 0x00;
  r[pc++] = 0x95;  // STAk
  r[pc++] = 0x22;  // POP2
  r[pc++] = 0x00;  // BRK
  for (int i = 0; i < 10; i++) {
    uxn_instance()->wst.dat[0x00] = i;
    uxn_instance()->wst.ptr = 0x01;
    uxn_eval(0x100);
    swv_printf("%02x %02x %02x\n", uxn_instance()->wst.ptr, uxn_instance()->wst.dat[0x00], r[0x200]);
  }
  // 1e6 ~ 2e6 instructions per second
  for (int i = 0; i < 256; i++) {
    uint8_t opcode = i & 0x1f;
    r[i] = opcode == 0x00 || (opcode >= 0x0c && opcode <= 0x0f) || (opcode == 0x11 || opcode == 0x13 || opcode == 0x15) ? 0x18 : i;
  }
  r[256] = 0x00;
  uint32_t t0 = HAL_GetTick();
  for (int i = 0; i < 10000; i++) uxn_eval(0x0);
  swv_printf("%u\n", HAL_GetTick() - t0); // ~2000
}

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_9,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  sensors_start();

  uint32_t tick = HAL_GetTick();
  uint32_t last_sensors_start = tick;
  while (1) {
    __disable_irq();
    run();
    __enable_irq();

    uint32_t cur = HAL_GetTick();
    if (cur >= last_sensors_start + 20) {
      struct sensors_readings r;
      uint16_t cap[4];
      cap_sense(cap);
      bool valid = sensors_read(&r);
      if (valid) {
        swv_printf("p=%u t=%u %u h=%u i=%u | ", r.p, r.t1, r.t2, r.h, r.i);
        for (int j = 0; j < 4; j++) swv_printf("%3d%c", cap[j] < 999 ? cap[j] : 999, j == 3 ? '\n' : ' ');
        sensors_start();
        last_sensors_start = cur;
      } else {
        swv_printf("Reading invalid! Check connections\n");
      }
    }
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

#define N 24
  static uint32_t frame = 0, frame_subdiv = 0;
  if (++frame_subdiv == 8) {
    frame_subdiv = 0;
    frame++;
    if (frame == seq_len /* N + 2 */) frame = 0;
  }

  uint8_t buf[N][3];
  for (int i = 0, f = frame; i < N; i++, f = (f == seq_len - 1 ? 0 : f + 1)) {
    uint8_t g = 0, r = 0, b = 0;

    uint8_t f1 = (f == seq_len - 1 ? 0 : f + 1);
    g = (seq[f][0] * (8 - frame_subdiv) + seq[f1][0] * frame_subdiv) + 1;
    r = (seq[f][1] * (8 - frame_subdiv) + seq[f1][1] * frame_subdiv) + 1;
    b = (seq[f][2] * (8 - frame_subdiv) + seq[f1][2] * frame_subdiv) + 1;

    buf[i][0] = g * 5;
    buf[i][1] = r * 5;
    buf[i][2] = b * 5;
  }
if (0) {
  for (int i = 0; i < N; i++) {
    buf[i][0] = buf[i][1] = buf[i][2] = 0;
  }
  for (int i = 0; i < N; i++) if (i >= (int)frame - 2 && i <= frame) {
    uint8_t value = 120;
    if (i == (int)frame - 2) value = (7 - frame_subdiv) * 15;
    if (i == frame + 0) value = frame_subdiv * 15;
    buf[i][0] = value >> 1;
    buf[i][2] = value;
  }
}

  GPIOB->BSRR = (1 << 9) << 16; // Reset code, drive low
  delay_us(60); // Nominal length is 50 us, leave some tolerance

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

  GPIOB->BSRR = (1 << 9); // Release line, write high, to avoid continuous current
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
