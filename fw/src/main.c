#include <stm32g0xx_hal.h>
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MUMU_ROM_SIZE 1024
#define MUMU_RAM_SIZE 512
static volatile bool mumu_run_timeout;
#define MUMU_TIMEOUT_CONDITION mumu_run_timeout
#include "../../misc/mumu/mumu.h"

#define REV 2

extern uint32_t _sidata, _sdata, _edata;
#define FLASH_START_ADDR    0x08000000
#define FW_END_ADDR         ((uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata))
#define STORAGE_START_ADDR  ((FW_END_ADDR + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1))
#define STORAGE_SIZE        4096
#define FLASH_END_ADDR      (FLASH_START_ADDR + (32 * 1024))
_Static_assert(STORAGE_SIZE % FLASH_PAGE_SIZE == 0);

#define STORAGE_ROM_LEN_OFFS  (STORAGE_SIZE - 4)
#define STORAGE_SRC_LEN_OFFS  (STORAGE_SIZE - 2)
#define STORAGE_ROM_OFFS      (0)
#define STORAGE_SRC_OFFS      (MUMU_ROM_SIZE)
#define STORAGE_ROM_LEN       ((uint16_t *)(STORAGE_START_ADDR + STORAGE_ROM_LEN_OFFS))
#define STORAGE_SRC_LEN       ((uint16_t *)(STORAGE_START_ADDR + STORAGE_SRC_LEN_OFFS))
#define STORAGE_ROM_START     ((void *)(STORAGE_START_ADDR + STORAGE_ROM_OFFS))
#define STORAGE_SRC_START     ((void *)(STORAGE_START_ADDR + STORAGE_SRC_OFFS))
#define STORAGE_ROM_MAX_LEN   (MUMU_ROM_SIZE)
#define STORAGE_SRC_MAX_LEN   (STORAGE_SIZE - 4 - STORAGE_SRC_OFFS)

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
__attribute__ ((format(printf, 1, 2)))
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

static UART_HandleTypeDef uart2;

#define N_LIGHTS 24
static mumu_vm_t m = { 0 };
static uint32_t *const lights = m.m + 256;
static uint8_t timeout_cooldown = 0;
static void output_lights();

static uint8_t rx_byte;

static void queue_tx_flush();
static void tx_readings_if_connected();

static _Atomic bool irqs_happened;
static _Atomic bool flash_rewritten;

__attribute__ ((section(".RamFunc")))
static void storage_write(uint32_t offs, const uint8_t data[256])
{
  // Silently break if offset not of a multiple of fast-write burst length (32 double-words)
  if (offs % (32 * 8) != 0) return;
  // Silently break if data is not properly aligned
  if ((uint32_t)&data[0] % 8 != 0) return;

  HAL_FLASH_Unlock();

  // Fast programming only works with the last erased page
  // https://community.st.com/t5/stm32-mcus-embedded-software/stm32g0-and-flash-typeprogram-fast-fail/m-p/122457/highlight/true#M6251
  // https://community.st.com/t5/stm32-mcus-products/stm32g0-flash-program-fast-on-non-last-erased-page/td-p/72284
  static uint32_t last_written_page = 0;
  if (offs == 0 || offs / FLASH_PAGE_SIZE != last_written_page) {
    last_written_page = offs / FLASH_PAGE_SIZE;
    uint32_t page_err;
    HAL_FLASHEx_Erase(&(FLASH_EraseInitTypeDef){
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .Banks = FLASH_BANK_1,
      .Page = (STORAGE_START_ADDR - FLASH_START_ADDR + offs) / FLASH_PAGE_SIZE,
      .NbPages = 1,
    }, &page_err);
  }

  __disable_irq();
  HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, STORAGE_START_ADDR + offs, (uint32_t)&data[0]);
  __enable_irq();
  HAL_FLASH_Lock();
}

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

#define N_ELECTRODES 4

#if REV == 1
  #define BTN_OUT_PORT  GPIOC
  #define BTN_OUT_PIN   GPIO_PIN_15
  #define TOUCH_PINS_A  ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))
  #define TOUCH_MASK_   1 << 0, 1 << 1, 1 << 2, 1 << 3,
#elif REV == 2
  #define BTN_OUT_PORT  GPIOA
  #define BTN_OUT_PIN   GPIO_PIN_6
  #define TOUCH_PINS_A  ((1 << 7) | (1 << 8) | (1 << 11) | (1 << 12))
  #define TOUCH_MASK_   1 << 7, 1 << 8, 1 << 11, 1 << 12,
#endif

static const uint16_t TOUCH_MASK[N_ELECTRODES] = { TOUCH_MASK_ };

static inline void pull_electrodes(bool level)
{
  pull_electrodes_port(GPIOA, TOUCH_PINS_A, level);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
static inline void cap_sense(uint16_t cap_sum[N_ELECTRODES])
{
  struct record_t {
    uint16_t t;
    uint16_t v;
  } record[16];

  static const uint16_t FULL_MASK =
    TOUCH_MASK[0] | TOUCH_MASK[1] | TOUCH_MASK[2] | TOUCH_MASK[3];

  uint16_t cap[4] = { 0 };
  for (int i = 0; i < N_ELECTRODES; i++) cap_sum[i] = 0;

  inline void toggle(const bool level) {
    pull_electrodes(1 - level); // Pull to the opposite level before reading
    int n_records;

    // Instead of depending on timers, we simply retry if interrupts happen midway
    do {
      n_records = 0;
      uint16_t last_v = (level == 1 ? ~FULL_MASK : FULL_MASK);
      record[n_records] = (struct record_t){.t = (uint16_t)-1, .v = last_v};

      HAL_SuspendTick();
      irqs_happened = false;
      HAL_GPIO_WritePin(BTN_OUT_PORT, BTN_OUT_PIN, level);
      for (int i = 0; i < 100; i++) {
        uint32_t combined_v = GPIOA->IDR;
        uint16_t cur_v = (level == 1 ? (combined_v | last_v) : (combined_v & last_v));
        if (last_v != cur_v) n_records++;
        record[n_records] = (struct record_t){.t = i, .v = cur_v};
        last_v = cur_v;
      }
      HAL_ResumeTick();
    } while (irqs_happened);

    for (int j = 0; j < N_ELECTRODES; j++) cap[j] = 0xffff;
    for (int i = 1; i <= n_records; i++) {
      uint16_t t = record[i - 1].t + 1;
      uint16_t diff = record[i - 1].v ^ record[i].v;
      for (int j = 0; j < N_ELECTRODES; j++)
        if (diff & TOUCH_MASK[j]) cap[j] = t;
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

static inline void adc_start()
{
  while (!(ADC1->CR & ADC_CR_ADEN)) {
    ADC1->CR = (ADC1->CR & ~(ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | ADC_CR_ADDIS)) | ADC_CR_ADEN;
  }
}
static inline uint32_t adc_poll_value()
{
  ADC1->ISR = (ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR);  // All bits of ADC_ISR are r/cw1
  ADC1->CR = ADC_CR_ADSTART;  // All bits of ADC_CR are r/s
  while (!(ADC1->ISR & ADC_FLAG_EOC)) { }
  return ADC1->DR;
}
static inline void adc_stop()
{
  ADC1->CR = (ADC1->CR & ~(ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | ADC_CR_ADDIS)) | ADC_CR_ADSTP;
  while (ADC1->CR & ADC_CR_ADSTART) { }
  ADC1->CR = (ADC1->CR & ~(ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | ADC_CR_ADDIS)) | ADC_CR_ADDIS;
  while (ADC1->CR & ADC_CR_ADEN) { }
}

#if REV == 1
  #define LED_OUT_PORT  GPIOB
  #define LED_OUT_PIN   (1 << 9)
#elif REV == 2
  #define LED_OUT_PORT  GPIOA
  #define LED_OUT_PIN   (1 << 5)
  #define CC1_ADC_CHANNEL 1
  #define CC2_ADC_CHANNEL 4
#endif

struct sensors_readings {
  uint32_t t;   // (LPS22HH, SHT30) Temperature (0.01 degC)
  uint32_t p;   // (LPS22HH) Pressure (Pa)
  uint32_t h;   // (SHT30) Humidity (0.01 %RH)
  uint32_t i;   // (BH1750FVI) Illuminance (lx)
  uint8_t c[4]; // Capacitive sensing
};
static struct sensors_readings last_readings;
static uint8_t last_readings_valid = 0; // Bitmask for the 3 sensors
static uint32_t frame_n = 0;

static inline uint16_t offs_clamp_16(uint32_t x, uint32_t offs)
{
  if (x < offs) return 0;
  x -= offs;
  return (x > 0xffff ? 0xffff : x);
}

#define TX_READINGS_LEN (14 + 1 + N_LIGHTS * 3)
static void fill_tx_readings(uint8_t buf[TX_READINGS_LEN])
{
  buf[ 0] = (frame_n >> 8) & 0xff;
  buf[ 1] = (frame_n >> 0) & 0xff;
  buf[ 2] = (last_readings.t >> 8) & 0xff;
  buf[ 3] = (last_readings.t >> 0) & 0xff;
  buf[ 4] = (last_readings.p >> 8) & 0xff;
  buf[ 5] = (last_readings.p >> 0) & 0xff;
  buf[ 6] = (last_readings.h >> 8) & 0xff;
  buf[ 7] = (last_readings.h >> 0) & 0xff;
  buf[ 8] = (last_readings.i >> 8) & 0xff;
  buf[ 9] = (last_readings.i >> 0) & 0xff;
  buf[10] = last_readings.c[0];
  buf[11] = last_readings.c[1];
  buf[12] = last_readings.c[2];
  buf[13] = last_readings.c[3];

  buf[14] = last_readings_valid | (timeout_cooldown > 0 ? 0x10 : 0x00);

  for (int i = 0; i < N_LIGHTS; i++) {
    buf[14 + 1 + i * 3 + 0] = (lights[i] >> 16) & 0xff;
    buf[14 + 1 + i * 3 + 1] = (lights[i] >>  8) & 0xff;
    buf[14 + 1 + i * 3 + 2] = (lights[i] >>  0) & 0xff;
  }
}

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

#if REV == 2
  // ======== UART ========
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Pin = GPIO_PIN_2 | GPIO_PIN_3,
    .Mode = GPIO_MODE_AF_PP,
    .Alternate = GPIO_AF1_USART2,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });
  __HAL_RCC_USART2_CLK_ENABLE();
  uart2 = (UART_HandleTypeDef){
    .Instance = USART2,
    .Init = (UART_InitTypeDef){
      .BaudRate = 921600,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .Mode = UART_MODE_TX_RX,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
    },
  };
  HAL_UART_Init(&uart2);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&uart2, &rx_byte, 1);
#endif

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

  uint32_t cc_mV_max = 0;
#if defined(CC1_ADC_CHANNEL) && defined(CC2_ADC_CHANNEL)
{
  // ======== ADC for CC pins ========
  __HAL_RCC_ADC_CLK_ENABLE();
  ADC_HandleTypeDef adc1 = {
    .Instance = ADC1,
    .Init = {
      .ClockPrescaler = ADC_CLOCK_ASYNC_DIV1,
      .Resolution = ADC_RESOLUTION_12B,
      .DataAlign = ADC_DATAALIGN_RIGHT,
      .ScanConvMode = ADC_SCAN_SEQ_FIXED,
      .EOCSelection = ADC_EOC_SINGLE_CONV,
      .LowPowerAutoWait = DISABLE,
      .LowPowerAutoPowerOff = ENABLE,
      .ContinuousConvMode = DISABLE,
      .NbrOfConversion = 1,
      .DiscontinuousConvMode = DISABLE,
      .ExternalTrigConv = ADC_SOFTWARE_START,
      .TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW,
    },
  };
  HAL_ADC_Init(&adc1);
  HAL_ADCEx_Calibration_Start(&adc1);
  adc_stop();

  ADC1->SMPR = 0b110; // 79.5 cycles

  // Internal reference
  ADC1_COMMON->CCR |= ADC_CCR_VREFEN;
  ADC1->CHSELR = (1 << 13);
  adc_start();
  uint32_t adc_vrefint = adc_poll_value();
  adc_stop();
  ADC1_COMMON->CCR &= ~ADC_CCR_VREFEN;
  // CC1
  ADC1->CHSELR = (1 << CC1_ADC_CHANNEL);
  adc_start();
  uint32_t adc_cc1 = adc_poll_value();
  adc_stop();
  // CC2
  ADC1->CHSELR = (1 << CC2_ADC_CHANNEL);
  adc_start();
  uint32_t adc_cc2 = adc_poll_value();
  adc_stop();

  uint32_t vrefint_cal = *VREFINT_CAL_ADDR;
  uint32_t cc1_mV = (uint32_t)(3000ULL * adc_cc1 * vrefint_cal / (4095 * adc_vrefint));
  uint32_t cc2_mV = (uint32_t)(3000ULL * adc_cc2 * vrefint_cal / (4095 * adc_vrefint));
  cc_mV_max = (cc1_mV > cc2_mV ? cc1_mV : cc2_mV);
  swv_printf("CC voltage %lu | %lu %lu %lu\n", cc_mV_max, adc_cc1, adc_cc2, adc_vrefint);

  __HAL_RCC_ADC_CLK_DISABLE();
}
#endif

  // ======== Timer for LED output ========
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM_HandleTypeDef tim3 = (TIM_HandleTypeDef){
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

  // ======== Timer for user program timeout ========
  __HAL_RCC_TIM17_CLK_ENABLE();
  TIM_HandleTypeDef tim17 = (TIM_HandleTypeDef){
    .Instance = TIM17,
    .Init = {
      .Prescaler = 64000 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 6 - 1, // Per 6 milliseconds
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_Base_Init(&tim17);
  TIM17->CR1 |= TIM_CR1_OPM;
  HAL_NVIC_SetPriority(TIM17_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM17_IRQn);
  __HAL_RCC_TIM17_CLK_DISABLE();

  // ======== I2C ========
  __HAL_RCC_I2C1_CLK_ENABLE();
  I2C_HandleTypeDef i2c1 = (I2C_HandleTypeDef){
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

  // Cannot be a static assertion, since this would require a link-time check
  if (STORAGE_START_ADDR + STORAGE_SIZE > FLASH_END_ADDR) {
    swv_printf("Insufficient storage!\n");
  }
  swv_printf("data end = 0x%08lx, storage start = 0x%08lx, page = %u\n",
    FW_END_ADDR, STORAGE_START_ADDR, FLASH_PAGE_SIZE);

  bool check_device_ready(uint8_t addr, const char *name)
  {
    HAL_StatusTypeDef device_ready = HAL_I2C_IsDeviceReady(&i2c1, addr, 3, 1000);
    if (device_ready != HAL_OK) {
      swv_printf("%s %lu (%lu)\n", name, (uint32_t)device_ready, (uint32_t)i2c1.ErrorCode);
      return false;
    }
    return true;
  }
  check_device_ready(0b0100011 << 1, "BH1750FVI");
  check_device_ready(0b1000100 << 1, "SHT30");
  check_device_ready(0b1011100 << 1, "LPS22HH");

  void sensors_start()
  {
    uint32_t result;

    // LPS22HH: One-shot
    // CTRL_REG2 = IF_ADD_INC | ONE_SHOT
    result = HAL_I2C_Mem_Write(&i2c1, 0b1011100 << 1, 0x11, I2C_MEMADD_SIZE_8BIT, (uint8_t []){0b00010001}, 1, 1000);
    if (result != HAL_OK) swv_printf("LPS22HH write %lu %lu\n", result, i2c1.ErrorCode);

    // SHT30: Clock stretching disabled, high repeatability
    result = HAL_I2C_Master_Transmit(&i2c1, 0b1000100 << 1, (uint8_t []){0x24, 0x00}, 2, 1000);
    if (result != HAL_OK) swv_printf("SHT30 write %lu %lu\n", result, i2c1.ErrorCode);

    // BH1750FVI: One Time L-Resolution Mode
    result = HAL_I2C_Master_Transmit(&i2c1, 0b0100011 << 1, (uint8_t []){0b00100011}, 1, 1000);
    if (result != HAL_OK) swv_printf("BH1750FVI write %lu %lu\n", result, i2c1.ErrorCode);
  }

  uint8_t sensors_read(struct sensors_readings *r)
  {
    uint32_t result;
    uint8_t buf[10];
    uint8_t valid = 0;

    // LPS22HH
    uint32_t p = 0, t1 = 0;
    result = HAL_I2C_Mem_Read(&i2c1, 0b1011100 << 1, 0x27, I2C_MEMADD_SIZE_8BIT, buf, 6, 1000);
    if (result != HAL_OK) goto _lps22hh_end;
    uint8_t status = buf[0] & 0x03; // STATUS should have T_DA | P_DA set
    if (status != 0x03) goto _lps22hh_end;
    uint32_t reading_p  =
      ((uint32_t)buf[1] <<  0) |
      ((uint32_t)buf[2] <<  8) |
      ((uint32_t)buf[3] << 16);
    uint32_t reading_t1 =
      ((uint32_t)buf[4] <<  0) |
      ((uint32_t)buf[5] <<  8);
    p  = reading_p * 100 / 4096;
    t1 = reading_t1;
    valid |= (1 << 0);
    _lps22hh_end: { }

    // SHT30
    uint32_t t2 = t1, h = 0;
    result = HAL_I2C_Master_Receive(&i2c1, 0b1000100 << 1, buf, 6, 1000);
    if (result != HAL_OK) goto _sht30_end;
    // CRC check, CRC-8/NRSC-5: MSB-first (not reflected), init = 0xff, poly = 0x31, final xor = 0x00
    static const uint8_t crc_lookup[256] = {
      0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e,
      0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d,
      0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20, 0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8,
      0xc5, 0xf4, 0xa7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb,
      0x3d, 0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71, 0x22, 0x13,
      0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5, 0x94, 0x03, 0x32, 0x61, 0x50,
      0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95,
      0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6,
      0x7a, 0x4b, 0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65, 0x54,
      0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17,
      0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2,
      0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91,
      0x47, 0x76, 0x25, 0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69,
      0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79, 0x48, 0x1b, 0x2a,
      0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef,
      0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac,
    };
    inline uint8_t crc8(uint8_t a[2]) {
      uint8_t s = 0xff;
      s = crc_lookup[s ^ a[0]];
      s = crc_lookup[s ^ a[1]];
      return s;
    }
    if (buf[2] != crc8(buf + 0) ||
        buf[5] != crc8(buf + 3)) goto _sht30_end;
    uint32_t reading_t2 = (((uint32_t)buf[0] << 8) | buf[1]);
    uint32_t reading_h  = (((uint32_t)buf[3] << 8) | buf[4]);
    t2 = -4500 + reading_t2 * 17500 / 65535;
    h  = 10000 * reading_h / 65535;
    valid |= (1 << 1);
    _sht30_end: { }

    // BH1750FVI
    uint32_t i = 0;
    result = HAL_I2C_Master_Receive(&i2c1, 0b0100011 << 1, buf, 2, 1000);
    if (result != HAL_OK) goto _bh1750fvi_end;
    i = ((((uint32_t)buf[0] << 8) | buf[1]) * 5 + 3) / 6;
    valid |= (1 << 2);
    _bh1750fvi_end: { }

    uint16_t cap[4];
    cap_sense(cap);

    uint32_t t = (t1 + t2) / 2;
    swv_printf("p=%lu t=%lu %lu h=%lu i=%lu | ", p, t1, t2, h, i);
    for (int j = 0; j < 4; j++) swv_printf("%3d%c", cap[j] < 999 ? cap[j] : 999, j == 3 ? '\n' : ' ');

    r->t = offs_clamp_16(t, 0);
    r->p = offs_clamp_16(p, 65536);
    r->h = offs_clamp_16(h, 0);
    r->i = offs_clamp_16(i, 0);
    for (int j = 0; j < 4; j++) r->c[j] = (cap[j] > 0xff ? 0xff : cap[j]);

    return valid;
  }

  HAL_GPIO_WritePin(LED_OUT_PORT, LED_OUT_PIN, 1);
  HAL_GPIO_Init(LED_OUT_PORT, &(GPIO_InitTypeDef){
    .Pin = LED_OUT_PIN,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  sensors_start();

  m.c = STORAGE_ROM_START;
  m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;

  uint32_t tick = HAL_GetTick();
  uint32_t last_sensors_start = tick;
  uint32_t last_sensors_tx = tick - 1000;
  while (1) {
    // Run Mumu VM
    m.pc = 0;

    mumu_run_timeout = false;

    // Provide readings in the lowest registers
    m.m[0] = last_readings.t;
    m.m[1] = last_readings.p;
    m.m[2] = last_readings.h;
    m.m[3] = last_readings.i;
    for (int i = 0; i < 4; i++)
      m.m[4 + i] = last_readings.c[i];

    __HAL_RCC_TIM17_CLK_ENABLE();
    // Clear timer state.
    // Setting CNT will trigger an update event (hence update interrupt) if unmasked
    TIM17->CR1 &= ~TIM_CR1_CEN;
    TIM17->DIER &= ~TIM_DIER_UIE;
    TIM17->SR = ~TIM_SR_UIF;
    TIM17->CNT = 0;
    TIM17->DIER |= TIM_DIER_UIE;
    TIM17->CR1 |= TIM_CR1_CEN;

    flash_rewritten = false;
    enum mumu_exit_t e = mumu_run(&m);

    TIM17->CR1 &= ~TIM_CR1_CEN;
    __HAL_RCC_TIM17_CLK_DISABLE();

    if (flash_rewritten) {
      // Interrupts and induced flash writes may disturb the Mumu ROM
      // In these cases we simply reset the entire VM
      // XXX: Maybe DRY?
      memset(m.m, 0, sizeof m.m);
      m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;
      timeout_cooldown = 0;
    } else {
      if (e == MUMU_EXIT_TIMEOUT) {
        swv_printf("timeout!\n");
        timeout_cooldown = 200;
        memset(m.m, 0, sizeof m.m);
        m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;
      }
      if (timeout_cooldown > 0) {
        for (int i = 0; i < N_LIGHTS; i++) {
          uint8_t phase = (i * 8 + frame_n) % 64;
          static const uint8_t lut[64] = {
            // from math import sin, pi, sqrt; print(', '.join('%d' % round(20 * sqrt(2 + sin(i/64.0 * pi*2))) for i in range(64)))
            28, 29, 30, 30, 31, 31, 32, 32, 33, 33, 34, 34, 34, 34, 35, 35, 35, 35, 35, 34, 34, 34, 34, 33, 33, 32, 32, 31, 31, 30, 30, 29, 28, 28, 27, 26, 25, 25, 24, 23, 23, 22, 22, 21, 21, 20, 20, 20, 20, 20, 20, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 28
          };
          lights[i] = (0x28 << 16) | ((uint32_t)lut[phase] << 8);
        }
        timeout_cooldown--;
      }

      frame_n++;

      // Write lights to output
      output_lights();
    }

    uint32_t cur = HAL_GetTick();
    if (cur >= last_sensors_start + 20) {
      struct sensors_readings r;
      uint8_t valid_flags = sensors_read(&r);
      if (valid_flags != 0b111) {
        swv_printf("Reading invalid (%u)! Check connections\n", (unsigned)valid_flags);
      }
      sensors_start();
      last_sensors_start = cur;
      last_readings = r;
      last_readings_valid = valid_flags;
      if (cur - last_sensors_tx >= 100) {
        tx_readings_if_connected();
        last_sensors_tx = cur;
      }
    }
    while ((cur = HAL_GetTick()) - tick < 10) {
      queue_tx_flush();
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    }
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
  __COMPILER_BARRIER(); \
} while (0)

#pragma GCC push_options
#pragma GCC optimize("O3")
__attribute__ ((section(".RamFunc")))
void output_lights()
{
  // 800 kHz = 80 cycles/bit

  // XXX: Since 800 kHz is on the same order of the serial transmission,
  // simply turnning off UART will lead to missed data;
  // Also, DMA for RX is not appropriate as we'd like to monitor
  // each byte's time of arrival.

  // A possible solution is to switch to PA4's SPI2_MOSI (AF1) and DMA the data out.
  // For now (Rev. 2), we check whether an we've been interrupted and retry if positive.
  do {
    LED_OUT_PORT->BSRR = LED_OUT_PIN << 16; // Reset code, drive low
    delay_us(60); // Nominal length is 50 us, leave some tolerance

    HAL_SuspendTick();
    irqs_happened = false;
    TIM3->SR = ~TIM_SR_UIF;
    for (int i = 0; i < N_LIGHTS; i++) {
      uint32_t v = lights[i];
      // G
      uint8_t g = (v >>  8) & 0xff;
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, 0);
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & (1 << 7));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & (1 << 6));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & (1 << 5));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & (1 << 4));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & (1 << 3));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & (1 << 2));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, g & (1 << 1));
      // R
      uint8_t r = (v >> 16) & 0xff;
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, 0);
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & (1 << 7));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & (1 << 6));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & (1 << 5));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & (1 << 4));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & (1 << 3));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & (1 << 2));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, r & (1 << 1));
      // B
      uint8_t b = (v >>  0) & 0xff;
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, 0);
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & (1 << 7));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & (1 << 6));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & (1 << 5));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & (1 << 4));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & (1 << 3));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & (1 << 2));
      OUTPUT_BIT(LED_OUT_PORT, LED_OUT_PIN, b & (1 << 1));
    }

    while ((TIM3->SR & TIM_SR_UIF) == 0) { }
    LED_OUT_PORT->BSRR = LED_OUT_PIN; // Release line, write high, to avoid continuous current
    HAL_ResumeTick();
  } while (irqs_happened);
}
#pragma GCC pop_options

/*
  0xAA - ask
    0x55 - acknowledgement

    0x56 - readings

  0xBB - bye

  0xFA - flash
    0xFC - proceed
    0xFE - error due to size limits
    0xCC - checksum
*/

static uint8_t encode_len(uint8_t *a, uint16_t x)
{
  if (x < 128) { a[0] = x; return 1; }
  else { a[0] = 128 | (x / 256); a[1] = x % 256; return 2; }
}

enum queue_tx_t {
  QUEUE_TX_HANDSHAKE,
  QUEUE_TX_FLASH_START_ERR,
  QUEUE_TX_FLASH_START_OK,
  QUEUE_TX_FLASH_CRC,
};
static uint8_t queue_tx_n = 0;
#define QUEUE_TX_CAPACITY 8
static enum queue_tx_t queue_tx_q[QUEUE_TX_CAPACITY];
static volatile atomic_flag queue_tx_lock_flag = ATOMIC_FLAG_INIT;

// For deduplication
static bool queue_tx_handshake_flag = false;

static inline void queue_tx_lock()
{
  while (!atomic_flag_test_and_set(&queue_tx_lock_flag)) { }
}

static inline void queue_tx_unlock()
{
  atomic_flag_clear(&queue_tx_lock_flag);
}

static inline void queue_tx(enum queue_tx_t e)
{
  if (queue_tx_n >= QUEUE_TX_CAPACITY - 1) return;
  queue_tx_lock();
  queue_tx_q[queue_tx_n++] = e;
  if (e == QUEUE_TX_HANDSHAKE) queue_tx_handshake_flag = true;
  queue_tx_unlock();
}

static void queue_tx_flush()
{
  if (queue_tx_n == 0) return;

  queue_tx_lock();
  int n = queue_tx_n;
  enum queue_tx_t q[QUEUE_TX_CAPACITY];
  memcpy(q, queue_tx_q, sizeof(enum queue_tx_t) * n);
  queue_tx_n = 0;
  queue_tx_unlock();

  for (int i = 0; i < n; i++) switch (q[i]) {
    case QUEUE_TX_HANDSHAKE: {
      if (!queue_tx_handshake_flag) break;
      delay_us(1000);
      // Transmit first readings
      uint8_t readings_buf[TX_READINGS_LEN];
      fill_tx_readings(readings_buf);
      HAL_UART_Transmit(&uart2, &(uint8_t){ 11 + 12 + TX_READINGS_LEN }, 1, HAL_MAX_DELAY);
      HAL_UART_Transmit(&uart2, (uint8_t *)"\x55" "Amakinoko" "\x20", 11, HAL_MAX_DELAY);
      HAL_UART_Transmit(&uart2, (uint8_t *)UID_BASE, 12, HAL_MAX_DELAY);
      HAL_UART_Transmit(&uart2, readings_buf, TX_READINGS_LEN, HAL_MAX_DELAY);
      // Transmit program binary and source text
      uint8_t l[2];
      uint16_t rom_size = *STORAGE_ROM_LEN;
      uint16_t src_size = *STORAGE_SRC_LEN;
      if (rom_size > MUMU_ROM_SIZE || src_size > STORAGE_SRC_MAX_LEN)
        rom_size = src_size = 0;
      HAL_UART_Transmit(&uart2, l, encode_len(l, rom_size), HAL_MAX_DELAY);
      HAL_UART_Transmit(&uart2, STORAGE_ROM_START, rom_size, HAL_MAX_DELAY);
      HAL_UART_Transmit(&uart2, l, encode_len(l, src_size), HAL_MAX_DELAY);
      HAL_UART_Transmit(&uart2, STORAGE_SRC_START, src_size, HAL_MAX_DELAY);
      queue_tx_handshake_flag = false;
      break;
    }
    case QUEUE_TX_FLASH_START_ERR:
    case QUEUE_TX_FLASH_START_OK: {
      uint8_t code = (q[i] == QUEUE_TX_FLASH_START_OK ? 0xFC : 0xFE);
      HAL_UART_Transmit(&uart2, (uint8_t []){ 1, code }, 2, HAL_MAX_DELAY);
      break;
    }
    case QUEUE_TX_FLASH_CRC: {
      __HAL_RCC_CRC_CLK_ENABLE();
      // Reset with polynomial 0x04C11DB7 and initial value 0xFFFFFFFF
      CRC->CR |= CRC_CR_RESET;
      uint16_t rom_size = *STORAGE_ROM_LEN;
      uint16_t src_size = *STORAGE_SRC_LEN;
      // Hardware 4-byte word access corresponds to MSB-first calculation
      // Indeed, `CRC_Handle_8()` in `stm32g0xx_hal_crc.c` does it,
      // which is more complexity than is needed.
      for (uint16_t i = 0; i < rom_size; i++) *(uint8_t *)&CRC->DR = *(uint8_t *)(STORAGE_ROM_START + i);
      for (uint16_t i = 0; i < src_size; i++) *(uint8_t *)&CRC->DR = *(uint8_t *)(STORAGE_SRC_START + i);
      uint32_t crc = CRC->DR;
      __HAL_RCC_CRC_CLK_DISABLE();
      HAL_UART_Transmit(&uart2, (uint8_t []){
        5,
        0xCC,
        (uint8_t)((crc >> 24) & 0xff),
        (uint8_t)((crc >> 16) & 0xff),
        (uint8_t)((crc >>  8) & 0xff),
        (uint8_t)((crc >>  0) & 0xff),
      }, 6, HAL_MAX_DELAY);
    }
    default: break;
  }
}

// For Rev. 2, this is set with any well-formatted packet and cleared with a goodbyte packet (0xBB).
// For future revisions, this can be based on CH343's ACT# signal or possibly hardware flow control.
static bool rx_connected = false;

static uint16_t rx_len = 0;
static uint8_t rx_buf[256];
static uint16_t rx_ptr = 0;

static uint8_t rx_flash = 0;  // 0 - inactive, 1 - ROM, 2 - source
static uint16_t rx_rom_len, rx_src_len;
static uint16_t rx_flash_ptr;
__attribute__ ((aligned(8))) static uint8_t flash_buf[256];

#define RX_BYTE_TIMEOUT 3
static uint32_t rx_last_timestamp = (uint32_t)-RX_BYTE_TIMEOUT;

#define RX_FLASH_PACKET_TIMEOUT 100
static uint32_t rx_flash_last_timestamp = (uint32_t)-RX_FLASH_PACKET_TIMEOUT;

#pragma GCC push_options
#pragma GCC optimize("O3")
static void serial_rx_process_byte(uint8_t c)
{
  uint32_t t = HAL_GetTick();
  if (t - rx_last_timestamp >= RX_BYTE_TIMEOUT) {
    // Reset
    rx_len = 0;
  }
  rx_last_timestamp = t;

  if (rx_len == 0) {
    if (rx_flash != 0) {
      if (t - rx_flash_last_timestamp >= RX_FLASH_PACKET_TIMEOUT) {
        // Reset
        rx_flash = 0;
      } else {
        rx_flash_last_timestamp = t;
      }
    }

    // Receive payload
    rx_len = (c == 0 ? 256 : c);
    rx_ptr = 0;

  } else if (rx_flash != 0) {
    if (rx_flash == 1) {
      uint8_t p = rx_flash_ptr % 256;
      flash_buf[p] = c;
      if (++rx_flash_ptr == rx_rom_len || p == 255) {
        for (p++; p != 0; p++) flash_buf[p] = 0xff;
        uint32_t row_offs = ((rx_flash_ptr - 1) & ~0xff);
        storage_write(STORAGE_ROM_OFFS + row_offs, flash_buf);
        if (rx_flash_ptr == rx_rom_len) {
          rx_flash = 2;
          rx_flash_ptr = 0;
        }
      }
    } else {  // rx_flash == 2
      uint8_t p = rx_flash_ptr % 256;
      flash_buf[p] = c;
      if (++rx_flash_ptr == rx_src_len || p == 255) {
        for (p++; p != 0; p++) flash_buf[p] = 0xff;
        uint32_t row_offs = ((rx_flash_ptr - 1) & ~0xff);
        // XXX: Too much duplication?
        if (rx_flash_ptr == rx_src_len && STORAGE_SRC_OFFS + row_offs == (STORAGE_ROM_LEN_OFFS & ~0xff)) {
          *(uint16_t *)&flash_buf[STORAGE_ROM_LEN_OFFS % 256] = rx_rom_len;
          *(uint16_t *)&flash_buf[STORAGE_SRC_LEN_OFFS % 256] = rx_src_len;
        }
        storage_write(STORAGE_SRC_OFFS + row_offs, flash_buf);
        if (rx_flash_ptr == rx_src_len && STORAGE_SRC_OFFS + row_offs != (STORAGE_ROM_LEN_OFFS & ~0xff)) {
          // Write last row
          memset(flash_buf, 0xff, 256);
          *(uint16_t *)&flash_buf[STORAGE_ROM_LEN_OFFS % 256] = rx_rom_len;
          *(uint16_t *)&flash_buf[STORAGE_SRC_LEN_OFFS % 256] = rx_src_len;
          storage_write(STORAGE_ROM_LEN_OFFS & ~0xff, flash_buf);
        }
        if (rx_flash_ptr == rx_src_len) {
          rx_flash = 0;
          queue_tx(QUEUE_TX_FLASH_CRC);
          // Reset Mumu RAM
          memset(m.m, 0, sizeof m.m);
          m.m[MUMU_RAM_SIZE - 1] = MUMU_RAM_SIZE - 1;
          timeout_cooldown = 0;
          flash_rewritten = true; // Notify the main loop in case the VM is running
        }
      }
    }

    if (++rx_ptr == rx_len) rx_len = 0;

  } else {
    rx_buf[rx_ptr++] = c;
    if (rx_ptr == rx_len) {
      // Packet complete! Process payload
      rx_connected = true;

      if (rx_len == 1 && rx_buf[0] == 0xAA) {
        queue_tx(QUEUE_TX_HANDSHAKE);
      }

      else if (rx_len == 1 && rx_buf[0] == 0xBB) {
        rx_connected = false;
      }

      else if (rx_len == 5 && rx_buf[0] == 0xFA) {
        rx_flash = 1;
        rx_rom_len = ((uint16_t)rx_buf[1] << 8) | rx_buf[2];
        rx_src_len = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
        if (rx_rom_len == 0 || rx_rom_len > STORAGE_ROM_MAX_LEN ||
            rx_src_len == 0 || rx_src_len > STORAGE_SRC_MAX_LEN) {
          rx_flash = 0;
          queue_tx(QUEUE_TX_FLASH_START_ERR);
        } else {
          rx_flash_ptr = 0;
          rx_flash_last_timestamp = HAL_GetTick();
          queue_tx(QUEUE_TX_FLASH_START_OK);
        }
      }

      // Wait for next packet
      rx_len = 0;
    }
  }
}
#pragma GCC pop_options

void tx_readings_if_connected()
{
  if (!rx_connected) return;
  if (rx_len != 0 && rx_ptr != rx_len) {
    // XXX: Debug use
    HAL_UART_Transmit(&uart2, (uint8_t []){ 3, 0x0a, rx_len, rx_ptr }, 4, HAL_MAX_DELAY);
    if (HAL_GetTick() - rx_last_timestamp < RX_BYTE_TIMEOUT) {
      return;
    }
  }

  uint8_t readings_buf[TX_READINGS_LEN];
  fill_tx_readings(readings_buf);
  HAL_UART_Transmit(&uart2, (uint8_t []){ TX_READINGS_LEN + 1 }, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart2, (uint8_t []){ 0x56 }, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&uart2, readings_buf, TX_READINGS_LEN, HAL_MAX_DELAY);
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void USART2_IRQHandler()
{
  irqs_happened = true;
  HAL_UART_IRQHandler(&uart2);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *_uart2)
{
  serial_rx_process_byte(rx_byte);
  HAL_UART_Receive_IT(&uart2, &rx_byte, 1);
}

void TIM17_IRQHandler()
{
  if (TIM17->SR & TIM_SR_UIF) {
    TIM17->SR = ~TIM_SR_UIF;
    mumu_run_timeout = true;
  }
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
void I2C1_IRQHandler() { while (1) { } }
void I2C2_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }
void SPI2_IRQHandler() { while (1) { } }
void USART1_IRQHandler() { while (1) { } }
