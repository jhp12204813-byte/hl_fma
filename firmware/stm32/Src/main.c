#include "stm32f401xe.h"
#include <stdint.h>

#define HSI_CLOCK_HZ          16000000U
#define UART_BAUDRATE         115200U
#define PWM_PERIOD            799U       /* 16 MHz / 800 = 20 kHz */
#define DRIVE_PWM             240U       /* 30% */
#define STEERING_PWM          400U       /* 50% */
#define STEERING_LEFT_TARGET  4040U      /* measured end: 4093 */
#define STEERING_CENTER       2182U
#define STEERING_RIGHT_TARGET 50U        /* measured end: 4 */
#define STEERING_DEADBAND     50U
#define COMMAND_TIMEOUT_MS    700U
#define SPEED_REPORT_MS       200U
#define WHEEL_CIRCUM_MM_X10   8800U      /* approximately 880.0 mm */
#define COUNTS_PER_REV_X10    3712U      /* 371.2 counts/revolution */

typedef enum
{
  DRIVE_STOP = 0,
  DRIVE_FORWARD,
  DRIVE_REVERSE
} drive_state_t;

static uint16_t steering_target = STEERING_CENTER;
static drive_state_t drive_state = DRIVE_STOP;
volatile uint32_t g_millis = 0U;
static uint32_t last_drive_command_ms = 0U;
static uint32_t last_steering_command_ms = 0U;
static uint8_t steering_active = 0U;
static int32_t current_speed_mm_s = 0;
static uint8_t steering_packet_digits = 0U;
static uint16_t steering_packet_value = 0U;

static void short_delay(volatile uint32_t count)
{
  while (count-- != 0U)
  {
    __NOP();
  }
}

static void watchdog_init(void)
{
  /* About 1 second: nominal 32 kHz LSI / 32 prescaler / (999 + 1). */
  DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
  IWDG->KR = 0xCCCCU;
  IWDG->KR = 0x5555U;
  IWDG->PR = 3U;
  IWDG->RLR = 999U;
  while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U)
  {
  }
  IWDG->KR = 0xAAAAU;
}

static void watchdog_refresh(void)
{
  IWDG->KR = 0xAAAAU;
}

static void uart2_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
  (void)RCC->APB1ENR;

  /* PA2 = USART2_TX, PA3 = USART2_RX, AF7. */
  GPIOA->MODER &= ~((3U << GPIO_MODER_MODER2_Pos) |
                    (3U << GPIO_MODER_MODER3_Pos));
  GPIOA->MODER |=  (2U << GPIO_MODER_MODER2_Pos) |
                   (2U << GPIO_MODER_MODER3_Pos);
  GPIOA->AFR[0] &= ~((0xFU << GPIO_AFRL_AFSEL2_Pos) |
                     (0xFU << GPIO_AFRL_AFSEL3_Pos));
  GPIOA->AFR[0] |=  (7U << GPIO_AFRL_AFSEL2_Pos) |
                    (7U << GPIO_AFRL_AFSEL3_Pos);

  USART2->BRR = (HSI_CLOCK_HZ + (UART_BAUDRATE / 2U)) / UART_BAUDRATE;
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void uart2_putchar(char c)
{
  while ((USART2->SR & USART_SR_TXE) == 0U)
  {
  }
  USART2->DR = (uint8_t)c;
}

static void uart2_print(const char *text)
{
  while (*text != '\0')
  {
    uart2_putchar(*text++);
  }
}

static void uart2_print_int(int32_t value)
{
  char buffer[11];
  uint32_t magnitude;
  uint32_t length = 0U;

  if (value < 0)
  {
    uart2_putchar('-');
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }

  do
  {
    buffer[length++] = (char)('0' + (magnitude % 10U));
    magnitude /= 10U;
  } while (magnitude != 0U);

  while (length != 0U)
  {
    uart2_putchar(buffer[--length]);
  }
}

static int uart2_try_getchar(char *value)
{
  if ((USART2->SR & USART_SR_RXNE) == 0U)
  {
    return 0;
  }
  *value = (char)(USART2->DR & 0xFFU);
  return 1;
}

static void gpio_and_pwm_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                  RCC_AHB1ENR_GPIOBEN |
                  RCC_AHB1ENR_GPIOCEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
  (void)RCC->APB1ENR;

  /* PB3=TIM2_CH2(AF1), PB4=TIM3_CH1(AF2), PC7=TIM3_CH2(AF2). */
  GPIOB->MODER &= ~((3U << GPIO_MODER_MODER3_Pos) |
                    (3U << GPIO_MODER_MODER4_Pos) |
                    (3U << GPIO_MODER_MODER5_Pos) |
                    (3U << GPIO_MODER_MODER10_Pos));
  GPIOB->MODER |=  (2U << GPIO_MODER_MODER3_Pos) |
                   (2U << GPIO_MODER_MODER4_Pos) |
                   (1U << GPIO_MODER_MODER5_Pos) |
                   (1U << GPIO_MODER_MODER10_Pos);
  GPIOB->AFR[0] &= ~((0xFU << GPIO_AFRL_AFSEL3_Pos) |
                     (0xFU << GPIO_AFRL_AFSEL4_Pos));
  GPIOB->AFR[0] |=  (1U << GPIO_AFRL_AFSEL3_Pos) |
                    (2U << GPIO_AFRL_AFSEL4_Pos);

  GPIOC->MODER &= ~(3U << GPIO_MODER_MODER7_Pos);
  GPIOC->MODER |=  (2U << GPIO_MODER_MODER7_Pos);
  GPIOC->AFR[0] &= ~(0xFU << GPIO_AFRL_AFSEL7_Pos);
  GPIOC->AFR[0] |=  (2U << GPIO_AFRL_AFSEL7_Pos);

  /* PA8/PA9 are L298N direction outputs. Start LOW/LOW. */
  GPIOA->MODER &= ~((3U << GPIO_MODER_MODER8_Pos) |
                    (3U << GPIO_MODER_MODER9_Pos));
  GPIOA->MODER |=  (1U << GPIO_MODER_MODER8_Pos) |
                   (1U << GPIO_MODER_MODER9_Pos);
  GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BR9;
  GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR10;

  TIM2->PSC = 0U;
  TIM2->ARR = PWM_PERIOD;
  TIM2->CCR2 = 0U;
  TIM2->CCMR1 = (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM2->CCER = TIM_CCER_CC2E;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  TIM3->PSC = 0U;
  TIM3->ARR = PWM_PERIOD;
  TIM3->CCR1 = 0U;
  TIM3->CCR2 = 0U;
  TIM3->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                (6U << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
  TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void adc1_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  (void)RCC->APB2ENR;

  GPIOA->MODER |= 3U << GPIO_MODER_MODER0_Pos; /* PA0 analog */
  GPIOA->PUPDR &= ~(3U << GPIO_PUPDR_PUPD0_Pos);
  ADC1->CR1 = 0U;
  ADC1->CR2 = ADC_CR2_ADON;
  ADC1->SMPR2 = 4U << ADC_SMPR2_SMP0_Pos;
  ADC1->SQR1 = 0U;
  ADC1->SQR3 = 0U; /* first conversion: ADC1_IN0 */
}

static uint16_t steering_adc_read(void)
{
  ADC1->CR2 |= ADC_CR2_SWSTART;
  while ((ADC1->SR & ADC_SR_EOC) == 0U)
  {
  }
  return (uint16_t)ADC1->DR;
}

static void encoder_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  (void)RCC->APB1ENR;

  GPIOB->MODER &= ~((3U << GPIO_MODER_MODER6_Pos) |
                    (3U << GPIO_MODER_MODER7_Pos));
  GPIOB->MODER |=  (2U << GPIO_MODER_MODER6_Pos) |
                   (2U << GPIO_MODER_MODER7_Pos);
  GPIOB->AFR[0] &= ~((0xFU << GPIO_AFRL_AFSEL6_Pos) |
                     (0xFU << GPIO_AFRL_AFSEL7_Pos));
  GPIOB->AFR[0] |=  (2U << GPIO_AFRL_AFSEL6_Pos) |
                    (2U << GPIO_AFRL_AFSEL7_Pos);
  GPIOB->PUPDR &= ~((3U << GPIO_PUPDR_PUPD6_Pos) |
                    (3U << GPIO_PUPDR_PUPD7_Pos));
  GPIOB->PUPDR |=  (1U << GPIO_PUPDR_PUPD6_Pos) |
                   (1U << GPIO_PUPDR_PUPD7_Pos);

  TIM4->PSC = 0U;
  TIM4->ARR = 0xFFFFU;
  TIM4->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos) |
                (3U << TIM_CCMR1_IC1F_Pos) |
                (1U << TIM_CCMR1_CC2S_Pos) |
                (3U << TIM_CCMR1_IC2F_Pos);
  TIM4->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
  TIM4->SMCR = 3U << TIM_SMCR_SMS_Pos;
  TIM4->CNT = 0U;
  TIM4->CR1 = TIM_CR1_CEN;
}

static void drive_set(drive_state_t state)
{
  if (state == drive_state)
  {
    return;
  }

  /* Remove PWM before changing direction. */
  TIM2->CCR2 = 0U;
  TIM3->CCR1 = 0U;
  TIM2->EGR = TIM_EGR_UG;
  TIM3->EGR = TIM_EGR_UG;
  short_delay(80000U);

  if (state == DRIVE_FORWARD)
  {
    GPIOB->BSRR = GPIO_BSRR_BR5 | GPIO_BSRR_BR10; /* DIR LOW */
    TIM2->CCR2 = DRIVE_PWM;
    TIM3->CCR1 = DRIVE_PWM;
  }
  else if (state == DRIVE_REVERSE)
  {
    GPIOB->BSRR = GPIO_BSRR_BS5 | GPIO_BSRR_BS10; /* DIR HIGH */
    TIM2->CCR2 = DRIVE_PWM;
    TIM3->CCR1 = DRIVE_PWM;
  }
  drive_state = state;
}

static void steering_stop(void)
{
  TIM3->CCR2 = 0U;
  GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BR9;
}

static void steering_update(uint16_t position)
{
  if ((uint32_t)position + STEERING_DEADBAND < steering_target)
  {
    /* Left: PA9/IN1 LOW, PA8/IN2 HIGH. */
    GPIOA->BSRR = GPIO_BSRR_BS8 | GPIO_BSRR_BR9;
    TIM3->CCR2 = STEERING_PWM;
  }
  else if (position > (uint32_t)steering_target + STEERING_DEADBAND)
  {
    /* Right: PA9/IN1 HIGH, PA8/IN2 LOW. */
    GPIOA->BSRR = GPIO_BSRR_BR8 | GPIO_BSRR_BS9;
    TIM3->CCR2 = STEERING_PWM;
  }
  else
  {
    steering_stop();
  }
}

static void command_process(char command)
{
  if ((command >= 'a') && (command <= 'z'))
  {
    command = (char)(command - ('a' - 'A'));
  }

  switch (command)
  {
    case 'W':
      drive_set(DRIVE_FORWARD);
      last_drive_command_ms = g_millis;
      uart2_print("FORWARD\r\n");
      break;
    case 'S':
      drive_set(DRIVE_REVERSE);
      last_drive_command_ms = g_millis;
      uart2_print("REVERSE\r\n");
      break;
    case 'A':
      steering_target = STEERING_LEFT_TARGET;
      steering_active = 1U;
      last_steering_command_ms = g_millis;
      uart2_print("LEFT\r\n");
      break;
    case 'D':
      steering_target = STEERING_RIGHT_TARGET;
      steering_active = 1U;
      last_steering_command_ms = g_millis;
      uart2_print("RIGHT\r\n");
      break;
    case 'C':
      steering_target = STEERING_CENTER;
      steering_active = 1U;
      last_steering_command_ms = g_millis;
      uart2_print("CENTER\r\n");
      break;
    case 'H':
      steering_stop();
      steering_target = steering_adc_read();
      steering_active = 0U;
      uart2_print("STEERING HOLD\r\n");
      break;
    case '7':
      drive_set(DRIVE_FORWARD);
      steering_target = STEERING_LEFT_TARGET;
      steering_active = 1U;
      last_drive_command_ms = g_millis;
      last_steering_command_ms = g_millis;
      uart2_print("FORWARD LEFT\r\n");
      break;
    case '9':
      drive_set(DRIVE_FORWARD);
      steering_target = STEERING_RIGHT_TARGET;
      steering_active = 1U;
      last_drive_command_ms = g_millis;
      last_steering_command_ms = g_millis;
      uart2_print("FORWARD RIGHT\r\n");
      break;
    case '1':
      drive_set(DRIVE_REVERSE);
      steering_target = STEERING_LEFT_TARGET;
      steering_active = 1U;
      last_drive_command_ms = g_millis;
      last_steering_command_ms = g_millis;
      uart2_print("REVERSE LEFT\r\n");
      break;
    case '3':
      drive_set(DRIVE_REVERSE);
      steering_target = STEERING_RIGHT_TARGET;
      steering_active = 1U;
      last_drive_command_ms = g_millis;
      last_steering_command_ms = g_millis;
      uart2_print("REVERSE RIGHT\r\n");
      break;
    case 'X':
    case '0':
    case ' ':
      drive_set(DRIVE_STOP);
      steering_stop();
      steering_target = steering_adc_read();
      steering_active = 0U;
      uart2_print("STOP\r\n");
      break;
    case 'P':
      uart2_print("ENC=");
      uart2_print_int((int16_t)TIM4->CNT);
      uart2_print(" SPEED=");
      uart2_print_int(current_speed_mm_s);
      uart2_print("mm/s");
      uart2_print(" STEER=");
      uart2_print_int(steering_adc_read());
      uart2_print(" DRIVE=");
      uart2_print_int((int32_t)drive_state);
      uart2_print("\r\n");
      break;
    case 'Q':
      drive_set(DRIVE_STOP);
      steering_stop();
      steering_target = steering_adc_read();
      steering_active = 0U;
      uart2_print("CONTROL ENDED\r\n");
      break;
    default:
      return;
  }

}

/* Camera controller packet: 'T' followed by exactly four decimal digits. */
static void uart2_process_byte(char value)
{
  if (steering_packet_digits != 0U)
  {
    if ((value < '0') || (value > '9'))
    {
      steering_packet_digits = 0U;
      steering_packet_value = 0U;
      steering_stop();
      steering_active = 0U;
      return;
    }

    steering_packet_value = (uint16_t)((steering_packet_value * 10U) +
                                       (uint16_t)(value - '0'));
    steering_packet_digits--;

    if (steering_packet_digits == 0U)
    {
      if ((steering_packet_value >= STEERING_RIGHT_TARGET) &&
          (steering_packet_value <= STEERING_LEFT_TARGET))
      {
        steering_target = steering_packet_value;
        steering_active = 1U;
        last_steering_command_ms = g_millis;
      }
      else
      {
        steering_stop();
        steering_active = 0U;
      }
      steering_packet_value = 0U;
    }
    return;
  }

  if ((value == 'T') || (value == 't'))
  {
    steering_packet_digits = 4U;
    steering_packet_value = 0U;
    return;
  }

  command_process(value);
}

int main(void)
{
  uint32_t last_report_ms;
  uint16_t previous_encoder_count;
  char command;

  uart2_init();
  gpio_and_pwm_init();
  adc1_init();
  encoder_init();
  steering_target = steering_adc_read(); /* do not move steering at boot */
  (void)SysTick_Config(HSI_CLOCK_HZ / 1000U);
  watchdog_init();
  last_report_ms = g_millis;
  previous_encoder_count = (uint16_t)TIM4->CNT;

  uart2_print("\r\nWASD Vehicle Ready\r\n");
  uart2_print("W=forward S=reverse A=left D=right C=center X/space=stop\r\n");

  while (1)
  {
    uint16_t steering_position;

    while (uart2_try_getchar(&command) != 0)
    {
      uart2_process_byte(command);
    }

    steering_position = steering_adc_read();
    if (steering_active != 0U)
    {
      steering_update(steering_position);
    }
    else
    {
      steering_stop();
    }

    if ((drive_state != DRIVE_STOP) &&
        ((uint32_t)(g_millis - last_drive_command_ms) >= COMMAND_TIMEOUT_MS))
    {
      drive_set(DRIVE_STOP);
      uart2_print("DRIVE TIMEOUT STOP\r\n");
    }

    if ((steering_active != 0U) &&
        ((uint32_t)(g_millis - last_steering_command_ms) >= COMMAND_TIMEOUT_MS))
    {
      steering_stop();
      steering_target = steering_position;
      steering_active = 0U;
      uart2_print("STEERING TIMEOUT STOP\r\n");
    }

    if ((uint32_t)(g_millis - last_report_ms) >= SPEED_REPORT_MS)
    {
      uint32_t now_ms = g_millis;
      uint32_t elapsed_ms = (uint32_t)(now_ms - last_report_ms);
      uint16_t encoder_count = (uint16_t)TIM4->CNT;
      int16_t delta_count = (int16_t)(encoder_count - previous_encoder_count);

      current_speed_mm_s = (int32_t)(((int64_t)delta_count *
                                      WHEEL_CIRCUM_MM_X10 * 1000LL) /
                                     ((int64_t)COUNTS_PER_REV_X10 * elapsed_ms));
      previous_encoder_count = encoder_count;
      last_report_ms = now_ms;

      uart2_print("ENC=");
      uart2_print_int((int16_t)encoder_count);
      uart2_print(" SPEED=");
      uart2_print_int(current_speed_mm_s);
      uart2_print("mm/s");
      uart2_print(" STEER=");
      uart2_print_int(steering_position);
      uart2_print(" DRIVE=");
      uart2_print_int((int32_t)drive_state);
      uart2_print("\r\n");
    }

    watchdog_refresh();
  }
}
