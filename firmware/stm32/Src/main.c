#include "stm32f401xe.h"
#include <stdint.h>

#define HSI_CLOCK_HZ          16000000U
#define UART_BAUDRATE         115200U
#define PWM_PERIOD            799U       /* 16 MHz / 800 = 20 kHz */
#define DRIVE_PWM             240U       /* 30% */
#define STEERING_PWM          520U       /* 65% */
#define STEERING_LEFT_TARGET  3950U      /* physical end: approximately 4095 */
#define STEERING_CENTER       2132U
#define STEERING_RIGHT_TARGET 150U       /* physical end: approximately 7 */
#define STEERING_DEADBAND     50U
#define STEERING_PACKET_TIMEOUT_MS 50U
#define UART_RX_CAPACITY     64U
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
typedef enum { PARSER_IDLE, PARSER_STEERING } parser_state_t;
static parser_state_t parser_state = PARSER_IDLE;
static uint32_t steering_packet_last_ms = 0U;
static uint8_t steering_packet_digits = 0U;

typedef struct
{
  char value;
  uint32_t received_ms;
} rx_byte_t;

/* ISR producer, main-loop consumer; one unused slot distinguishes full/empty. */
static volatile rx_byte_t rx_buffer[UART_RX_CAPACITY];
static volatile uint32_t rx_head = 0U;
static volatile uint32_t rx_tail = 0U;
static volatile uint8_t rx_fault = 0U;
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

/* SR then DR read clears RXNE and ORE/FE/NE/PE on STM32F401. No TX in ISR. */
void USART2_IRQHandler(void)
{
  uint32_t status = USART2->SR;
  if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE |
                 USART_SR_NE | USART_SR_PE)) != 0U)
  {
    char value = (char)(USART2->DR & 0xFFU);
    uint32_t next = (rx_head + 1U) % UART_RX_CAPACITY;
    if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U)
    {
      rx_fault = 1U;
    }
    else if (rx_fault == 0U)
    {
      if (next == rx_tail)
      {
        rx_fault = 1U;
      }
      else
      {
        rx_buffer[rx_head].value = value;
        rx_buffer[rx_head].received_ms = g_millis;
        __DMB();
        rx_head = next;
      }
    }
  }
}

static void uart2_rx_start(void)
{
  /* Enable only after PWM, ADC, timebase and watchdog are ready. */
  (void)USART2->SR;
  (void)USART2->DR;
  NVIC_SetPriority(USART2_IRQn, 1U);
  NVIC_ClearPendingIRQ(USART2_IRQn);
  USART2->CR3 |= USART_CR3_EIE;
  USART2->CR1 |= USART_CR1_RXNEIE | USART_CR1_PEIE;
  NVIC_EnableIRQ(USART2_IRQn);
}

static int uart2_try_getchar(rx_byte_t *item)
{
  /* Only the small queue operation masks interrupts, never command execution. */
  uint32_t primask = __get_PRIMASK();
  int result = 0;
  __disable_irq();
  if (rx_fault != 0U)
  {
    rx_tail = rx_head; /* discard the whole potentially corrupted backlog */
    rx_fault = 0U;
    result = -1;
  }
  else if (rx_tail != rx_head)
  {
    item->value = rx_buffer[rx_tail].value;
    item->received_ms = rx_buffer[rx_tail].received_ms;
    rx_tail = (rx_tail + 1U) % UART_RX_CAPACITY;
    result = 1;
  }
  __set_PRIMASK(primask);
  return result;
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

static void parser_reset(void)
{
  parser_state = PARSER_IDLE;
  steering_packet_digits = 0U;
  steering_packet_value = 0U;
}

static void command_fail_safe(void)
{
  /* No ADC conversion, direction-change delay, or blocking ACK on STOP. */
  TIM2->CCR2 = 0U;
  TIM3->CCR1 = 0U;
  steering_stop();
  TIM2->EGR = TIM_EGR_UG;
  TIM3->EGR = TIM_EGR_UG;
  drive_state = DRIVE_STOP;
  steering_active = 0U;
  parser_reset();
}

static int parser_check_timeout(uint32_t now_ms)
{
  if ((parser_state == PARSER_STEERING) &&
      ((uint32_t)(now_ms - steering_packet_last_ms) >= STEERING_PACKET_TIMEOUT_MS))
  {
    command_fail_safe();
    return 1;
  }
  return 0;
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
      break;
    case 'S':
      drive_set(DRIVE_REVERSE);
      last_drive_command_ms = g_millis;
      break;
    case 'A':
      steering_target = STEERING_LEFT_TARGET;
      steering_active = 1U;
      last_steering_command_ms = g_millis;
      break;
    case 'D':
      steering_target = STEERING_RIGHT_TARGET;
      steering_active = 1U;
      last_steering_command_ms = g_millis;
      break;
    case 'C':
      steering_target = STEERING_CENTER;
      steering_active = 1U;
      last_steering_command_ms = g_millis;
      break;
    case 'H':
      steering_stop();
      steering_target = steering_adc_read();
      steering_active = 0U;
      break;
    case 'X':
    case ' ':
      command_fail_safe();
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
      command_fail_safe();
      break;
    default:
      return;
  }

}

/* Only this state consumes digits. IDLE digits can never initiate motion. */
static void uart2_process_byte(char value, uint32_t received_ms)
{
  if ((value == 'X') || (value == 'x'))
  {
    command_fail_safe(); /* preempts a partial T packet, even at timeout */
    return;
  }
  if (parser_check_timeout(received_ms) != 0)
  {
    return; /* discard the byte following an expired partial packet */
  }
  if (parser_state == PARSER_STEERING)
  {
    if ((value < '0') || (value > '9'))
    {
      command_fail_safe(); /* never reinterpret the offending byte as W/S */
      return;
    }
    steering_packet_last_ms = received_ms;
    steering_packet_value = (uint16_t)(steering_packet_value * 10U + (value - '0'));
    steering_packet_digits++;
    if (steering_packet_digits == 4U)
    {
      if ((steering_packet_value >= STEERING_RIGHT_TARGET) &&
          (steering_packet_value <= STEERING_LEFT_TARGET))
      {
        steering_target = steering_packet_value;
        steering_active = 1U;
        last_steering_command_ms = received_ms;
        parser_reset();
      }
      else
      {
        command_fail_safe();
      }
    }
    return;
  }
  if ((value >= '0') && (value <= '9'))
  {
    return;
  }
  if ((value == 'T') || (value == 't'))
  {
    parser_state = PARSER_STEERING;
    steering_packet_digits = 0U;
    steering_packet_value = 0U;
    steering_packet_last_ms = received_ms;
    /* Do not keep applying an old steering target during incomplete input. */
    steering_stop();
    steering_active = 0U;
    return;
  }
  command_process(value);
}

static void uart2_process_rx(void)
{
  rx_byte_t item;
  int result;
  /* Bounded work: UART flooding cannot starve watchdog or motion timeouts. */
  for (uint32_t count = 0U; count < UART_RX_CAPACITY; count++)
  {
    result = uart2_try_getchar(&item);
    if (result < 0)
    {
      command_fail_safe();
      return;
    }
    if (result == 0)
    {
      /* Only use wall time after draining arrival-timestamped bytes. */
      (void)parser_check_timeout(g_millis);
      return;
    }
    if ((uint32_t)(g_millis - item.received_ms) >= STEERING_PACKET_TIMEOUT_MS)
    {
      command_fail_safe(); /* do not execute a stale queued motion command */
      continue;
    }
    uart2_process_byte(item.value, item.received_ms);
  }
}

int main(void)
{
  uint32_t last_report_ms;
  uint16_t previous_encoder_count;

  uart2_init();
  gpio_and_pwm_init();
  adc1_init();
  encoder_init();
  steering_target = steering_adc_read(); /* do not move steering at boot */
  (void)SysTick_Config(HSI_CLOCK_HZ / 1000U);
  watchdog_init();
  uart2_rx_start();
  last_report_ms = g_millis;
  previous_encoder_count = (uint16_t)TIM4->CNT;

  uart2_print("\r\nWASD Vehicle Ready\r\n");
  uart2_print("W=forward S=reverse A=left D=right C=center X/space=stop\r\n");

  while (1)
  {
    uint16_t steering_position;

    uart2_process_rx();

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
