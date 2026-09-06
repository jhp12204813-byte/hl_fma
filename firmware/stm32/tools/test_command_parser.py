#!/usr/bin/env python3
"""Compile actual firmware against fake registers; never open hardware/serial."""
from pathlib import Path
import re
import subprocess
import tempfile


HARNESS = r'''
#include <assert.h>
#include <stdio.h>
#define main firmware_main
#include "main.c"
#undef main

static void reset(void)
{
  command_fail_safe();
  rx_head = rx_tail = rx_fault = 0;
  g_millis = 100;
  steering_target = 2132;
}
static void stopped(void)
{
  assert(drive_state == DRIVE_STOP && steering_active == 0);
  assert(TIM2->CCR2 == 0 && TIM3->CCR1 == 0 && TIM3->CCR2 == 0);
  assert(parser_state == PARSER_IDLE);
}
static void feed(const char *s)
{
  while (*s) uart2_process_byte(*s++, g_millis++);
}
static void receive(char value, uint32_t status)
{
  USART2->SR = status;
  USART2->DR = (unsigned char)value;
  USART2_IRQHandler();
}
static void queued(const char *s)
{
  while (*s) { receive(*s++, USART_SR_RXNE); g_millis++; }
}
int main(void)
{
  const char *orphans[] = {"0", "1", "3", "7", "9", "2182", "2300", "0123456789"};
  for (unsigned i = 0; i < sizeof(orphans)/sizeof(orphans[0]); i++)
  {
    reset(); feed(orphans[i]); stopped();
    /* Even during existing motion, IDLE digits cannot alter the command. */
    feed("W"); feed(orphans[i]);
    assert(drive_state == DRIVE_FORWARD && !steering_active);
  }
  const char *valid[] = {"T2182", "T0150", "T3950", "T2300", "t2182"};
  const unsigned targets[] = {2182, 150, 3950, 2300, 2182};
  for (unsigned i = 0; i < 5; i++)
  {
    reset(); feed(valid[i]);
    assert(steering_active && steering_target == targets[i]);
    assert(drive_state == DRIVE_STOP && parser_state == PARSER_IDLE);
  }
  reset(); feed("T218");
  assert(!steering_active && steering_target == 2132);
  feed("2"); assert(steering_active && steering_target == 2182);
  const char *bad[] = {"T0149", "T3951", "T0050", "T4040", "T0007", "T4095", "T9999", "T0000", "T2W", "T2S", "T2?", "TT"};
  for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); i++)
  {
    reset(); feed("WT2300"); TIM3->CCR2 = STEERING_PWM;
    feed(bad[i]); stopped();
    feed("2182"); stopped();
  }
  const char *partial[] = {"T", "T2", "T21", "T218"};
  for (unsigned i = 0; i < 4; i++)
  {
    reset(); feed("W"); feed(partial[i]);
    uint32_t last = steering_packet_last_ms;
    assert(!parser_check_timeout(last + 49));
    assert(parser_check_timeout(last + 50)); stopped();
    reset(); feed("WT2300"); feed(partial[i]); feed("X"); stopped();
    reset(); feed("S"); feed(partial[i]); feed("x"); stopped();
  }
  reset(); feed("T2");
  uart2_process_byte('W', steering_packet_last_ms + 50); stopped();
  /* Timer wraps without making expired packets fresh. */
  reset(); g_millis = UINT32_MAX - 20; feed("T");
  assert(parser_check_timeout(29)); stopped();
  reset(); feed("W"); assert(drive_state == DRIVE_FORWARD);
  feed("S"); assert(drive_state == DRIVE_REVERSE);
  feed("X"); stopped();
  /* Receive a full packet before the main loop runs (e.g. during telemetry TX). */
  reset(); queued("T2182"); assert(!steering_active);
  uart2_process_rx(); assert(steering_active && steering_target == 2182);
  reset(); queued("WT2X"); uart2_process_rx(); stopped();
  /* Incomplete packet times out through the real main-loop RX service. */
  reset(); queued("WT21"); uart2_process_rx();
  g_millis += 50; uart2_process_rx(); stopped();
  /* Arrival timestamp gap is respected even when dequeued together. */
  reset(); queued("T2"); g_millis += 50; queued("182");
  uart2_process_rx(); stopped();
  reset(); queued("W"); g_millis += 50; uart2_process_rx(); stopped();
  /* Queue overflow must discard pending commands, and later recover. */
  reset(); feed("WT2182");
  for (unsigned i = 0; i < UART_RX_CAPACITY; i++) receive('W', USART_SR_RXNE);
  assert(rx_fault); uart2_process_rx(); stopped();
  assert(rx_head == rx_tail && !rx_fault);
  queued("T2300"); uart2_process_rx(); assert(steering_target == 2300 && steering_active);
  /* Every hardware receive error also drops an otherwise dangerous byte/backlog. */
  const uint32_t errors[] = {USART_SR_ORE, USART_SR_FE, USART_SR_NE, USART_SR_PE};
  for (unsigned i = 0; i < 4; i++)
  {
    reset(); feed("WT2182"); queued("S");
    receive('W', USART_SR_RXNE | errors[i]);
    receive('S', USART_SR_RXNE); assert(rx_fault);
    uart2_process_rx(); stopped(); assert(rx_head == rx_tail);
  }
  /* Exercise producer/consumer wraparound repeatedly without overflow. */
  reset();
  for (unsigned i = 0; i < 200; i++)
  {
    queued("T2182"); uart2_process_rx();
    assert(!rx_fault && steering_active && steering_target == 2182);
  }
  puts("PASS: orphan digits, valid/malformed/incomplete T, 50ms timeout, X preemption,");
  puts("      arrival timestamps, wraparound, overflow, UART errors, W/S compatibility");
  return 0;
}
'''


def main():
    source = Path(__file__).resolve().parents[1] / 'Src' / 'main.c'
    text = source.read_text()
    # Only the MCU register surface is mocked; parser/ISR/queue/STOP code is actual C.
    peripherals = sorted(set(re.findall(r'\b([A-Z][A-Z0-9]*)->', text)))
    fields = sorted(set(re.findall(r'->([A-Za-z0-9_]+)', text)))
    fields = [field for field in fields if field not in ('value', 'received_ms')]
    constants = sorted(set(re.findall(
        r'\b(?:RCC|GPIO|TIM|ADC|USART|DBGMCU|IWDG)_[A-Za-z0-9_]+\b', text)))
    header = '#include <stdint.h>\ntypedef struct {\n'
    for field in fields:
        header += f'  volatile uint32_t {field}' + ('[2]' if field == 'AFR' else '') + ';\n'
    header += '} fake_registers;\n'
    for peripheral in peripherals:
        header += f'static fake_registers fake_{peripheral};\n'
        header += f'#define {peripheral} (&fake_{peripheral})\n'
    for name in constants:
        # Distinct UART status bits are essential to test the real IRQ branches.
        value = (1 << ['USART_SR_RXNE', 'USART_SR_ORE', 'USART_SR_FE',
                       'USART_SR_NE', 'USART_SR_PE', 'USART_SR_TXE'].index(name)
                 if name.startswith('USART_SR_') else 0)
        header += f'#define {name} {value}U\n'
    header += '''
#define __NOP() ((void)0)
#define __DMB() ((void)0)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define USART2_IRQn 0
#define NVIC_SetPriority(a,b) ((void)0)
#define NVIC_ClearPendingIRQ(a) ((void)0)
#define NVIC_EnableIRQ(a) ((void)0)
#define SysTick_Config(a) 0
'''
    with tempfile.TemporaryDirectory(prefix='fma-parser-test-') as tmp:
        tmp = Path(tmp)
        (tmp / 'stm32f401xe.h').write_text(header)
        (tmp / 'test.c').write_text(HARNESS)
        binary = tmp / 'test'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                        '-I', str(tmp), '-I', str(source.parent),
                        str(tmp / 'test.c'), '-o', str(binary)], check=True, timeout=30)
        subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == '__main__':
    main()
