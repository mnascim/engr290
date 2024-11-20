#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

// Constant for Timer1 50 Hz PWM (ICR mode)
#define PWM_TOP 2500

void gpio_init();
void uart_tx_init();
void uart_init();
void timer1_50Hz_init (uint8_t en_IRQ);
void timer0_init ();
void adc_init (uint8_t channel, uint8_t en_IRQ);
void twi_init();
