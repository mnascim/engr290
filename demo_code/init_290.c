// For H/W version 3.10
// July 2022

#include "init_290.h"
#include <avr/interrupt.h>
#include <stdlib.h>

// Constant for Timer1 50 Hz PWM (ICR mode)
#define PWM_TOP 4999

// Converter of [0;255] range of x to [0,PWM_top] range for OCR of Timer1 (16 bit)
#define D1B(x) (uint16_t)(((x)*(uint32_t)(PWM_top))>>8)


// #define BAUD 57600UL
#ifndef F_CPU
#define F_CPU 16000000
#endif //F_CPU

#define BAUD 9600UL
#define UBRR ((F_CPU)/((BAUD)*(16UL))-1)

#define SCL_CLOCK 100000L

const uint16_t Servo_angle [256]={ //  +/-90 degrees
85,85,85,85,85,85,85,85,85,85,85,85,85,85,85,85, // 0...15
108,108,108,108,108,108,108,108,108,108,108,108,108,108,108,108, // 16...31 0.9ms pulse(108) 
119,119,119,119,119,119,119,119,119,119,119,119,119,119,119,119, // 32...47
131,131,131,131,131,131,131,131,131,131,131,131,131,131,131,131, // 48...63
142,142,142,142,142,142,142,142,142,142,142,142,142,142,142,142, // 64...79
154,154,154,154,154,154,154,154,154,154,154,154,154,154,154,154, // 80...95
165,165,165,165,165,165,165,165,165,165,165,165,165,165,165,165, // 96...111
177,177,177,177,177,177,177,177,177,177,177,177,188,188,188,188, // center - 1.5ms pulse (188) 112...127
188,188,188,188,198,198,198,198,198,198,198,198,198,198,198,198, // 128...143
208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208, // 143...159
218,218,218,218,218,218,218,218,218,218,218,218,218,218,218,218, // 160...175
228,228,228,228,228,228,228,228,228,228,228,228,228,228,228,228, // 175...191
238,238,238,238,238,238,238,238,238,238,238,238,238,238,238,238, // 192...207
248,248,248,248,248,248,248,248,248,248,248,248,248,248,248,248, // 208...223
270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270, // 224...239
290,290,290,290,290,290,290,290,290,290,290,290,290,290,290,290  // 240...255 
};

void gpio_init() {
  cli();
  DDRB = ((1 << PB2) | (1 << PB1)); // PB3-PB1 are set as outputs for PWM
  PORTB=((1<<PB5)|(1<<PB4)|(1<<PB3)|(1<<PB0)); //enablet pull-up resistors on PB5, PB4 and PB0; PB3-HI (D3 OFF), PB2 and PB1 are set to LOW
    
  DDRC=0; // all pins are inputs. Not really needed as 0 is default value. 
  PORTC=((1<<PC4)|(1<<PC4)); // temporary pulling up TWI pins. The pins' function will be overriden once TWI is enabled.
  DIDR0=((1<<PC3)|(1<<PC2)|(1<<PC1)|(1<<PC0)); // disable digital inputs on analog pins.
  
  DDRD=(1<<PD7)| (1<<PD2) /*Echo pin output*/|(1<<PD6)|(1<<PD5)|(1<<PD4)|(1<<PD1); //Set PD1 (TX) and PD7-PD4 (power control) as outputs.
  PORTD=((1<<PD1)|(1<<PD3)); // Set PD1(TX) to HIGH (idle) and enable pull-ups on PD3 and PD2.
  sei(); 
} //end gpio_init


void uart_tx_init() {
  UBRR0H = (uint8_t)((UBRR)>>8); // Set the UART speed as defined by UBRR
  UBRR0L = (uint8_t)UBRR;
  UCSR0B|=(1<<TXCIE0)|(1<<TXEN0); //(1<<UDRIE0) Enable TX and TX IRQ.
  UCSR0C=(3<<UCSZ00); // Asynchronous UART, 8-N-1
}// end UART init

void uart_init(unsigned int baud) { // TX and RX init with IRQ
  unsigned int ubrr = F_CPU / 16 / baud - 1;
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable receiver and transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}// end UART init





// ======= PWM0 and PWM1 control (16-bit timer1) ===================
void timer1_50Hz_init(uint8_t en_IRQ) { //en_IRQ eanbles 
  TCCR1A|=(1<<COM1A1)|(1<<COM1B1); // non-inv PWM on channels A and B
  TCCR1B|=(1<<WGM13);  //PWM, Phase and Frequency Correct. TOP=ICR1.
  ICR1=PWM_TOP; //50Hz PWM
  OCR1A=Servo_angle[127]; 
  OCR1B=0; 
  TCCR1B|=((1<<CS11)|(1<<CS10)); //timer prescaler
  if (en_IRQ) TIMSK1|=(1<<ICIE1); // enable Input Capture Interrupt. NOTE: the ISR MUST be defined!!! 
}
 

void timer0_init () { 
// ======= PWM2 and D4 control (8-bit timer0) ===================
   // Set the waveform generation mode to Fast PWM (Mode 3)
    TCCR0A |= (1 << WGM00) | (1 << WGM01); // WGM02 is in TCCR0B
    TCCR0B &= ~(1 << WGM02);

    // Set the Compare Output Mode to non-inverting on OC0B (Pin D5)
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
    TCCR0A &= ~(1 << COM0B0);

    // Set the prescaler to 64 (to control PWM frequency)
    TCCR0B |= (1 << CS01) | (1 << CS00);
    TCCR0B &= ~(1 << CS02);
               
  OCR0A=0; 
  OCR0B=0; 
  TCCR0B|=((1<<CS01)|(1<<CS00)); 
}


void adc_init (uint8_t channel, uint8_t en_IRQ) {
// ADC init
  ADMUX = ((1<<ADLAR)|(channel&0x0F)); // "left-aligned" result for easy 8-bit reading. 
  ADMUX |= (1<<REFS0);				// AVcc as Aref |(1<<REFS0)
  					// Sets ADC to the specified channel. Can be changed later.
  ADCSRA=(1<<ADEN); 
  if (en_IRQ) ADCSRA|=(1<<ADIE); // enable ADC Complete Interrupt. NOTE: the ISR MUST be defined!!!
  ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); // ADC clock prescaler
  ADCSRA|=(1<<ADATE); // Continuosly running mode
  ADCSRA|=(1<<ADSC); // Start ADC
}

void twi_init(){
// TWI init
TWSR=0; // no prescaler
TWBR=(uint8_t)(((F_CPU/SCL_CLOCK)-16)>>1);  //setting SCL; must be >10 for stable operation
}
