#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>

#define TRIG_PIN PB5  // Trigger pin on PORTB; OC2A
#define ECHO_PIN PD3  // Echo pin on PORTB; INT0

//VARIABLES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static volatile long pulse;
static volatile long distance;


//FUNCTIONS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void USS_INIT(){
	DDRB |= (1 << TRIG_PIN); // Set TRIG_PIN as output
	DDRD &= ~(1 << ECHO_PIN); // Set ECHO_PIN as input (corrected)
}

//UART start transmission function
void send_reading(char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait until the buffer is empty
	UDR0 = data; // Send received char to UDR0 (Data register)
}

// send string to transmission function
void send_string(const char* str) { // send string to send function
	while (*str) { // while string not empty
		send_reading(*str++); //send string to send_Readin
	}
}

void trigger_sensor(void){
	PORTB &= ~(1 << TRIG_PIN);
	_delay_us(2);
 
	// Send a 10µs pulse to the trigger pin
  PORTB |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTB &= ~(1 << TRIG_PIN);
}

long pulse_length()
{
	long duration = 0;
	
	while (!(PIND & (1 << ECHO_PIN)));

	while (PIND & (1 << ECHO_PIN)) {
		duration++;
		_delay_us(0.05);
	}
	
	return duration;
}

uint32_t getDistance(long pulse){
  return ((pulse * 0.0343) / 2);
}


//MAIN CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(void)
{
    USS_INIT(); //Initialize US sensor
    uart_init(); //Initialize Uart
    
    char buffer[128];
    
    while (1) //constantly loops while the power is on
    {
		  _delay_ms(100);

      //ultrasonic sensor
		  trigger_sensor(); //sends pulse to the trigger
		  pulse = pulse_length(); //see how long the pulse takes to get to the echo
		  distance = getDistance(pulse); //calculate the distance

      //sending data to terminal
		  char buffer[64];
		  snprintf(buffer, sizeof(buffer), "Pulse length: %ld, Distance: %ld cm\n\r", pulse, distance);
		  send_string(buffer);
    }
    
    return 0; //end
}
