#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>

//P13 ON BOARD
#define FRONT_TRIG_PIN PB5  // Trigger pin on PORTB; OC2A
#define FRONT_ECHO_PIN PD3  // Echo pin on PORTB; INT0

//P6 ON BOARD
#define TOP_TRIG_PIN PB3
#define TOP_ECHO_PIN PD2

//VARIABLES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static volatile long front_pulse, top_pulse;
static volatile long front_distance, top_distance;


//FUNCTIONS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void USS_INIT(){
  //front USS
  DDRB |= (1 << FRONT_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << FRONT_ECHO_PIN); // Set ECHO_PIN as input

  //top USS
  DDRB |= (1 << TOP_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << TOP_ECHO_PIN); // Set ECHO_PIN as input 
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

void trigger_sensor(int TRIG_PIN){ //TRIG_PIN is a variable in this case to use the same function twice
  PORTB &= ~(1 << TRIG_PIN);
  _delay_us(2);
 
  // Send a 10µs pulse to the trigger pin
  PORTB |= (1 << TRIG_PIN);
  _delay_us(10);
  PORTB &= ~(1 << TRIG_PIN);
}

long pulse_length(int ECHO_PIN) //ECHO_PIN is a variable in this case to use the same function twice
{
  long duration = 0;

  while (!(PIND & (1 << ECHO_PIN)));

  while (PIND & (1 << ECHO_PIN)) {
    duration++;
    _delay_us(0.05);

    if(duration == 40000){ //overflow at  686 cm idk you can change this, stops unnecessary waiting
      return duration;
    }
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
    snprintf(buffer, sizeof(buffer), "BOTH FANS ON\n\r");
    send_string(buffer);
    
    while (1) //constantly loops while the power is on
    {
      _delay_ms(5);
      //front ultrasonic sensor
      trigger_sensor(FRONT_TRIG_PIN); //sends pulse to the trigger
      front_pulse = pulse_length(FRONT_ECHO_PIN); //see how long the pulse takes to get to the echo
      front_distance = getDistance(front_pulse); //calculate the distance

      _delay_ms(5);
      //top ultrasonic sensor
      trigger_sensor(TOP_TRIG_PIN); //sends pulse to the trigger
      top_pulse = pulse_length(TOP_ECHO_PIN); //see how long the pulse takes to get to the echo
      top_distance = getDistance(top_pulse); //calculate the distance

      if(top_distance <= 40){ //first check if the top is detected since most important
        _delay_ms(500);
        snprintf(buffer, sizeof(buffer), "TOP BAR DETECTED, STOP ALL FANS\n\r");
        send_string(buffer);

        return 0;
      }
      else if(front_distance <= 10){ //second check the front if its in front of a wall
        _delay_ms(500);
        snprintf(buffer, sizeof(buffer), "WALL DETECTED, STOP PROPULSION\n\rturn motor left 90 deg\n\r");
        send_string(buffer);
        //read and save value for left motor
        snprintf(buffer, sizeof(buffer), "turn motor right 180 deg\n\r");
        send_string(buffer);
        //read and save value for right motor

        _delay_ms(500);
        //if statement to compare which is best
        snprintf(buffer, sizeof(buffer), "PROPULSION FAN BACK ON\n\r");
        send_string(buffer);
        _delay_ms(500);
      }


      //THESE TWO PRINTS CAN BE DELETED FOR THE FINAL THING. JUST USED TO SEE WHAT HAPPENS WHEN TESTING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //sending data to terminal
      snprintf(buffer, sizeof(buffer), "Front pulse length: %ld, Front distance: %ld cm\n\r", front_pulse, front_distance);
      send_string(buffer);

      //sending data to terminal
      snprintf(buffer, sizeof(buffer), "Top pulse length: %ld, Top distance: %ld cm\n\r", top_pulse, top_distance);
      send_string(buffer);
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    
    return 0; //end
}
