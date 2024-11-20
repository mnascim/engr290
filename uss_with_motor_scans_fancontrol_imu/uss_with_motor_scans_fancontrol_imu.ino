#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>

//FRONT USS P13 ON BOARD
#define FRONT_TRIG_PIN PB5  // Trigger pin on PORTB; OC2A
#define FRONT_ECHO_PIN PD3  // Echo pin on PORTB; INT0

//TOP USS P6 ON BOARD
#define TOP_TRIG_PIN PB3
#define TOP_ECHO_PIN PD2

#define propulsion_fan_direction OCR1A //easier to read in code
#define levitation_fan_pwm OCR1B //easier to read in code
#define propulsion_fan_pwm OCR0A //easier to read in code

//SERVO IS P9 ON BOARD (PWM0)

//LEVITATION FAN IS P4 ON BOARD (PWM1)

//PROPULSION FAN IS P3 ON BOARD (PWM2)

//IMU IS P7 ON BOARD - PC5 -> SCL, PC4 -> SDA

//VARIABLES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static volatile long front_pulse, top_pulse;
static volatile long front_distance, top_distance, left_distance, right_distance;

//UART~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

//USS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void USS_INIT(){
  //front USS
  DDRB |= (1 << FRONT_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << FRONT_ECHO_PIN); // Set ECHO_PIN as input

  //top USS
  DDRB |= (1 << TOP_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << TOP_ECHO_PIN); // Set ECHO_PIN as input 
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

int scan_front_uss(void){
  _delay_ms(5);
  //front ultrasonic sensor
  trigger_sensor(FRONT_TRIG_PIN); //sends pulse to the trigger
  front_pulse = pulse_length(FRONT_ECHO_PIN); //see how long the pulse takes to get to the echo
  int distance = getDistance(front_pulse); //calculate the distance

  return distance;
}

int scan_top_uss(void){
  _delay_ms(5);
  //top ultrasonic sensor
  trigger_sensor(TOP_TRIG_PIN); //sends pulse to the trigger
  top_pulse = pulse_length(TOP_ECHO_PIN); //see how long the pulse takes to get to the echo
  int distance = getDistance(top_pulse); //calculate the distance

  return distance;
}

//MOTOR~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void set_servo_position(int yaw){
    propulsion_fan_direction = Servo_angle[yaw];
}

//MAIN CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(void)
{
    gpio_init();
    USS_INIT(); //Initialize US sensor
    uart_init(9600); //Initialize Uart
    timer1_50Hz_init(0); //in init_290.c, initialize PWM0 for servo and PWM1 for leviation fan
    timer0_init(); //PWM2 for propulsion fan
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "BOTH FANS ON\n\r");
    send_string(buffer);

    levitation_fan_pwm = 255; //start fan at max rpm
    propulsion_fan_pwm = 255; //start fan at max rpm
    
    while (1) //constantly loops while the power is on
    {
      
      front_distance = scan_front_uss(); //calculate the distance
      top_distance = scan_top_uss();
      

      if(top_distance <= 40){ //first check if the top is detected since most important
        _delay_ms(500); // delete after this is just to read
        snprintf(buffer, sizeof(buffer), "TOP BAR DETECTED, STOP ALL FANS\n\r");
        send_string(buffer);

        levitation_fan_pwm = 0; //stop fan
        propulsion_fan_pwm = 0; //stop fan

        return 0;
      }
      else if(front_distance <= 10){ //second check the front if its in front of a wall
        _delay_ms(500); // delete after this is just to read
        snprintf(buffer, sizeof(buffer), "WALL DETECTED, STOP PROPULSION\n\r");
        send_string(buffer);

        propulsion_fan_pwm = 0; //stop fan

        //scan left side
        _delay_ms(500); // delete after this is just to read
        set_servo_position(1); //turn servo all the way left
        left_distance = scan_front_uss();
        snprintf(buffer, sizeof(buffer), "read and save value for left side\n\r");
        send_string(buffer);

        //scan right side
        _delay_ms(500); // delete after this is just to read
        set_servo_position(254); //turn servo all the way left
        right_distance = scan_front_uss();
        snprintf(buffer, sizeof(buffer), "read and save value for right side\n\r");
        send_string(buffer);

        //comparing left and right wall distances
        if(left_distance < right_distance){ //wall on left
          _delay_ms(500);// delete after this is just to read
          snprintf(buffer, sizeof(buffer), "left is closer than right, turn right\n\r");
          send_string(buffer);
        }
        else if(right_distance < left_distance){  //wall on right
          _delay_ms(500);// delete after this is just to read
          snprintf(buffer, sizeof(buffer), "right is closer than left, turn left\n\r");
          send_string(buffer);
        }
        //if they are equal it will just repeat the loop, detect the wall in front again, then check again
        
        _delay_ms(500); // delete after this is just to read
        set_servo_position(127); //return to middle
        _delay_ms(1000);//pause for sensor to get data?? dont know if we need yet
        snprintf(buffer, sizeof(buffer), "PROPULSION FAN BACK ON\n\r");
        send_string(buffer);

        propulsion_fan_pwm = 255; //start fan at max rpm
        
        _delay_ms(500); // delete after this is just to read
      }

      /*
      //THESE TWO PRINTS CAN BE DELETED FOR THE FINAL THING. JUST USED TO SEE WHAT HAPPENS WHEN TESTING~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //sending data to terminal
      snprintf(buffer, sizeof(buffer), "Front pulse length: %ld, Front distance: %ld cm\n\r", front_pulse, front_distance);
      send_string(buffer);

      //sending data to terminal
      snprintf(buffer, sizeof(buffer), "Top pulse length: %ld, Top distance: %ld cm\n\r", top_pulse, top_distance);
      send_string(buffer);
      */
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }
    
    return 0; //end
}
