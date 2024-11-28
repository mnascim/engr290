#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>
#include "TWI_290.c"

//TOP USS P13 ON BOARD
#define TOP_TRIG_PIN PB5  // Trigger pin on PORTB; OC2A
#define TOP_ECHO_PIN PD3  // Echo pin on PORTB; INT0

//FRONT USS P6 ON BOARD
#define FRONT_TRIG_PIN PB3
#define FRONT_ECHO_PIN PD2

#define propulsion_fan_direction OCR1A //easier to read in code
#define levitation_fan_pwm OCR0A //easier to read in code
#define propulsion_fan_pwm OCR0B //easier to read in code

//SERVO IS P9 ON BOARD (PWM0)

//LEVITATION FAN IS P4 ON BOARD (PWM1)

//PROPULSION FAN IS P3 ON BOARD (PWM2)

//IMU IS P7 ON BOARD - PC5 -> SCL, PC4 -> SDA
//defining registers address
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define I2C_ADDRESS 0x68
 
//defininf gyro output registers for read
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
 
// defining accel output registers for read
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

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
/*
void set_servo_position(int yaw){
    propulsion_fan_direction = Servo_angle[yaw];
}
*/
//IMU~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void TWI_init (uint8_t ){
    TWSR=0; // no prescaler
    TWBR=(uint8_t)(((F_CPU/SCL_CLOCK)-16)>>1);
 
}

//vars to hold values on each axis
int16_t x_gyro, y_gyro, z_gyro;
int16_t x_accel, y_accel, z_accel;

void Read_Gyro_Data() {
    uint16_t x_gyro_data[2], y_gyro_data[2], z_gyro_data[2];
   
    // Read X-axis
    _delay_ms(2);
    Read_Reg_N(I2C_ADDRESS, GYRO_XOUT_H, 2, x_gyro_data);
    x_gyro = (x_gyro_data[0] << 8) | x_gyro_data[1];
   
    // Read Y-axis
    _delay_ms(2);
    Read_Reg_N(I2C_ADDRESS, GYRO_YOUT_H, 2, y_gyro_data);
    y_gyro = (y_gyro_data[0] << 8) | y_gyro_data[1];
   
    // Read Z-axis
    _delay_ms(2);
    Read_Reg_N(I2C_ADDRESS, GYRO_ZOUT_H, 2, z_gyro_data);
    z_gyro = (z_gyro_data[0] << 8) | z_gyro_data[1];
   
    Serial.println("z_gyro: ");
    Serial.println(z_gyro);
}

//Reading Accelerometer Data held in the accel config register
void Read_Accel_Data(){
    //read x-axis
    int16_t x_accel_data[2]; //array holding low and high bytes of gyro register
    Read_Reg_N(ACCEL_CONFIG, ACCEL_XOUT_H,2, x_accel_data);//read starting from H and store to array
    x_accel= (x_accel_data[0]<<8) | (x_accel_data[1] &0xFF); // shift by 8 bits x_gyro_data (1byte) -> MSB and combine with the other byte
   
    //y
    int16_t y_accel_data[2];
    Read_Reg_N(ACCEL_CONFIG,ACCEL_YOUT_H,2, y_accel_data);
    y_accel= (y_accel_data[0]<<8) | (y_accel_data[1] &0xFF);
   
 
    //z
    int16_t z_accel_data[2];
    Read_Reg_N(ACCEL_CONFIG,ACCEL_ZOUT_H,2, z_accel_data);
    z_accel= (z_accel_data[0]<<8) | (z_accel_data[1] &0xFF);
   
    //Note: &FF to ensure it's reading 8 bits, avoid sign extensions
}

float yaw = 90.0;
float Calculate_Yaw() {
    float z_gyro_rate = (z_gyro / 77.2); // Assuming ±500 deg/s scale
 
    if ((z_gyro_rate < 0.75) && (z_gyro_rate > -0.75)) {
        z_gyro_rate = 0;
    }
 
    yaw += (z_gyro_rate * 0.02); // Integrate over 20ms (50Hz)
   
    // Normalize yaw to 0-360 range
    while (yaw < 0) yaw += 360;
    while (yaw >= 360) yaw -= 360;
 
    if (( yaw <= 360) && (yaw >= 270)){yaw = 0;}
    if (( yaw < 270) && (yaw >= 180)){yaw = 180;}
    
    return yaw;
}

//set servo position
void set_servo_position(float yaw){
    //map yaw to values in range[0,255]
    int position = yaw/180 *255;
   
    //keep value within range
    if(position<0)
        position=0;
    if (position>= 256)
        position=255;
   
    int temp = 255 - position;
    //set PWM to value
    OCR1A = Servo_angle[position];
}

//MAIN CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(void)
{
    //initializing
    gpio_init();
    USS_INIT(); //Initialize US sensor
    uart_init(9600); //Initialize Uart
    TWI_init(); //initialize TWI for the IMU
    timer1_50Hz_init(0); //in init_290.c, initialize PWM0 for servo and PWM1 for leviation fan
    timer0_init(); //PWM2 for propulsion fan



    // Initialize IMU
    Write_Reg(I2C_ADDRESS, 0x6B, 0x00);
    Write_Reg(I2C_ADDRESS, ACCEL_CONFIG, 0x00); // ±2g range
    Write_Reg(I2C_ADDRESS, GYRO_CONFIG, 0x00);  // ±500 deg/s range


    //calculate initial value for z axis
    uint8_t angle;
    int32_t z_gyro_offset = 0;
    for (int m = 0; m < 100; m++) {
        Read_Gyro_Data();
        z_gyro_offset += (z_gyro);
        
    }
    z_gyro_offset /= 100;

    //just used to troubleshoot
    //char buffer[128];
    // snprintf(buffer, sizeof(buffer), "poop\n\r");
    //    send_string(buffer);

    //turn on both fans
    levitation_fan_pwm = 255; //start fan at max rpm
    propulsion_fan_pwm = 191; //start fan at max rpm
    
    while (1) //constantly loops while the power is on
    {
      // read IMU
      _delay_ms(20);
      Read_Gyro_Data();
      //Read_Accel_Data();
      z_gyro -= z_gyro_offset;

      float calculated_yaw = Calculate_Yaw();
     
      if (calculated_yaw < 5){
        angle = 5;
      }
      else if (calculated_yaw > 175){
        angle = 175;
      }
      else {
        angle = calculated_yaw;
      }

      set_servo_position(angle); //adjust motor to where it should be

      // read USS value
      front_distance = scan_front_uss(); //calculate the distance
      top_distance = scan_top_uss();
      
      if(top_distance <= 5){ //first check if the top is detected since most important
        /*
        _delay_ms(500); // delete after this is just to read
        snprintf(buffer, sizeof(buffer), "TOP BAR DETECTED, STOP ALL FANS\n\r");
        send_string(buffer);
        */
        levitation_fan_pwm = 0; //stop fan
        propulsion_fan_pwm = 0; //stop fan

        return 0;
      }
      else if(front_distance <= 10){ //second check the front if its in front of a wall
        /*
        _delay_ms(500); // delete after this is just to read
        snprintf(buffer, sizeof(buffer), "WALL DETECTED, STOP PROPULSION\n\r");
        send_string(buffer);
        */
        propulsion_fan_pwm = 0; //stop fan

        //scan left side
        //_delay_ms(500); // delete after this is just to read
        set_servo_position(1); //turn servo all the way left
        left_distance = scan_front_uss();
        /*
        snprintf(buffer, sizeof(buffer), "read and save value for left side\n\r");
        send_string(buffer);
        */
        //scan right side
        //_delay_ms(500); // delete after this is just to read
        set_servo_position(254); //turn servo all the way left
        right_distance = scan_front_uss();
        /*
        snprintf(buffer, sizeof(buffer), "read and save value for right side\n\r");
        send_string(buffer);
        */
        //comparing left and right wall distances
        if(left_distance < right_distance){ //wall on left
          //_delay_ms(500);// delete after this is just to read
          //snprintf(buffer, sizeof(buffer), "left is closer than right, turn right\n\r");
          //send_string(buffer);
        }
        else if(right_distance < left_distance){  //wall on right
          //_delay_ms(500);// delete after this is just to read
          //snprintf(buffer, sizeof(buffer), "right is closer than left, turn left\n\r");
          //send_string(buffer);
        }
        //if they are equal it will just repeat the loop, detect the wall in front again, then check again
        
        set_servo_position(127); //return to middle
        
        propulsion_fan_pwm = 191; //start fan at max rpm
        
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
