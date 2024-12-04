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

//Definitions to make code reading easier later
#define propulsion_fan_direction OCR1A
#define levitation_fan_pwm OCR0A 
#define propulsion_fan_pwm OCR0B 

//Direction definitions to make it easy to read in code later
#define LEFT 1
#define RIGHT 254
#define CENTER 127

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

//VARIABLES~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static volatile long front_pulse, top_pulse;
static volatile long front_distance, top_distance, left_distance, right_distance;

//USS~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void USS_INIT() {
  //front USS
  DDRB |= (1 << FRONT_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << FRONT_ECHO_PIN); // Set ECHO_PIN as input

  //top USS
  DDRB |= (1 << TOP_TRIG_PIN); // Set TRIG_PIN as output
  DDRD &= ~(1 << TOP_ECHO_PIN); // Set ECHO_PIN as input
}

void trigger_sensor(int TRIG_PIN) { //TRIG_PIN is a variable in this case to use the same function twice
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

  while (!(PIND & (1 << ECHO_PIN))); //lock here until its ready

  while (PIND & (1 << ECHO_PIN)) {
    duration++;
    _delay_us(0.05);

    if (duration == 40000) { //overflow to stop unnecessary waiting
      return duration;
    }
  }
  return duration; //give time of pulse
}

uint32_t getDistance(long pulse) {
  return ((pulse * 0.0343) / 2); //calculate cm distance with given formula
}

int scan_front_uss(void) {
  _delay_ms(5);
  trigger_sensor(FRONT_TRIG_PIN); //sends pulse to the trigger
  front_pulse = pulse_length(FRONT_ECHO_PIN); //see how long the pulse takes to get to the echo
  int distance = getDistance(front_pulse); //calculate the distance

  return distance;
}

int scan_top_uss(void) {
  _delay_ms(5);
  trigger_sensor(TOP_TRIG_PIN); //sends pulse to the trigger
  top_pulse = pulse_length(TOP_ECHO_PIN); //see how long the pulse takes to get to the echo
  int distance = getDistance(top_pulse); //calculate the distance

  return distance;
}

//IMU~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void MPU6050_init(void) {
  TWI_status = TWI_start(I2C_ADDRESS, TW_WRITE);
  if (TWI_status) return;

  TWI_status = TWI_write(0x6B);
  if (TWI_status) return;

  TWI_status = TWI_write(0x00);
  if (TWI_status) return;

  TWI_stop();
}

//Variables to hold values on each axis
int16_t x_gyro, y_gyro, z_gyro;
int16_t x_accel, y_accel, z_accel;

void Read_Gyro_Data() { //we read all of them but only use Z
  uint16_t x_gyro_data[2], y_gyro_data[2], z_gyro_data[2];

  // Read X-axis
  Read_Reg_N(I2C_ADDRESS, GYRO_XOUT_H, 2, x_gyro_data);
  x_gyro = (x_gyro_data[0] << 8) | x_gyro_data[1];

  // Read Y-axis
  Read_Reg_N(I2C_ADDRESS, GYRO_YOUT_H, 2, y_gyro_data);
  y_gyro = (y_gyro_data[0] << 8) | y_gyro_data[1];

  // Read Z-axis
  Read_Reg_N(I2C_ADDRESS, GYRO_ZOUT_H, 2, z_gyro_data);
  z_gyro = (z_gyro_data[0] << 8) | z_gyro_data[1];
}

float yaw = 0;
float prev_yaw = 0;
bool right, left;
int32_t z_gyro_offset = 0;

float Calculate_Yaw() {
  //Compensate for gyroscope offset
  float z_gyro_rate = (z_gyro - z_gyro_offset) / 65.5; // Convert to deg/s
  
  //Apply a dead zone to filter out noise
  if (fabs(z_gyro_rate) < 1.2) { 
    z_gyro_rate = 0;
  }
  
  yaw = prev_yaw + (z_gyro_rate * 0.02); //Integrate gyroscope rate to calculate yaw over 20 ms
  prev_yaw = yaw; //Previous value kept for next calculation

  return yaw;
}
//set servo position
void set_servo_position(float yaw) {
  //Map yaw to values in range[0,255]
  yaw *= 1.1;
  if(yaw < -90) yaw = -90; //Max on one side
  if(yaw > 90) yaw = 90; //Max on other side
  
  uint16_t position = (yaw+90) / 180 * 255; 
  propulsion_fan_direction = Servo_angle[position]; //Turn motor to calculated position
}

//MAIN CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
int main(void)
{
  //Initializing everything
  gpio_init();
  USS_INIT(); //Initialize US sensor
  TWI_init(); //Init TWI
  MPU6050_init(); //Initialize MPU6050
  timer1_50Hz_init(0); //in init_290.c, initialize PWM0 for servo and PWM1 for leviation fan
  timer0_init(); //PWM2 for propulsion fan

  //Initialize IMU
  Write_Reg(I2C_ADDRESS, 0x6B, 0x00);
  Write_Reg(I2C_ADDRESS, ACCEL_CONFIG, 0x00); // ±2g range
  Write_Reg(I2C_ADDRESS, GYRO_CONFIG, 0x00);  // ±500 deg/s range

  //Calibrating the IMU
  for (int m = 0; m < 500; m++) {
    Read_Gyro_Data();
    z_gyro_offset += (z_gyro);
    _delay_ms(10);
  }
  z_gyro_offset /= 500;

  //Center propulsion fan initially
  propulsion_fan_direction = CENTER;
  
  //Turn on both fans
  levitation_fan_pwm = 255; //start fan at max rpm
  propulsion_fan_pwm = 255; //start fan at max rpm

  //Initialize variables used in main loop
  float target_yaw = 0;
  float diff = 0;

  while (1) //Constantly loops while the power is on
  {
    // read IMU
    _delay_ms(20);
    Read_Gyro_Data();
  
    yaw = Calculate_Yaw();
    //Logic added to make the reference either +90 or -90 depending on if we wanted a left/right turn later on in code
    if(right){
      yaw += 90;
      if(yaw > 90) 
        yaw = 90;
    }
    if(left){
      yaw -= 90;
      if(yaw < -90) 
        yaw = -90;
    }
    
    diff = abs(yaw - target_yaw); //Calculate the difference between where we are facing now and where we want to be facing
    if (diff > 2.0) {
      set_servo_position(yaw); //Adjust motor to where it should be
    }

    //Read USS values and save them
    front_distance = scan_front_uss(); //calculate the distance
    top_distance = scan_top_uss();

    if (top_distance <= 50) { //First check if the top is detected since most important
      levitation_fan_pwm = 0; //Stop fan
      propulsion_fan_pwm = 0; //Stop fan
      return 0; //Turn off all the code.
    }
    else if (front_distance <= 25) { //Second check the front if its in front of a wall
      propulsion_fan_pwm = 0; //Stop fan
      levitation_fan_pwm = 0; //Stop fan
      
      //Scan left side
      propulsion_fan_direction = Servo_angle[LEFT]; //Turn servo all the way left
      _delay_ms(750); //Small delay
      left_distance = scan_front_uss(); //Save value from USS

      //Scan right side
      propulsion_fan_direction = Servo_angle[RIGHT]; //Turn servo all the way right
      _delay_ms(750); //Small delay
      right_distance = scan_front_uss(); //Save value from USS

      //Comparing left and right wall distances
      if(left_distance <= 25 && right_distance <= 25){ //false trigger, allows the hovercraft to go straight if there are walls on both sides
        target_yaw = yaw; //Set new target
        prev_yaw = 0;
        yaw = 0;
        propulsion_fan_direction = Servo_angle[CENTER]; //Move motor to the direction we want
        
        //Boolean for new reference point (if statements at begninning of while loop)
        right = false;
        left = false;
      }
      else if (left_distance < right_distance) { //wall on left
        target_yaw = yaw+90; //Set new target
        prev_yaw = 0;
        yaw = 90;
        propulsion_fan_direction = Servo_angle[RIGHT]; //Move motor to the direction we want
        
        //Boolean for new reference point (if statements at begninning of while loop)
        right = true;
        left = false;
      }
      else if (right_distance < left_distance) { //wall on right
        target_yaw = yaw-90; //Set new target
        prev_yaw = 0;
        yaw = -90;
        propulsion_fan_direction = Servo_angle[LEFT]; //Move motor to the direction we want
        
        //Boolean for new reference point (if statements at begninning of while loop)
        right = false;
        left = true;
      }
      //If they are equal it will just repeat the loop, detect the wall in front again, then check again. Therefore dont need a final "else"
      
      propulsion_fan_pwm = 255; //Turn fan on again
      levitation_fan_pwm = 255; //Turn fan on again
      _delay_ms(1000); //small delay to allow the sensor to turn away from any walls before scanning again
    }
  }
  return 0; //End code
}
