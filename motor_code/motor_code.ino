#include <avr/io.h>
#include "init_290.c"
#include <stdio.h>
#include <avr/delay.h>

//MOTOR CONNECTED TO P9 ON BOARD

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

//motor movement
void set_servo_position(int yaw){
    OCR1A = Servo_angle[yaw];
}

int main(void) {
  gpio_init();// in init_290.c
  uart_init(9600); //Initialize Uart, changed function
  timer1_50Hz_init(0); //in init_290.c
  
  char buffer[128];

  //i just want to spin the motor from left, right, middle, repeat
  while(1){
    _delay_ms(2000);
    set_servo_position(1);
    
    snprintf(buffer, sizeof(buffer), "scan left side\n\r");
    send_string(buffer);
    

    _delay_ms(2000);
    set_servo_position(254);
    
    snprintf(buffer, sizeof(buffer), "scan right side\n\r");
    send_string(buffer);

    _delay_ms(2000);
    set_servo_position(127);
    
  }


  return 0;
}
