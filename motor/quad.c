//#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>

//#include "api.h"
#include "QTRSensor.c"

static void GoStraight(){
       motor_right(180,1);
       motor_left(180,1);
       for (int i = 0; i < 500000; i++)
       {
           delay_us(1000);
       }
       motor_right(0,1);
       motor_left(0,1);
       
}

static void TurnLeft(){
       motor_right(100,0);
       motor_left(200,1);
       for (int i = 0; i < 1000000; i++)
       {
           delay_us(1000);
       }
       motor_right(0,1);
       motor_left(0,1);
       
}

static void TurnRight(){
       motor_right(200,1);
       motor_left(100,0);
       for (int i = 0; i < 1000000; i++)
       {
           delay_us(1000);
       }
       motor_right(0,1);
       motor_left(0,1);
       
}

static void GoBack(){
       motor_right(100,0);
       motor_left(180,1);
       for (int i = 0; i < 1800000; i++)
       {
           delay_us(1000);
       }
       
       motor_right(0,1);
       motor_left(0,1);
}
// quadrillage detection we'll use the 3rd and 4th data of sensors value to detection the edges 
