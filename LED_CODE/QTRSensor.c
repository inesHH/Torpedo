#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include "delay.c"

#define SENSOR_PORT GPIOA
#define CenterPIND 0
#define CenterPING 1
#define LineFollowPIND 2
#define LineFollowPING 3
#define GridDetectPIND 4
#define GridDetectPING 5

#define LED_PORT GPIOD
#define LINEG_LED GPIO13
#define LINED_LED GPIO15
#define QUADG_LED GPIO12
#define QUADD_LED GPIO14

const int32_t SENSOR_PIN[6] = {GPIO0, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5};

static void readSensor(uint16_t * sensorValues)
{
	uint16_t sensorCount = 6; // number of sensors used in line and quad detection 
	//int16_t sensorValues[sensorCount]; // the sensor's read  values
	//uint16_t sensorBool[sensorCount]; // 0 for white line and 1 for black

	uint16_t TimeOut = 1000; // the nax time that we would consider a line as black
	
	delay_setup();

	uint16_t compare_time;

    /* set the Sonsor PIN to OUTPUT */
    gpio_mode_setup(SENSOR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SENSOR_PIN[0] | SENSOR_PIN[1]
    | SENSOR_PIN[2] | SENSOR_PIN[3] | SENSOR_PIN[4] | SENSOR_PIN[5]);

    /* set the pin to high*/
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        sensorValues[i] = TimeOut;
        /* drive sensor line high  */
        gpio_set(SENSOR_PORT,SENSOR_PIN[i]);
    }
    /* delay 10us */
    delay_us(10);

    /* disable interupts on block */
    //nvic_disable_irq(NVIC_PIOA_IRQ); /* to switch all pins as close as possible */ 
    
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        /* make the pin as INPUT (should also ensure pull-up is disabled) */
        gpio_mode_setup(SENSOR_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SENSOR_PIN[i]);
    }
    
    /* re-enable interupts  */
    //nvic_enable_irq(NVIC_PIOA_IRQ);

    /* start counting time */ 
    compare_time = 0;
    while (compare_time < TimeOut) {
        for (uint8_t i = 0; i < sensorCount; i++)
        {
            /* check if input is null thqt means that the capacitor is decharged */ 
            if ((gpio_get(SENSOR_PORT,SENSOR_PIN[i]) == 0) && (compare_time < sensorValues[i]))
            {
                /* save the time value */
                sensorValues[i] = compare_time;
            }				
        }
        delay_us(10);
        compare_time += 10;	
    }
    /*for (uint8_t i = 0; i < sensorCount; i++)
    {
        // if timout then the line is black  otherwise it's white 
        if (sensorValues[i] < 550)sensorBool[i] = 0;// the white line 
        else sensorBool[i] = 1; //the black line 
        
    }*/
}

static void CalibrateValues(int32_t * CalibAvr){
    
	uint16_t NBCalib = 20; // number of reads in the calibration phase
    uint16_t CalibrateValues[20][6]; // the sensor's read  values
    for (uint8_t i = 0; i < 20; i++)
    {
        /* Read the sensors values */
        readSensor(CalibrateValues[i]);
    }
    for (uint8_t i = 0; i < NBCalib; i++)
    {
        /* Calculae the average */
        for (uint8_t j = 0; j < 6; j++)
        {
            /* Calculae the average */
             CalibAvr[j] += CalibrateValues[i][j];
        }
    }
    for (uint8_t j = 0; j < 6; j++)
    {
        /* Calculae the average */
            CalibAvr[j] = CalibAvr[j]/NBCalib;
    }    
}

