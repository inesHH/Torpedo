//#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>

//#include "api.h"
#include "QTRSensor.c"



#define PWM_STEPS 256
#define PWM_FREQUENCY 500 // 500 Hz

#define MOTORS_TIM          TIM3
#define MOTORS_GPIO         GPIOC
#define MOTORS_RCC_GPIO     RCC_GPIOC

#define MOTOR_RIGHT_PWM     GPIO9
#define MOTOR_RIGHT_IN1     GPIO7
#define MOTOR_RIGHT_IN2     GPIO11
#define MOTOR_RIGHT_CH      TIM_OC4

#define MOTOR_LEFT_PWM      GPIO8
#define MOTOR_LEFT_IN1      GPIO6
#define MOTOR_LEFT_IN2      GPIO10
#define MOTOR_LEFT_CH       TIM_OC3

#define SENSORDroit




/* Set STM32 to 168 MHz. */
static void clock_setup(void){
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}


static void tim_init(void){
    uint32_t TIMER_Frequency = rcc_apb1_frequency * 2;
    uint32_t COUNTER_Frequency = PWM_STEPS * PWM_FREQUENCY;
    uint32_t PSC_Value = (TIMER_Frequency / COUNTER_Frequency) - 1;
    uint16_t ARR_Value = PWM_STEPS - 1;

    //-------------------------------------------------------------------------------------

	rcc_periph_clock_enable(RCC_TIM3);  /* Enable TIM3 clock. */
	rcc_periph_reset_pulse(RST_TIM3); /* Reset TIM2 peripheral to defaults. */

    /* Set the timers global mode to:
	 * - use no divider
	 * - alignment edge
	 * - count direction up
	 * */
	timer_set_mode(MOTORS_TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);


	timer_set_prescaler(MOTORS_TIM, PSC_Value); // set prescaler

	timer_enable_preload(MOTORS_TIM);                // enable preload
	timer_continuous_mode(MOTORS_TIM);               // set continuous mode
	timer_set_repetition_counter(MOTORS_TIM, 0);     // set repetition counter
	timer_set_period(MOTORS_TIM, ARR_Value);         // set period
}

static void pwm_init_chanels(void){

    //disable TIM3 channels
    timer_disable_oc_output(MOTORS_TIM,MOTOR_RIGHT_CH);
    timer_disable_oc_output(MOTORS_TIM,MOTOR_LEFT_CH);


    timer_set_oc_mode(MOTORS_TIM,MOTOR_RIGHT_CH,TIM_OCM_PWM1);
    timer_set_oc_mode(MOTORS_TIM,MOTOR_LEFT_CH,TIM_OCM_PWM1);


    timer_enable_oc_preload(MOTORS_TIM, MOTOR_RIGHT_CH);
    timer_enable_oc_preload(MOTORS_TIM, MOTOR_LEFT_CH);

    timer_set_oc_value(MOTORS_TIM,MOTOR_RIGHT_CH,0);
    timer_set_oc_value(MOTORS_TIM,MOTOR_LEFT_CH,0);


    timer_enable_oc_output(MOTORS_TIM,MOTOR_RIGHT_CH);
    timer_enable_oc_output(MOTORS_TIM,MOTOR_LEFT_CH);

    timer_enable_counter(MOTORS_TIM);
}

static void pwm_start(void){
    timer_generate_event(MOTORS_TIM, TIM_EGR_UG);
	timer_enable_counter(MOTORS_TIM);
}

static void pwm_gpio_setup(void){
    /* Enable GPIOC clock. */
	rcc_periph_clock_enable(MOTORS_RCC_GPIO);

    gpio_mode_setup(MOTORS_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, MOTOR_RIGHT_IN1 | MOTOR_RIGHT_IN2 | MOTOR_LEFT_IN1 | MOTOR_LEFT_IN2);


    gpio_mode_setup(MOTORS_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, MOTOR_RIGHT_PWM | MOTOR_LEFT_PWM);
	gpio_set_output_options(MOTORS_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,  MOTOR_RIGHT_PWM | MOTOR_LEFT_PWM);/* Push Pull, Speed 50 MHz */
	gpio_set_af(MOTORS_GPIO, GPIO_AF2, MOTOR_LEFT_PWM | MOTOR_RIGHT_PWM); // Alternate Function: TIM3 CH4



    /* Enable GPIO clock for leds. */
	rcc_periph_clock_enable(RCC_GPIOD);
	/* Enable GPIO clock for the sensor's pins. */
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Enable led as output */
	//gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LINEG_LED);
	//gpio_clear(LED_PORT,LINEG_LED);
	gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LINEG_LED | LINED_LED | QUADG_LED | QUADD_LED);
	gpio_clear(LED_PORT,LINEG_LED | LINED_LED | QUADG_LED | QUADD_LED);

}

static void motor_right(uint16_t duty_cycle, uint8_t direction){

    timer_set_oc_value(MOTORS_TIM,MOTOR_RIGHT_CH,duty_cycle); // set duty cycle

    if(direction == 1){
        gpio_set(MOTORS_GPIO, MOTOR_RIGHT_IN1);
        gpio_clear(MOTORS_GPIO, MOTOR_RIGHT_IN2);
    }
    else{
        gpio_clear(MOTORS_GPIO, MOTOR_RIGHT_IN1);
        gpio_set(MOTORS_GPIO, MOTOR_RIGHT_IN2);
    } 
    
}

static void motor_left(uint8_t duty_cycle, uint8_t direction){

    timer_set_oc_value(MOTORS_TIM,MOTOR_LEFT_CH,duty_cycle); // set duty cycle

    if(direction == 1){
        gpio_set(MOTORS_GPIO, MOTOR_LEFT_IN1);
        gpio_clear(MOTORS_GPIO, MOTOR_LEFT_IN2);
    }
    else{
        gpio_clear(MOTORS_GPIO, MOTOR_LEFT_IN1);
        gpio_set(MOTORS_GPIO, MOTOR_LEFT_IN2);
    } 
}

static void motors_init(void){
    tim_init();
    pwm_gpio_setup();
    pwm_init_chanels();

    timer_set_oc_value(MOTORS_TIM,MOTOR_RIGHT_CH,0);
    timer_set_oc_value(MOTORS_TIM,MOTOR_LEFT_CH,0);

    pwm_start();
}

static void loop(void){
    int error = 0;
    int lasterror = 0;
    int integralerror= 0;
    int erreurDroite, erreurGauche;
    /* Calculate the calibration values */
    int32_t sensorsCalib[6]= {0, 0, 0, 0, 0, 0};
    CalibrateValues(sensorsCalib);
    int M1 = 95; // the bases speeds of the Right engine
    int M2 = 95; // the bases speeds of the Left engine
    int m1Speed, m2Speed;  //the actual speeds of the L/R engine
    double KP =  0.5; // Coefficients of PID controller proportionnel
    double KI = 0; // Coefficients of PID controller integral
    double KD = -0.15; // Coefficients of PID controller derviate
    uint16_t sensors[6];
    int motoradjust;

    // look for float or double 
    // add stop if inside things are white
    while(1) {
 
        // read the values of the sensors
        readSensor(sensors);

        /* 
        if(sensors[GridDetectPING] <550) gpio_set(LED_PORT, LINEG_LED);
        else gpio_clear(LED_PORT, LINEG_LED);
        */

        /*  Calculate the erreur comparing to the calibration value */
        erreurDroite = sensorsCalib[LineFollowPIND] - sensors[LineFollowPIND];
        erreurGauche = sensorsCalib[LineFollowPING] - sensors[LineFollowPING];

        /* calculate the error */
        error = erreurGauche - erreurDroite;
        integralerror += error;

        /*  calculate the new speed according to error */
        motoradjust = KP *error + KI * integralerror + KD *(lasterror - error);
        lasterror = error; // for derivation

        m1Speed = M1 + motoradjust;
        m2Speed = M2 - motoradjust; 

        // keep the speeds in the authorised intervals
        if (m1Speed < 0) {
            m1Speed = - m1Speed;
            //m1Speed = 0;
            motor_right(m1Speed,0);
        }else {
            if (m1Speed > 255){
             m1Speed = 255;
             motor_right(m2Speed,1);
            }else{
                motor_right(m1Speed,1);
            }
            
        }
        if (m2Speed < 0) {
            m2Speed = - m2Speed;
            //m2Speed = 0;
            motor_left(m2Speed,0);
        }else {
            if (m2Speed > 255) {
                m2Speed = 255;
                motor_left(m2Speed,1);
            }else{
                motor_left(m2Speed,1);
            }

            
        }
        
        if(sensors[CenterPIND] < 250 && sensors[CenterPING] < 250){
             motor_left(0,1);
             motor_right(0,1);
             gpio_set(LED_PORT, LINEG_LED);
             /* turn back */
        }
        else{
             gpio_clear(LED_PORT, LINEG_LED);
        }

	}
}

int main(void){

    int i;

	clock_setup();
	motors_init();
    loop();
    
	

	return 0;

}


