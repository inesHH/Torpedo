//#include <stdint.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f4/nvic.h>
#include <libopencm3/stm32/usart.h>

//#include "api.h"
#include "QTRSensor.c"

#define PWM_STEPS 256
#define PWM_FREQUENCY 200 // 500 Hz

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

enum State {PID, TRANS, ROTATE, FIN};
enum State S;
int32_t sensorsCalib[6]= {0, 0, 0, 0, 0, 0};
int m1Speed, m2Speed;

/* Set STM32 to 168 MHz. */
static void clock_setup(void){
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

    rcc_periph_clock_enable(RCC_GPIOD);

   /* Enable clocks for USART2. */
	rcc_periph_clock_enable(RCC_USART3);

    /* Setup GPIO pins for USART2 transmit. */
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8);

	/* Setup USART2 TX pin as alternate function. */
	gpio_set_af(GPIOD, GPIO_AF7, GPIO8);
}

static void usart_setup(void)
{
	/* Setup USART2 parameters. */
	usart_set_baudrate(USART3, 115200);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_mode(USART3, USART_MODE_TX);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART3);
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

static void motor_right(uint8_t duty_cycle, uint8_t direction){

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

static void TurnRight(void){
       motor_right(60,0);
       motor_left(70,1);
       uint16_t sensors[6];
       readSensor(sensors);
       // get to the dark grid pins are dark. 
       while (sensors[GridDetectPIND] < 550 && sensors[GridDetectPING] < 550)
       {
         readSensor(sensors);
       }

       // get the white state.
       while (sensors[GridDetectPIND] > 300 || sensors[GridDetectPING] > 300 || 
       sensors[LineFollowPIND] > 300 || sensors[LineFollowPING] > 300)
       {
         readSensor(sensors);
       }
       //motor_right(0,1);
       //motor_left(0,1);
       
}

static void TurnLeft(void){
       motor_right(70,1);
       motor_left(60,0);
       uint16_t sensors[6];
       readSensor(sensors);
       // get to the dark grid pins are dark. 
       while (sensors[GridDetectPIND] < 550 && sensors[GridDetectPING] < 550)
       {
         readSensor(sensors);
       }

       // get the white state.
       while (sensors[GridDetectPIND] > 300 || sensors[GridDetectPING] > 300 || 
       sensors[LineFollowPIND] > 300 || sensors[LineFollowPING] > 300)
       {
         readSensor(sensors);
       }
       //motor_right(0,1);
       //motor_left(0,1);
       
}

static void Transiton(void){
    /* go forward untill the two IR leds see the white line */
    motor_right(55,1);
    motor_left(55,1);
    uint16_t sensors[6];
    readSensor(sensors);    
    while (sensors[GridDetectPIND] > 300 || sensors[GridDetectPING] > 300)
    {
        readSensor(sensors);
    }
    //motor_right(0,1);
    //motor_left(0,1);    
}

static void Pid(void){
    /* the line follower function */
    int error = 0;
    int lasterror = 0;
    int integralerror= 0;
    int erreurDroite, erreurGauche;

    int M1 = 110; // the bases speeds of the Right engine
    int M2 = 110; // the bases speeds of the Left engine
    //the actual speeds of the L/R engine
    /*double KP = (0.55 * 2.5)/2; // Coefficients of PID controller proportionnel
    double KI = 0; // Coefficients of PID controller integral
    double KD = 0.55 * -0.15; //    Coefficients of PID controller derviate
    */
    double KP = 0.5 ; // Coefficients of PID controller proportionnel
    double KI = 0.0002; // Coefficients of PID controller integral
    double KD = -0.25;
    uint16_t sensors[6];
    int motoradjust;
    while(1) {
 
        // read the values of the sensors
        readSensor(sensors);
        /* sending the read value using uart*/
        //usart_send_blocking(USART3,   sensors[GridDetectPIND]/4);

        if(sensors[GridDetectPIND] > 600 || sensors[GridDetectPING] > 600){ //we are at intersection
            //motor_right(0,1);
            //motor_left(0,1);
            return ;
            
        }
        else{
            
            /* 
            if(sensors[GridDetectPING] <550) gpio_set(LED_PORT, LINEG_LED);
            else gpio_clear(LED_PORT, LINEG_LED);
            */

            /*  Calculate the erreur comparing to the calibration value */
            erreurDroite = 1* (sensors[LineFollowPIND] - sensorsCalib[LineFollowPIND]) 
            + 0 *(sensors[CenterPIND] - sensorsCalib[CenterPIND]);
            erreurGauche = 1*(sensors[LineFollowPING] - sensorsCalib[LineFollowPING])
            + 0 * (sensors[CenterPING] - sensorsCalib[CenterPING]);

            //erreurGauche = sensors[LineFollowPIND] - sensorsCalib[LineFollowPIND] ;
            //erreurDroite = sensors[LineFollowPING] - sensorsCalib[LineFollowPING] ;
            
            /* calculate the error */
            error =  erreurDroite - erreurGauche;
            integralerror += error;

            /*  calculate the new speed according to error */
            motoradjust = KP *error + KI * integralerror + KD *(lasterror - error);
            /*char buffer[20];
            itoa((int)(KP *error),buffer,10);
            usart_send_blocking(USART3, 'K' );
            usart_send_blocking(USART3, 'P' );
            for (int i = 0; i < 3; i++)
            {
                usart_send_blocking(USART3, (uint16_t)buffer[i]);
            }            
            usart_send_blocking(USART3, '\n');*/
            //usart_send_blocking(USART3,   KI * integralerror);
            //usart_send_blocking(USART3, KD *(lasterror - error));
            lasterror = error; // for derivation

            /*if( motoradjust < 50) {
                gpio_set(LED_PORT, GPIO12);
                gpio_clear(LED_PORT, GPIO13);
            }
            else {
                gpio_clear(LED_PORT, GPIO12);
                if( motoradjust > 100) gpio_set(LED_PORT, GPIO13);
                else gpio_clear(LED_PORT, GPIO13);
            }*/

            m1Speed = 0.55 *( M1 - motoradjust);
            m2Speed = 0.55 *( M2 + motoradjust); 

            // keep the speeds in the authorised intervals
            if (m1Speed < 0) {
                m1Speed = - m1Speed;
                motor_right(m1Speed,0);
            }else {
                if (m1Speed > 255){
                m1Speed = 255;
                motor_right(m1Speed,1);
                }else{
                    motor_right(m1Speed,1);
                }
                
            }
            if (m2Speed < 0) {
                m2Speed = - m2Speed;
                motor_left(m2Speed,0);
            }else {
                if (m2Speed > 255) {
                    m2Speed = 255;
                    motor_left(m2Speed,1);
                }else{
                    motor_left(m2Speed,1);
                }
                
            }
            
            /*if(sensors[CenterPIND] < 250 && sensors[CenterPING] < 250){
                motor_left(0,1);
                motor_right(0,1);
                gpio_set(LED_PORT, LINEG_LED);
                // turn back 
            }
            else{
                gpio_clear(LED_PORT, LINEG_LED);
            }*/
        }        

	}
}

int main(void){

  	clock_setup();
	motors_init();
    usart_setup();

    /*int mat[ROW][COL] =
    {
    {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,0,0,0},
            {0,0,0,1,1,0},
            {0,0,0,1,1,0},
            {0,0,0,1,1,0},
            {0,1,1,1,1,0},
            {1,1,1,1,0,0},
            {0,0,0,0,0,0}
    };

    Point source = {7, 0};
    Point dest = {3, 3};

    uint8_t path_from_source[50];



    int size = BFS(mat, source, dest,path_from_source);

    for(int loop = 0; loop < size; loop++)
      printf("%d  ", path_from_source[loop]);*/ 

    int PathIndex = 0;
    char Path[7] = {'L','R','S','L','S', 'S','R'}; //the path to follow in order to get to the final point 

    /* Calculate the calibration values */
    CalibrateValues(sensorsCalib);

    S = PID;
    while (1)
    {
        switch (S){
            case PID:
                usart_send_blocking(USART3, 'P');
                Pid(); // call the line follower function
                S = ROTATE;
                break;
            case TRANS:
                usart_send_blocking(USART3, 'T');
                Transiton(); // call the black line transit function
                S = ROTATE;
                break;
            case ROTATE:
                usart_send_blocking(USART3, 'R');
                if(Path[PathIndex] == 'L'){ TurnLeft();}
                else 
                {
                  if (Path[PathIndex] == 'R') { TurnRight(); }
                }
                PathIndex = (PathIndex == 7) ? 0 : PathIndex + 1;
                S = PID;
                break;
            default:
                Pid();
                S = ROTATE;
                break;
        }   
    }       
	return 0;
}



