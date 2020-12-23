/**
 * @file main.c
 * @brief Balance two-wheeled robot
 *
 * Stabilize two-wheeled balancing robot in upright orientation
 *
 * @author Lucas Tiziani
 * @date 2020-12-01
 *
 */

/*TODO:
 * take sensor reads/control updates out of interrupt
 *   then make some stuff not global
 * check interrupt durations on scope
 * include left wheel in system control
 * error checking on start-up
 * simplify update IMU functions
 * calibrate accelerometer
 * byte stuffing
 * recalibration option
 * send new control parameters via UART
 *
 */


////////////////////////////////////////////////////////////////////////////////
/* Header files */
#include "mpu6050.h"
#include "motor.h"
#include "enc.h"
#include "uart_cust.h"
#include "util.h"


//#include <i2c_cust.h>
//#include "interrupts.h"

#include "msp.h"
#include "driverlib.h"

#include <stdbool.h>
#include <stdint.h>
//#include <stdio.h>


////////////////////////////////////////////////////////////////////////////////
/* Macros */
#define NDEBUG 1

#define DEG_TO_RAD 0.0174533         // degrees to radians
#define RAD_TO_DEG 57.2958           // radians to degrees

// clocks
#define DCO_FREQ 48E+6  // clock frequency
#define SMCLK_DIV CS_CLOCK_DIVIDER_4     // subsystem master clock divider
#define SMCLK_FREQ DCO_FREQ/4   // subsystem master clock frequency

// timers
#define IMU_CALC_FREQ 100.0       // (Hz) frequency of IMU position calculations
//#define ENC_CALC_FREQ 100.0       // (Hz) frequency at which encoder angles are calculated
#define CONTROL_FREQ 100.0        // (Hz) control update frequency
#define MOTOR_FREQ 1000         // motor PWM timer frequency
//#define MOTOR_PERIOD SMCLK_FREQ/MOTOR_FREQ // motor PWM timer period

// motor pins/registers
#define MOTOR_RF_PIN GPIO_PIN4 // right motor forward pin   (TA0.1 --> P2.4)
#define MOTOR_RB_PIN GPIO_PIN5 // right motor backward pin  (TA0.2 --> P2.5)
#define MOTOR_LF_PIN GPIO_PIN7 // left motor forward pin    (TA0.4 --> P2.7)
#define MOTOR_LB_PIN GPIO_PIN6 // left motor backward pin   (TA0.3 --> P2.6)
#define MOTOR_RF_DUTY &TA0CCR1 // right motor for duty reg  (TA0.1 --> P2.4)
#define MOTOR_RB_DUTY &TA0CCR2 // right motor back duty reg (TA0.2 --> P2.5)
#define MOTOR_LF_DUTY &TA0CCR4 // left motor for duty reg   (TA0.4 --> P2.7)
#define MOTOR_LB_DUTY &TA0CCR3 // left motor back duty reg  (TA0.3 --> P2.6)

// encoder ports/pins
#define ENC_0_PORT GPIO_PORT_P3     // encoder 0 port
#define ENC_0_CH_A_PIN GPIO_PIN2    // encoder 0 channel A pin
#define ENC_0_CH_B_PIN GPIO_PIN3    // encoder 0 channel B pin
#define ENC_1_PORT GPIO_PORT_P3     // encoder 1 port
#define ENC_1_CH_A_PIN GPIO_PIN6    // encoder 1 channel A pin
#define ENC_1_CH_B_PIN GPIO_PIN7    // encoder 1 channel B pin

// LEDs
#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3


////////////////////////////////////////////////////////////////////////////////
/* Global variables */
volatile imu_t g_imu;  // imu struct
volatile enc_t g_enc_r; // right encoder struct
volatile enc_t g_enc_l; // left encoder struct

motor_t g_motor_r = {               // right motor struct
//    .pins = {MOTOR_RF_PIN, MOTOR_RB_PIN},   // motor pins (0=forward, 1=backward)
    .reg_duty = {MOTOR_RF_DUTY, // duty cycle timer registers corresponding to pins
                 MOTOR_RB_DUTY},
    .deadzone = 14.17,                      // (% duty cycle) motor dead zone (1700/12000)
    .pid_vel = {.k_p = 0.03,                  // motor velocity control proportional gain
                .k_i = 0.005,                 // motor velocity control integral gain
                .k_d = 0},                    // motor velocity control derivative
};

motor_t g_motor_l = {               // left motor struct
//    .pins = {MOTOR_LF_PIN, MOTOR_LB_PIN},   // motor pins (0=forward, 1=backward)
    .reg_duty = {MOTOR_LF_DUTY,  // duty cycle timer registers corresponding to pins
                 MOTOR_LB_DUTY},
    .deadzone = 17.50,                      // (% duty cycle) motor dead zone (2100/12000)
    .pid_vel = {.k_p = 0.03,                  // motor velocity control proportional gain
                .k_i = 0.005,                     // motor velocity control integral gain
                .k_d = 0},                  // motor velocity control derivative
};

volatile bool g_flag_imu_read = 0;
volatile bool g_flag_transmit = 0;

#ifndef NDEBUG
    volatile bool g_flag_debug = 0;
    volatile float g_debug = 0;
#endif


////////////////////////////////////////////////////////////////////////////////
/* Prototypes */
//void TA0_0_IRQHandler(void);
void TA1_0_IRQHandler(void);
void PORT3_IRQHandler(void);

void EncGPIO_Config(volatile enc_t * enc, uint8_t port, uint8_t pinA, uint8_t pinB);
void updateControl(void);
float* recordData(void);


////////////////////////////////////////////////////////////////////////////////
/* Configure MSP */
void main(void) {
    MAP_WDT_A_holdTimer(); // hold the watchdog timer (stop from running)
    MAP_Interrupt_disableMaster(); // disable interrupts


    // Configure master and subsystem master clocks
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1); // higher voltage level for 48 MHz
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_setDCOFrequency(48E+6);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, 1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4);


    // Configure motor PWM with Timer A0
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, MOTOR_RF_PIN |
        MOTOR_RB_PIN | MOTOR_LF_PIN | MOTOR_LB_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, MOTOR_RF_PIN | MOTOR_RB_PIN |
        MOTOR_LF_PIN | MOTOR_LB_PIN);

    Timer_A_PWMConfig pwm_right_forward_config = // right forward PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // clock source divider
        MOTOR_PERIOD,                       // timer period
        TIMER_A_CAPTURECOMPARE_REGISTER_1,  // capture compare register
        TIMER_A_OUTPUTMODE_RESET_SET,       // reset set output mode
        0                                   // initial duty cycle
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_right_forward_config); // TA0.1 --> P2.4

    Timer_A_PWMConfig pwm_right_back_config = // right back PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0 // initial duty cycle
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_right_back_config); // TA0.2 --> P2.5

    Timer_A_PWMConfig pwm_left_forward_config = // left forward PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_left_forward_config); // TA0.3 --> P2.6

    Timer_A_PWMConfig pwm_left_back_config = // left back PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        MOTOR_PERIOD,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_left_back_config); // TA0.4 --> P2.7

    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // start timer A0


    // Configure Timer A1 for sensor reading: 100 Hz
    const Timer_A_UpModeConfig timer_sensor_config =
    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
        0x02,                               // clock source divider
        SMCLK_FREQ/IMU_CALC_FREQ/2,         // timer period
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // disable timer A rollover interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // enable capture compare interrupt
        TIMER_A_DO_CLEAR                    // clear counter upon initialization
    };
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &timer_sensor_config);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Interrupt_setPriority(INT_TA1_0, 0); //TODO
    MAP_Interrupt_enableInterrupt(INT_TA1_0);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);


    // Configure I2C
    EUSCI_B1->CTLW0 |= 1;     // disable UCB1 during configuration
    EUSCI_B1->CTLW0 = 0x0F81; // 7-bit slave address, master, I2C, synch mode, use SMCLK
    EUSCI_B1->BRW = 0x1E;     // set clock prescaler to get 400kHz baud rate
    P6->SEL0 |= 0x30;         // configure P6.4, P6.5 for UCB1: set register bits 4 & 5
    P6->SEL1 &= ~0x30;        // configure P6.4, P6.5 for UCB1: clear register bits 4 & 5
    EUSCI_B1->CTLW0 &= ~1;    // enable UCB1 after configuration


    // Configure UART to 38400 baud rate
    const eUSCI_UART_Config uart_config =
    {    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // clock source
         19,                                            // clock prescaler
         8,                                             // first mod reg
         85,                                            // second mod reg
         EUSCI_A_UART_NO_PARITY,                        // no parity
         EUSCI_A_UART_LSB_FIRST,                        // LSB first
         EUSCI_A_UART_ONE_STOP_BIT,                     // one stop bit
         EUSCI_A_UART_MODE,                             // UART mode
         EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // oversampling on
    };
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
        (GPIO_PIN2 | GPIO_PIN3), GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A0_BASE, &uart_config);
    MAP_UART_enableModule(EUSCI_A0_BASE);


    // Configure encoder pins
    EncGPIO_Config(&g_enc_r, GPIO_PORT_P3, GPIO_PIN2, GPIO_PIN3); // configure encoder 0
    EncGPIO_Config(&g_enc_l, GPIO_PORT_P3, GPIO_PIN6, GPIO_PIN7); // configure encoder 1


    // Configure LEDs
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 |GPIO_PIN2);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0| GPIO_PIN1 |GPIO_PIN2);


    ////////////////////////////////////////////////////////////////////////////
    /* Initialize */
    delayMs(DCO_FREQ, 500); // delay to let IMU power up

    LED2_set(LED_GREEN);
    Enc_init(&g_enc_r); // initialize encoder 0
    Enc_init(&g_enc_l); // initialize encoder 1
    delayMs(DCO_FREQ, 100);
    LED2_set(LED_OFF);

    LED2_set(LED_RED);
    IMU_init(&g_imu, 44, 1000, 2);   // initialize IMU
    delayMs(DCO_FREQ, 100);
    LED2_set(LED_OFF);

    LED2_set(LED_BLUE);
    IMU_calibrate(&g_imu, 20000); // calibrate IMU //TODO: cal_cycles as argument
    LED2_set(LED_OFF);

    LED2_set(LED_GREEN);
    delayMs(DCO_FREQ, 2000); // allow time for positioning
    LED2_set(LED_OFF);

    MAP_Interrupt_enableMaster(); // enable interrupts


    ////////////////////////////////////////////////////////////////////////////
    /* Run */
    float* data_arr;
    while(1) {
//        if (g_flag_imu_read == 1) {
//            IMU_CalcAngleFused(&imu);  // calculate IMU orientation
//            g_flag_imu_read = 0;  // clear IMU update flag
//        }
        if (g_flag_transmit == 1) {
            g_flag_transmit = 0; // clear flag
            data_arr = recordData();
            UARTc_sendFloatArray(data_arr, 9);

            #ifndef NDEBUG
                g_flag_debug++;

                //TODO: for calibration
                if (g_flag_debug == 200) { // 10 seconds
                    g_motor_r.pid_vel.vel_des = 800;
                    g_motor_l.pid_vel.vel_des = -800;
    //                *(g_motor_l.reg_duty.forward) = 2100;
                }
                if (g_flag_debug == 300) { // 15 seconds
                    g_motor_r.pid_vel.vel_des = 0;
                    g_motor_l.pid_vel.vel_des = 0;
    //                *(g_motor_l.reg_duty.forward) = 0;
                    g_flag_debug = 0;
                }

                LED2_set(LED_GREEN); // toggle green LED
            #endif
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/* Interrupts */
//void TA0_0_IRQHandler(void) {
//    /* timer 0 interrupt routine*/
//    MAP_Interrupt_disableMaster(); // disable interrupts
//
//    //TODO: implement controller here?
//
//    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
//    MAP_Interrupt_enableMaster(); // enable interrupts
//}


void TA1_0_IRQHandler(void) {
    /* Timer 1 interrupt routine: read sensors*/
//    MAP_Interrupt_disableMaster(); // disable interrupts
    static int count;

    IMU_readVals(&g_imu);     // update values from IMU
    IMU_calcAngleFused(&g_imu);  // calculate IMU orientation

    Enc_calcAngle(&g_enc_r); // calculate encoder angles
    Enc_calcAngle(&g_enc_l); // calculate encoder angles

    updateControl(); // update motor velocity setpoints
    Motor_velUpdate(&g_motor_r, g_enc_r.vel[0]); // update right motor velocity
    Motor_velUpdate(&g_motor_l, -(g_enc_l.vel[0])); // update left motor velocity

    count++; // increment data transmission counter
//    g_flag_imu_read = 1; // raise  IMU update flag

    if (count == 5) { // transmit data at 20 Hz
        g_flag_transmit= 1;
        count = 0;
    }

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
//    MAP_Interrupt_enableMaster(); // enable interrupts
}


void PORT3_IRQHandler(void) {
    /* Encoder interrupt handler */
//    MAP_Interrupt_disableMaster(); // disable interrupts
    uint_fast16_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);

    if (status <= (ENC_0_CH_A_PIN | ENC_0_CH_B_PIN)) { // check which encoder
        Enc_update(&g_enc_r);
//        LED2_Set(LED_GREEN);
    }
    else if (status <= (ENC_1_CH_A_PIN | ENC_1_CH_B_PIN)) {
        Enc_update(&g_enc_l);
//        LED2_Set(LED_BLUE);
    }

    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
//    MAP_Interrupt_enableMaster(); // disable interrupts
}


////////////////////////////////////////////////////////////////////////////////
/* Functions */
void EncGPIO_Config(volatile enc_t * enc, uint8_t port, uint8_t pinA, uint8_t pinB) {
    enc->port = port;
    enc->pins[0] = pinA;
    enc->pins[1] = pinB;

    MAP_GPIO_setAsInputPinWithPullDownResistor(port, pinA | pinB);
    MAP_GPIO_clearInterruptFlag(port, pinA | pinB);
    MAP_GPIO_enableInterrupt(port, pinA | pinB);
    MAP_Interrupt_setPriority(port, 0); //TODO
    MAP_Interrupt_enableInterrupt(port);
}


void updateControl(void) {
    /* Update motor velocity set-points based on sensor measurements*/

    //TODO: put gains somewhere
    static float k_lqr[4] = {-0.1000, -0.8716, -410.4197, -71.6509};

    // create state vector of angles and angular velocities
    float x[4];
    x[0] = (g_enc_r.pos[0])*DEG_TO_RAD;          // alpha: wheel angle relative to chassis
    x[1] = (g_enc_r.vel_filt)*DEG_TO_RAD;        // alpha vel: wheel angular velocity relative to chassis
    x[2] = (g_imu.angle.fused[0][0])*DEG_TO_RAD; // theta2: chassis angle
    x[3] = (g_imu.ang_vel.fused[0])*DEG_TO_RAD;  // theta2 vel: chassis angular velocity

    // calculate acceleration input
    int i;
    float accel_alpha = 0; // initialize desired wheel angular acceleration relative to chassis
    for (i = 0; i < 4; i++) {
        accel_alpha = accel_alpha - k_lqr[i]*x[i]; // calculate acceleration from LQR state-feedback control law
    }

//    // impose acceleration limit
//    if (accel_alpha > ACCEL_LIMIT)
//    {
//        accel_alpha = ACCEL_LIMIT;
//    }
//    else if (accel_alpha < -ACCEL_LIMIT)
//    {
//        accel_alpha = -ACCEL_LIMIT;
//    }

    // integrate acceleration to get velocity input
    float vel_alpha = 0;
    vel_alpha = vel_alpha + (accel_alpha/CONTROL_FREQ)*RAD_TO_DEG;

    #ifndef NDEBUG
        g_debug = vel_alpha;
    #endif

//    // impose velocity limit
//    if (vel_alpha > VEL_LIMIT)
//    {
//        vel_alpha = VEL_LIMIT;
//    }
//    else if (vel_alpha < -VEL_LIMIT)
//    {
//        vel_alpha = -VEL_LIMIT;
//    }

    g_motor_r.pid_vel.vel_des = vel_alpha;
    g_motor_l.pid_vel.vel_des = vel_alpha;
}






float* recordData(void) {
    /* record data to transmit via UART */
    static float data[9];      // vector of data to send via UART

//    MAP_Interrupt_disableMaster(); // disable interrupts to record data

    data[0] = g_enc_r.vel_filt;
    data[1] = -g_enc_l.vel_filt;
    #ifndef NDEBUG
        data[2] = g_debug;
    #endif

    data[3] = g_imu.raw.ang_vel[0];
    data[4] = g_imu.raw.ang_vel[1];
    data[5] = g_imu.raw.ang_vel[2];

    data[6] = g_imu.angle.gyro[0];
    data[7] = g_imu.angle.accel[0];
    data[8] = g_imu.angle.fused[0][0];

//    data[6] = g_imu.angle.fused[0][0];
//    data[8] = g_imu.ang_vel.fused[0];
//    data[7] = g_imu.angle.accel[0];

//    MAP_Interrupt_enableMaster(); // enable interrupts

    return data;
}




