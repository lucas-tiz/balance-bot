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

#include "msp.h"
#include "driverlib.h"

#include <stdbool.h>
#include <stdint.h>


////////////////////////////////////////////////////////////////////////////////
/* Macros */
#define NDEBUG 1

#define DEG_TO_RAD 0.0174533f
#define RAD_TO_DEG 57.2958f

#define FREQ_DCO CS_48MHZ
#define DIV_SMCLK CS_CLOCK_DIVIDER_4
#define DIV_TIMER_SENSE 0x02
#define DIV_TIMER_CONTROL 0x02

#define PERIOD_MOTOR 12000 // freq_dco=48e6/div_smclk=4/freq_motor=1000
#define PERIOD_SENSE 12000 // freq_dco=48e6/div_smclk=4/div_timer=2/freq_sense=500
#define PERIOD_CONTROL 12000 // freq_dco=48e6/div_smclk=4/div_timer=2/freq_control=500
#define FREQ_CONTROL 500.0f

#define PIN_MOTOR_RF GPIO_PIN4
#define PIN_MOTOR_RB GPIO_PIN5
#define PIN_MOTOR_LF GPIO_PIN7
#define PIN_MOTOR_LB GPIO_PIN6
#define REG_MOTOR_RF_DUTY &TA0CCR1 // (TA0.1 --> P2.4)
#define REG_MOTOR_RB_DUTY &TA0CCR2 // (TA0.2 --> P2.5)
#define REG_MOTOR_LF_DUTY &TA0CCR4 // (TA0.4 --> P2.7)
#define REG_MOTOR_LB_DUTY &TA0CCR3 // (TA0.3 --> P2.6)

#define PORT_ENC GPIO_PORT_P3
#define PIN_ENC0_CHA GPIO_PIN2
#define PIN_ENC0_CHB GPIO_PIN3
#define PIN_ENC1_CHA GPIO_PIN6
#define PIN_ENC1_CHB GPIO_PIN7

#define N_UART_DATA 9

#define LED_OFF 0
#define LED_RED 1
#define LED_GREEN 2
#define LED_BLUE 3


////////////////////////////////////////////////////////////////////////////////
/* Global variables */
enc_t g_enc_r; // right encoder struct
enc_t g_enc_l; // left encoder struct

volatile bool g_flag_sense = 0;
volatile bool g_flag_control = 0;
volatile bool g_flag_transmit = 0;

#ifndef NDEBUG
    volatile bool g_flag_debug = 0;
    volatile float g_debug = 0;
#endif


////////////////////////////////////////////////////////////////////////////////
/* Prototypes */
void TA1_0_IRQHandler(void);
void TA2_0_IRQHandler(void);
void PORT3_IRQHandler(void);

void configEncGpio(enc_t * enc, uint8_t port, uint8_t pinA, uint8_t pinB);
void updateControl(imu_t * imu, enc_t * enc_r, enc_t * enc_l,
                   motor_t * motor_r, motor_t * motor_l);
void recordData(imu_t * imu, float * data);


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
    MAP_CS_setDCOFrequency(FREQ_DCO);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, 1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, DIV_SMCLK);


    // Configure motor PWM with Timer A0
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, PIN_MOTOR_RF |
        PIN_MOTOR_RB | PIN_MOTOR_LF | PIN_MOTOR_LB, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, PIN_MOTOR_RF | PIN_MOTOR_RB |
        PIN_MOTOR_LF | PIN_MOTOR_LB);

    Timer_A_PWMConfig pwm_right_forward_config = // right forward PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // clock source divider
        PERIOD_MOTOR,                       // timer period
        TIMER_A_CAPTURECOMPARE_REGISTER_1,  // capture compare register
        TIMER_A_OUTPUTMODE_RESET_SET,       // reset set output mode
        0                                   // initial duty cycle
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_right_forward_config); // TA0.1 --> P2.4

    Timer_A_PWMConfig pwm_right_back_config = // right back PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        PERIOD_MOTOR,
        TIMER_A_CAPTURECOMPARE_REGISTER_2,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0 // initial duty cycle
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_right_back_config); // TA0.2 --> P2.5

    Timer_A_PWMConfig pwm_left_forward_config = // left forward PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        PERIOD_MOTOR,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_left_forward_config); // TA0.3 --> P2.6

    Timer_A_PWMConfig pwm_left_back_config = // left back PWM signal
    {   TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_1,
        PERIOD_MOTOR,
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_OUTPUTMODE_RESET_SET,
        0
    };
    MAP_Timer_A_generatePWM(TIMER_A0_BASE, &pwm_left_back_config); // TA0.4 --> P2.7

    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // start timer A0


    // Configure Timer A1 for sensor reading
    const Timer_A_UpModeConfig timer_sensor_config =
    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
        DIV_TIMER_SENSE,                    // clock source divider
        PERIOD_SENSE,                       // timer period
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // disable timer A rollover interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // enable capture compare interrupt
        TIMER_A_DO_CLEAR                    // clear counter upon initialization
    };
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &timer_sensor_config);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Interrupt_setPriority(INT_TA1_0, 2);
    MAP_Interrupt_enableInterrupt(INT_TA1_0);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);


    // Configure Timer A2 for control update
    const Timer_A_UpModeConfig timer_control_config =
    {   TIMER_A_CLOCKSOURCE_SMCLK,          // SMCLK clock source
        DIV_TIMER_CONTROL,                  // clock source divider
        PERIOD_CONTROL,                     // timer period
        TIMER_A_TAIE_INTERRUPT_DISABLE,     // disable timer A rollover interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // enable capture compare interrupt
        TIMER_A_DO_CLEAR                    // clear counter upon initialization
    };
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &timer_control_config);
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Interrupt_setPriority(INT_TA2_0, 2);
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);


    // Configure I2C
    EUSCI_B1->CTLW0 |= 1;     // disable UCB1 during configuration
    EUSCI_B1->CTLW0 = 0x0F81; // 7-bit slave addr, master, I2C, synch, SMCLK
    EUSCI_B1->BRW = 0x1E;     // set clock prescaler to get 400kHz baud rate
    P6->SEL0 |= 0x30;         // configure P6.4, P6.5 for UCB1: set bits 4 & 5
    P6->SEL1 &= ~0x30;        // configure P6.4, P6.5 for UCB1: clear bits 4 & 5
    EUSCI_B1->CTLW0 &= ~1;    // enable UCB1 after configuration


    // Configure UART to 115200 baud rate
    const eUSCI_UART_Config uart_config =
    {    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // clock source
         6,                                             // clock prescaler
         8,                                             // first mod reg
         32,                                            // second mod reg
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
    configEncGpio(&g_enc_r, PORT_ENC, GPIO_PIN2, GPIO_PIN3);
    configEncGpio(&g_enc_l, PORT_ENC, GPIO_PIN6, GPIO_PIN7);
    MAP_Interrupt_setPriority(PORT_ENC, 0);
    MAP_Interrupt_enableInterrupt(PORT_ENC);


    // Configure LEDs
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 |GPIO_PIN2);
    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0| GPIO_PIN1 |GPIO_PIN2);


    ////////////////////////////////////////////////////////////////////////////
    /* Local variables */
    imu_t imu;  // imu struct

    motor_t motor_r = {
        .reg_duty = {REG_MOTOR_RF_DUTY,
                     REG_MOTOR_RB_DUTY},
        .deadzone = 14.17,               // (% duty cycle) (1700/12000)
        .pid_vel = {.k_p = 0.03,
                    .k_i = 0.005,
                    .k_d = 0},
    };
    motor_t motor_l = {
        .reg_duty = {REG_MOTOR_LF_DUTY,
                     REG_MOTOR_LB_DUTY},
        .deadzone = 17.50,                // (% duty cycle) (2100/12000)
        .pid_vel = {.k_p = 0.03,
                    .k_i = 0.005,
                    .k_d = 0},
    };

    float data_uart[N_UART_DATA] = {0,0,0,0,0,0,0,0,0};


    ////////////////////////////////////////////////////////////////////////////
    /* Initialize */
    delayMs(FREQ_DCO, 500); // delay to let IMU power up

//    LED2_set(LED_GREEN);
//    Enc_init(&g_enc_r); // initialize encoder 0
//    Enc_init(&g_enc_l); // initialize encoder 1
//    delayMs(FREQ_DCO, 100);
//    LED2_set(LED_OFF);

    LED2_set(LED_RED);
    IMU_init(&imu, 44, 1000, 2);   // initialize IMU
    delayMs(FREQ_DCO, 100);
    LED2_set(LED_OFF);

    LED2_set(LED_BLUE);
    IMU_calibrate(&imu, 200, 20000); // calibrate IMU
    IMU_calcAngleFused(&imu, 0.002); // get initial measurement
    LED2_set(LED_OFF);

    LED2_set(LED_GREEN);
    delayMs(FREQ_DCO, 2000); // allow time for positioning
    LED2_set(LED_OFF);

    MAP_Interrupt_enableMaster(); // enable interrupts


    ////////////////////////////////////////////////////////////////////////////
    /* Run */
    while(1) {
        if (1 == g_flag_sense) {
            IMU_readVals(&imu);
            IMU_calcAngleFused(&imu, 0.002);
//            Enc_calcAngle(&g_enc_r);
//            Enc_calcAngle(&g_enc_l);
            g_flag_sense = 0;
        }

        if (1 == g_flag_control) {
//            updateControl(&imu, &g_enc_r, &g_enc_l, &motor_r, &motor_l);
//            Motor_velUpdate(&motor_r, g_enc_r.vel[0], PERIOD_MOTOR);
//            Motor_velUpdate(&motor_l, -(g_enc_l.vel[0]), PERIOD_MOTOR);
            g_flag_control = 0;
        }

        if (1 == g_flag_transmit) {
            recordData(&imu, data_uart);
//            float data_test[N_UART_DATA] = {1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9};
            UARTc_sendFloatArray(data_uart, 9); //(int)(sizeof(data_uart)/sizeof(data_uart[0])));

            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);

            g_flag_transmit = 0; // clear flag


            // NOTE: for calibration
//            #ifndef NDEBUG
//                g_flag_debug++;
//
//                if (200 = g_flag_debug) { // 10 seconds
//                    motor_r.pid_vel.vel_des = 800;
//                    motor_l.pid_vel.vel_des = -800;
//    //                *(motor_l.reg_duty.forward) = 2100;
//                }
//                if (300 == g_flag_debug) { // 15 seconds
//                    motor_r.pid_vel.vel_des = 0;
//                    motor_l.pid_vel.vel_des = 0;
//    //                *(motor_l.reg_duty.forward) = 0;
//                    g_flag_debug = 0;
//                }
//
//                LED2_set(LED_GREEN); // toggle green LED
//            #endif
        }
    }
}


////////////////////////////////////////////////////////////////////////////////
/* Interrupts */
void TA1_0_IRQHandler(void) {
    /* Timer 0 interrupt routine: read sensors */
    g_flag_sense = 1;
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


void TA2_0_IRQHandler(void) {
    /* Timer 1 interrupt routine: update control */
    static int count;

    g_flag_control = 1;

    count++;
    if (50 == count) { // transmit data at 10 Hz
        g_flag_transmit = 1;
        count = 0;
    }

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


void PORT3_IRQHandler(void) {
    /* Encoder interrupt handler */
//    MAP_Interrupt_disableMaster(); // disable interrupts
    uint_fast16_t status = MAP_GPIO_getEnabledInterruptStatus(PORT_ENC);

    if (status <= (PIN_ENC0_CHA | PIN_ENC0_CHB)) { // check which encoder
        Enc_update(&g_enc_r);
//        LED2_Set(LED_GREEN);
    }
    else if (status <= (PIN_ENC1_CHA | PIN_ENC1_CHB)) {
        Enc_update(&g_enc_l);
//        LED2_Set(LED_BLUE);
    }

    MAP_GPIO_clearInterruptFlag(PORT_ENC, status);
//    MAP_Interrupt_enableMaster(); // disable interrupts
}


////////////////////////////////////////////////////////////////////////////////
/* Functions */
void configEncGpio(enc_t * enc, uint8_t port, uint8_t pinA, uint8_t pinB) {
    enc->port = port;
    enc->pins[0] = pinA;
    enc->pins[1] = pinB;

    MAP_GPIO_setAsInputPinWithPullDownResistor(port, pinA | pinB);
    MAP_GPIO_clearInterruptFlag(port, pinA | pinB);
    MAP_GPIO_enableInterrupt(port, pinA | pinB);
}


void updateControl(imu_t * imu, enc_t * enc_r, enc_t * enc_l,
    motor_t * motor_r, motor_t * motor_l) {
    /* Update motor velocity set-points based on sensor measurements*/

    //TODO: put gains somewhere?
    static float k_lqr[4] = {-0.1000f, -0.8716f, -410.4197f, -71.6509f};

    // create state vector of angles and angular velocities
    float x[4];
    x[0] = (enc_r->pos[0])*DEG_TO_RAD;          // alpha: wheel angle relative to chassis
    x[1] = (enc_r->vel_filt)*DEG_TO_RAD;        // alpha vel: wheel angular velocity relative to chassis
    x[2] = (imu->angle.fused[0][0])*DEG_TO_RAD; // theta2: chassis angle
    x[3] = (imu->ang_vel.fused[0])*DEG_TO_RAD;  // theta2 vel: chassis angular velocity

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
    vel_alpha = vel_alpha + (accel_alpha/FREQ_CONTROL)*RAD_TO_DEG;

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

    motor_r->pid_vel.vel_des = vel_alpha;
    motor_l->pid_vel.vel_des = vel_alpha;
}


void recordData(imu_t * imu, float * data) {
    /* Record data to transmit via UART */
//    data[0] = g_enc_r.vel_filt;
//    data[1] = -g_enc_l.vel_filt;
//    #ifndef NDEBUG
//        data[2] = g_debug;
//    #endif

    data[3] = imu->raw.ang_vel[0];
    data[4] = imu->raw.ang_vel[1];
    data[5] = imu->raw.ang_vel[2];

    data[6] = imu->angle.gyro[0];
    data[7] = imu->angle.accel[0];
    data[8] = imu->angle.fused[0][0];

//    data[6] = imu->angle.fused[0][0];
//    data[8] = imu->ang_vel.fused[0];
//    data[7] = imu->angle.accel[0];
}




