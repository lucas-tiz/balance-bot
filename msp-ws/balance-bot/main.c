//*****************************************************************************
//
// MSP432 main.c - IMU
//
//****************************************************************************

/*TODO:
 * check interrupt durations on scope
 * error checking on start-up
 * simplify update IMU functions
 * calibrate accelerometer
 *
 */

#include "init.h"


// global variables
volatile struct imuStruct imu;  // imu struct
volatile struct encStruct encR; // right wheel struct
volatile struct encStruct encL; // left wheel struct

struct motorStruct motorR = {               // right motor struct
    .pins = {MOTOR_RF_PIN, MOTOR_RB_PIN},   // motor pins (0=forward, 1=backward)
    .duty = {MOTOR_RF_DUTY, MOTOR_RB_DUTY}, // duty cycle timer registers corresponding to pins
    .deadZone = 14.17,                      // (% duty cycle) motor dead zone (1700/12000)
    .velPID = {.kP = 0.03,                  // motor velocity control proportional gain
               .kI = 0.005,                 // motor velocity control integral gain
               .kD = 0},};                  // motor velocity control derivative

struct motorStruct motorL = {               // left motor struct
    .pins = {MOTOR_LF_PIN, MOTOR_LB_PIN},   // motor pins (0=forward, 1=backward)
    .duty = {MOTOR_LF_DUTY, MOTOR_LB_DUTY}, // duty cycle timer registers corresponding to pins
    .deadZone = 17.50,                      // (% duty cycle) motor dead zone (2100/12000)
    .velPID = {.kP = 0.03,                  // motor velocity control proportional gain
               .kI = 0.005,                     // motor velocity control integral gain
               .kD = 0},};                  // motor velocity control derivative





volatile bool imuUpdateFlag = 0;
volatile bool transmitDataFlag = 0;

volatile int debugFlag = 0;
volatile float debugVar = 0;

// prototypes
float* recordData(void);


void main(void) {
    MAP_WDT_A_holdTimer(); // hold the watchdog timer (stop from running)
    MAP_Interrupt_disableMaster(); // disable interrupts

    // configure
    Clock_Config();     // configure clocks
    LED1_Config();      // configure LED1
    LED2_Config();      // configure LED2
    I2C_Config();       // configure I2C
    UART_Config();      // configure UART
    TimerA0_Config();   // configure timer for motor PWM signals
    TimerA1_Config();   // configure timer for sensor calculations
    EncGPIO_Config(&encR, ENC_0_PORT, ENC_0_CH_A_PIN, ENC_0_CH_B_PIN); // configure encoder 0
    EncGPIO_Config(&encL, ENC_1_PORT, ENC_1_CH_A_PIN, ENC_1_CH_B_PIN); // configure encoder 1


    // initialize
    delayMs(DCO_FREQ, 500); // delay to let IMU power up

    LED2_Set(LED_GREEN);
    Enc_Initialize(&encR); // initialize encoder 0
    Enc_Initialize(&encL); // initialize encoder 1
    delayMs(DCO_FREQ, 100);
    LED2_Set(LED_OFF);

    LED2_Set(LED_RED);
    IMU_Init(&imu, 44, 1000, 2);   // initialize IMU
    delayMs(DCO_FREQ, 100);
    LED2_Set(LED_OFF);

    LED2_Set(LED_BLUE);
    IMU_Calibrate(&imu, 20000); // calibrate IMU
    LED2_Set(LED_OFF);

    LED2_Set(LED_GREEN);
    delayMs(DCO_FREQ, 2000); // allow time for positioning
    LED2_Set(LED_OFF);

    MAP_Interrupt_enableMaster(); // enable interrupts

    // run
    float* dataArr;
    while(1) {
        if (imuUpdateFlag == 1) {
//            IMU_CalcAngleFused(&imu);  // calculate IMU orientation
            imuUpdateFlag = 0;  // clear IMU update flag
        }
        if (transmitDataFlag == 1) {
            transmitDataFlag = 0; // clear flag
            dataArr = recordData();
            UART_SendFloatArray(dataArr, 9);

            debugFlag++;

            //TODO: for calibration
            if (debugFlag == 200) { // 10 seconds
                motorR.velPID.velDes = 800;
                motorL.velPID.velDes = -800;
//                *(motorL.duty[0]) = 2100;
            }
            if (debugFlag == 300) { // 15 seconds
                motorR.velPID.velDes = 0;
                motorL.velPID.velDes = 0;
//                *(motorL.duty[0]) = 0;
                debugFlag = 0;
            }

            LED2_Set(LED_GREEN); // toggle green LED
        }
        //TODO: add recalibration option
    }
}


float* recordData(void) {
    /* record data to transmit via UART */
    static float data[9];      // vector of data to send via UART

    MAP_Interrupt_disableMaster(); // disable interrupts to record data

    data[0] = encR.velFilt;
    data[1] = -encL.velFilt;
    data[2] = debugVar;

    data[3] = imu.raw.angVel[0];
    data[4] = imu.raw.angVel[1];
    data[5] = imu.raw.angVel[2];

    data[6] = imu.angle.gyro[0];
    data[7] = imu.angle.accel[0];
    data[8] = imu.angle.fused[0][0];

//    data[6] = imu.angle.fused[0][0];
//    data[8] = imu.angVel.fused[0];
//    data[7] = imu.angle.accel[0];

    MAP_Interrupt_enableMaster(); // enable interrupts

    return data;
}

