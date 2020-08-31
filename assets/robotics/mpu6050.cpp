/*Code Adapted from Richard Hirst and uses header files made by Richard Hirst
    Author - Mubeen Padaniya
    Date - 3/27/2020
    Purpose - For a camera gimbal on Yaw and Pitch axes
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <wiringPi.h>

//initializing the sensor object mpu
MPU6050 mpu;
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }

    //WiringPI Setup

    if (wiringPiSetupGpio() == -1)
    exit(1);
    pinMode(12, PWM_OUTPUT); //YAW MOTOR at PWM pin 12
    pinMode(18, PWM_OUTPUT); //PITCH MOTOR at PWM pin 18
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(4000);
    pwmSetClock(98);
    // 20 ms / 4000 = 5e-3 ms -> ( 5e-3 / 2 )*180 = 0.45 deg per 1 pwm Value
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

int main()
{
    setup();
    usleep(100000);
    for (;;)
        // if programming failed, don't try to do anything
        if (!dmpReady)
            return;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        printf("FIFO overflow!\n");

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (fifoCount >= 42)
    {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);            // get the dmp in quaternion form
        mpu.dmpGetYawPitchRoll(&ypr, &q, &gravity); //get the dmp in ypr form
        int yaw = ypr[0]; //get Yaw value
        int pitch = ypr[1]; //get Pitch value
        if (yaw < 90 && yaw > -90)
        {
            move = floor(20 / ((0.45 * 2) / (yaw+90))); 
            pwmWrite(12, abs(move-400));
        }
        else
        {
            printf("Sensor yaw value out of Range max = 180 degree, min = 0 degree");
        }
        if (pitch < 90 && pitch > -90)
        {
            move = floor(20 / ((0.45 * 2) / (pitch+90)));
            pwmWrite(18, abs(move-400));
        }
        else
        {
            printf("Sensor Pitch value out of Range max = 180 degree, min = 0 degree");
        }
    }

    return 0;
