/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp007.c ========
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Memory.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/I2C.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/PWM.h>

/* Example/Board Header files */
#include "Board.h"
#include "SensorOpt3001.h"
#include "SensorI2C.h"
#include "SensorMpu9250.h"

#define TASKSTACKSIZE   1024   /* stack size for constructed tasks */

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */

PIN_Config ledPinTable[] = {
    Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


/* ==== declarations ==== */
bool init_mpu(void);
bool init_light(void);
void pwmLEDFxn(UArg arg0, UArg arg1);
void lightTaskFxn(UArg arg0);
void mpuTaskFxn(UArg arg0);
float readMpu9250();
float readOPT3001();

/*
 *  ======== echoFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
#define HDC1000_REG_TEMP           0x00 // Temperature
#define HDC1000_REG_HUM            0x01 // Humidity
#define HDC1000_REG_CONFIG         0x02 // Configuration
#define HDC1000_REG_SERID_H        0xFB // Serial ID high
#define HDC1000_REG_SERID_M        0xFC // Serial ID middle
#define HDC1000_REG_SERID_L        0xFD // Serial ID low
#define HDC1000_REG_MANF_ID        0xFE // Manufacturer ID
#define HDC1000_REG_DEV_ID         0xFF // Device ID

// Fixed values
#define HDC1000_VAL_MANF_ID        0x5449
#define HDC1000_VAL_DEV_ID         0x1000
#define HDC1000_VAL_CONFIG         0x1000 // 14 bit, acquired in sequence
#define SENSOR_DESELECT()   SensorI2C_deselect()


Task_Struct task0Struct, task1Struct, task2Struct;
Char task0Stack[TASKSTACKSIZE], task1Stack[TASKSTACKSIZE], task2Stack[TASKSTACKSIZE];

/* Mailbox Struct */
typedef struct Mailbox {
    float Value;
}AccMail, LightMail;

/* Handles created dynamically in the c */
Mailbox_Handle AccMailHandle = NULL;
Mailbox_Handle LuxMailHandle = NULL;

/* ==== Task Function for Light Sensor ==== */
float readOPT3001()
{
    uint16_t rawdata = 0;
    float lux = 0.0;

    // Read sensor value
    if (SensorOpt3001_read(&rawdata)) {
        // convert raw to readable format
        lux = SensorOpt3001_convert(rawdata);
    }
    else {
        System_printf("SensorOpt3001 I2C fault!\n");
    }

    System_flush();

    return lux;
}

/* ==== Task Function for MPU Sensor ==== */
float readMpu9250()
{
    uint16_t rawdata = 0;
    float accConvert = 0.0;

    // read MPU data
    if (SensorMpu9250_accRead(&rawdata)) {
        // convert raw to readable format
        accConvert = SensorMpu9250_accConvert(rawdata);
    }
    else {
       System_printf("SensorMPU9250 I2C fault!\n");
    }

    System_flush();

    return accConvert;
}

void pwmLEDFxn(UArg arg0, UArg arg1)
{
    AccMail AccMsg;
    LightMail LightMsg;

    PWM_Handle pwm1;
    PWM_Params params;
    uint16_t   pwmPeriod = 3000;      // Period and duty in microseconds
    uint16_t   duty = 0;

    float dutyAcc, dutyLux;

    PWM_Params_init(&params);
    params.dutyUnits = PWM_DUTY_US;
    params.dutyValue = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1 = PWM_open(Board_PWM0, &params);

    if (pwm1 == NULL) {
        System_abort("Board_PWM0 did not open");
    }
    PWM_start(pwm1);

    /* Loop forever incrementing the PWM duty */
    while (1) {
        Task_sleep((UInt) arg0);

        PWM_setDuty(pwm1, duty);

        // acc
        if (Mailbox_pend(AccMailHandle, &AccMsg, BIOS_NO_WAIT)) {
            float currAcc = AccMsg.Value;
            // absolute function
            if (currAcc < 0) {
                currAcc = -currAcc;
            }
            dutyAcc = (currAcc/10.0) * pwmPeriod;
        }

        // lux
        if (Mailbox_pend(LuxMailHandle, &LightMsg, BIOS_NO_WAIT)) {
            float currLux = LightMsg.Value;
            dutyLux = (currLux/5000.0) * pwmPeriod;
        }

        // getting the larger value
        if (dutyAcc > dutyLux) {
            duty = (int)dutyAcc;
        } else {
            duty = (int)dutyLux;
        }

        // limit of 3000 pwmPeriod for LED brightness
        if (duty > pwmPeriod) {
            duty = pwmPeriod;
        }
    }
}

/* Task Function for Light Sensor and Accelerometer */
void lightTaskFxn(UArg arg0)
{
    LightMail LightMail;

    float curr_lux;

    if (!init_light()) {
        System_abort("Error Initializing I2C lightTaskFxn\n");
        System_flush();
    }

    // Reading sensor values
    while (1) {
        // sleep so I2C can react in time
        Task_sleep((UInt)arg0);
        curr_lux = readOPT3001();

        // using mailbox to transfer msg
        LightMail.Value = curr_lux;
        Mailbox_post(LuxMailHandle, &LightMail, BIOS_NO_WAIT);
    }
}

void mpuTaskFxn(UArg arg0)
{
    AccMail AccMail;

    float curr_acc, prev_acc;

    if (!init_mpu()) {
        System_abort("Error Initializing I2C mpuTaskFxn\n");
        System_flush();
    }

    // Reading sensor values
    while (1) {
        // sleep
        Task_sleep((UInt)arg0);

        curr_acc = readMpu9250();

        if (!prev_acc) {
            prev_acc = curr_acc;
        }

        // using mailbox to transfer msg
        // we calculate the delta value to adjust the LED brightness with
        AccMail.Value = curr_acc - prev_acc;
        Mailbox_post(AccMailHandle, &AccMail, BIOS_NO_WAIT);

        prev_acc = curr_acc;
    }
}

bool init_mpu(void)
{
    // config MPU
    if (!SensorMpu9250_init()) {
        System_printf("SensorMPU9250 cannot init!\n");
        System_flush();
        return false;
    }
    SensorMpu9250_accSetRange(ACC_RANGE_4G);
    SensorMpu9250_enable(8); // X
    SensorMpu9250_enable(16); // Y
    SensorMpu9250_enable(32); // Z

    if (!SensorMpu9250_test()) {
        System_printf("SensorMPU9250 did not pass test!\n");
        System_flush();
        return false;
    }

    return true;
}

bool init_light(void)
{
    // config light sensor
    SensorOpt3001_init();
    SensorOpt3001_enable(true);

    if (!SensorOpt3001_test()) {
       System_printf("SensorOpt3001 did not pass test!\n");
       System_flush();
       return false;
    }

    return true;
}

/*
 *  ======== main ========
 */
int main(void)
{
    /* Construct BIOS objects */
    Task_Params taskParams;
    Mailbox_Params mboxParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initPWM();

    /* SYS/BIOS Mailbox create */
    Mailbox_Params_init(&mboxParams);
    AccMailHandle = Mailbox_create(sizeof(AccMail), 5, &mboxParams, NULL);
    if (AccMailHandle == NULL) {
        System_abort("Mailbox create failed\nAborting...");
    }
    LuxMailHandle = Mailbox_create(sizeof(LightMail), 5, &mboxParams, NULL);
    if (LuxMailHandle == NULL) {
        System_abort("Mailbox create failed\nAborting...");
    }

    /* I2C MPU Sensor Task */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 100000 / (Clock_tickPeriod);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)mpuTaskFxn, &taskParams, NULL);

    /* I2C Light Sensor Task */
    taskParams.arg0 = 2 * (100000 / (Clock_tickPeriod));
    taskParams.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)lightTaskFxn, &taskParams, NULL);

    /* PWM Task */
    taskParams.arg0 = 100000 / (Clock_tickPeriod);
    taskParams.stack = &task2Stack;
    Task_construct(&task2Struct, (Task_FuncPtr)pwmLEDFxn, &taskParams, NULL);

    /* Opening up I2C connection */
    if (!SensorI2C_open()) {
        System_abort("Error Initializing I2C\n");
        System_flush();
    }

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the I2C example\nSystem provider is set to SysMin."
                  " Halt the target to view any SysMin contents in ROV.\n");

    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}



