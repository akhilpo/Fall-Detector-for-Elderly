//*****************************************************************************
//
// switch_task.c - A simple switch task to process the buttons.
//
// Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
//
//*****************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "utils/uartstdio.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "accelerometer_task.h"
#include "bluetooth_task.h"

//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define BLUETOOTHTASKSTACKSIZE        512         // Stack size in words
#define BLUETOOTHTASKPERIOD           100

extern xQueueHandle g_pAccelerometerQueue;
//extern xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// This task reads the buttons' state and passes this information to LEDTask.
//
//*****************************************************************************
void BluetoothTask(void *pvParameters)
{
    portTickType ulLastTime;
    unsigned char cMessage;
    const char msg = 'a';

    
    ulLastTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {
        if(xQueueReceive(g_pAccelerometerQueue, &cMessage, 0) == pdPASS)
        {
            if (cMessage == 0x01)
            {
 //               UARTprintf("\r\n Fall Detected\r\n");
                UARTwrite(&msg, 1);
            }
        }


        //
        // Wait for the required amount of time to check back.
        //
        vTaskDelayUntil(&ulLastTime, BLUETOOTHTASKPERIOD / portTICK_RATE_MS);
    }
}

//*****************************************************************************
//
// Initializes the Bluetooth task.
//
//*****************************************************************************
unsigned long
BluetoothTaskInit(void)
{
   
    if(xTaskCreate(BluetoothTask, (signed portCHAR *)"Bluetooth",
                   BLUETOOTHTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_BLUETOOTH_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}
