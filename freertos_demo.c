//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
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
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "accelerometer_task.h"
#include "bluetooth_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//#include "wearable_driver/i2c.h"
//#include "wearable_driver/I2C_Stellaris_API.h"




//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency. 
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************


//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
//xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run at 50 MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioInit(1);

    //
    // Print demo introduction.
    //
    //UARTprintf("\n\nWelcome to the Stellaris EK-LM4F120 FreeRTOS Demo!\n");

    //
    // Create a mutex to guard the UART.
    //
    //g_pUARTSemaphore = xSemaphoreCreateMutex();

//#######i2c test############
/*
    unsigned long ulClockValue = ROM_SysCtlClockGet();
    unsigned char i2creturn,i2ctemp = 0;

    i2c0Init(ulClockValue);
    i2creturn = i2c0ReadReg((unsigned char)0x53, (unsigned char) 0x00, &i2ctemp);

    UARTprintf("\r\n\r\nI2C read value = %d, return value = %d", i2ctemp, i2creturn);

    while (1);

*/
//################
    /*
    unsigned long baseaddr0 = 0x40020000;
    unsigned long temp = 0;
    I2CSetup(baseaddr0, 100000);
    temp = I2CRegRead(baseaddr0, 0x53, 0x00);
    UARTprintf("\r\n\r\nI2C read value = %d\r\n", temp);
    while (1);

    */

    //
    // Create the LED task.



    //
    if(AccelerometerTaskInit() != 0)
    {
        
        while(1)
        {
        }
    }

    //
    // Create the switch task.
    //
    if(BluetoothTaskInit() != 0)
    {
        
        while(1)
        {
        }
    }

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //

    while(1)
    {
    }
}

/*
void i2c_init(void)
{
    #define baseadd = (unsigned long) 0x40020000
     // Enable I2C Module.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Enable GPIO used.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure Pins for I2C function and open-drain operation with weak
    // pull-ups.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
    ROM_GPIOPinConfigure(GPIO_PB2_I2C0SCL);

    I2CMasterInitExpClk(baseadd, (ROM_SysCtlClockGet()), 0);
    I2CMasterEnable(baseadd);
}

*/
