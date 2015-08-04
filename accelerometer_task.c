#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "drivers/buttons.h"
#include "drivers/rgb.h"
#include "utils/uartstdio.h"
// #include "switch_task.h"
// #include "led_task.h"
#include "priorities.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "wearable_driver/i2c.h"
#include "accelerometer_task.h"

//*****************************************************************************
//
// The stack size for the display task.
//
//*****************************************************************************
#define ACCELEROMETERTASKSTACKSIZE        512         // Stack size in words
#define ACCELEROMETERTASKPERIOD            250


#define ACCELEROMETER_ITEM_SIZE           sizeof(unsigned char)
#define ACCELEROMETER_QUEUE_SIZE          10



#define I2C0BASEADDR        0x40020000
#define SLAVEID             0x53
//Accelerometer Registers
#define DEVID       0x00
#define THRESH_FF   0x28
#define TIME_FF     0x29
#define INT_MAP     0x2F
#define INT_ENABLE  0x2E
#define POWER_CTL   0x2D
#define INT_SOURCE  0x30
#define FF_INT_BIT  (1<<2)


//extern xQueueHandle g_pLEDQueue;
//extern xSemaphoreHandle g_pUARTSemaphore;
xQueueHandle g_pAccelerometerQueue;

static unsigned long g_ulColors[3] = { 0x0000, 0x0000, 0x0000 };
static unsigned char g_ucColorsIndx;

//*****************************************************************************
//
// This task reads the buttons' state and passes this information to LEDTask.
//
//*****************************************************************************
void AccelerometerTask(void *pvParameters)
{
    portTickType ulLastTime;
    unsigned char ucMessageAccelerometer = 1;
    volatile unsigned char temp = 0;

//    unsigned long ulAccelerometerDelay = 50;
    
    //
    // Get the current tick count.
    //
    ulLastTime = xTaskGetTickCount();

    //
    // Loop forever.
    //
    while(1)
    {
        temp = 2;
  
      temp = I2CRegRead(I2C0BASEADDR, SLAVEID, INT_SOURCE);
  
       if (temp & FF_INT_BIT)
        {
//            RGBEnable();
            if(xQueueSend(g_pAccelerometerQueue, &ucMessageAccelerometer, portMAX_DELAY) !=
                   pdPASS)
                {
                    while(1);
                }
        }

                //
        // Wait for the required amount of time to check back.
        //
        vTaskDelayUntil(&ulLastTime, ACCELEROMETERTASKPERIOD / portTICK_RATE_MS);
    }
}

//*****************************************************************************
//
// Initializes the switch task.
//
//*****************************************************************************
unsigned long
AccelerometerTaskInit(void)
{
 //   volatile unsigned char foo = 28;

    RGBInit(1);
    RGBIntensitySet(0.3f);

    //
    // Turn on the Green LED
    //
    g_ucColorsIndx = 0;
    g_ulColors[g_ucColorsIndx] = 0x8000;
    RGBColorSet(g_ulColors);
    RGBDisable();


    //////////////
    g_pAccelerometerQueue = xQueueCreate(ACCELEROMETER_QUEUE_SIZE, ACCELEROMETER_ITEM_SIZE);

    I2CSetup(I2C0BASEADDR, 40000);

    if (((I2CRegRead(I2C0BASEADDR, SLAVEID, DEVID)) & 0xE5) == 0)
    {
        while (1);
    }

    I2CRegWrite(I2C0BASEADDR, SLAVEID, THRESH_FF, 0x06);
    simple_delay();

    I2CRegWrite(I2C0BASEADDR, SLAVEID, TIME_FF, 0x15);
    simple_delay();

    I2CRegWrite(I2C0BASEADDR, SLAVEID, INT_MAP, 0x00);
    simple_delay();

    I2CRegWrite(I2C0BASEADDR, SLAVEID, POWER_CTL, 0x08);
    simple_delay();

    I2CRegWrite(I2C0BASEADDR, SLAVEID, INT_ENABLE, 0x04);
    simple_delay();

    if(xTaskCreate(AccelerometerTask, (signed portCHAR *)"Acclerometer",
                   ACCELEROMETERTASKSTACKSIZE, NULL, tskIDLE_PRIORITY +
                   PRIORITY_ACCELEROMETER_TASK, NULL) != pdTRUE)
    {
        return(1);
    }

    //
    // Success.
    //
    return(0);
}

void simple_delay(void) 
{
    volatile unsigned int counter;

    for (counter = 0; counter<=60000; counter++);

}





