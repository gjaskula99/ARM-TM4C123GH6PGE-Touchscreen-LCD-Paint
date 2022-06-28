//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Maciej Kucia July 2013
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
// This is part of revision 1.0 of the EK-LM4F232 Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h" //Memory mapping of the used device (maps all the pins ports etc. to specific numbers)
#include "driverlib/fpu.h" //Routines for manipulating the floating-point unit in the Cortex-M processor.
#include "driverlib/sysctl.h" //Driver for the system controller
#include "driverlib/rom.h"		//Macros to facilitate calling functions in the ROM
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts
#include "driverlib/uart.h" 	// Driver for the UART
#include "grlib/grlib.h"	//Prototypes for the low level primitives provided by the graphics library
#include "drivers/ili9341_240x320x262K.h" //Display driver for the MULTI-INNO TECHNOLOGY
										// MI0283QT-9 TFT display with an ILI9341 controller.
#include "utils/uartstdio.h"		//Prototypes for the UART console functions.
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "drivers/touch.h"



#define GPIO_PINS_ALL GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7



//*****************************************************************************
//
// Global system tick counter holds elapsed time since the application started
// expressed in 100ths of a second.
//
//*****************************************************************************
volatile uint32_t g_ui32SysTickCount;



int_fast32_t i32CenterX, i32CenterY;
//int i32CenterX_temp = 100;
//int i32CenterY_temp = 100;
tContext sContext;

//*****************************************************************************
//
// This is the interrupt handler for the SysTick interrupt.  It is used to
// update our local tick count which, in turn, is used to check for transmit
// timeouts.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    g_ui32SysTickCount++;
}


//*****************************************************************************
//
//! Callback for calibration process
//!
//! \param X - x coordinate of point to check
//! \param Y - y coordinate of point to check
//! \param cX - x coordinate of circle center
//! \param cY - y coordinate of circle center
//! \param radius - circle radius
//!
//! \return boolean true if coordinates are within circle
//
//*****************************************************************************
inline bool IsInCircle(int32_t X,int32_t Y,int32_t cX,int32_t cY,int32_t radius)
{
	return ( (X-cX)*(X-cX) + (Y-cY)*(Y-cY) < (radius*radius) );
}



//*****************************************************************************
//
//! Callback for calibration process
//!
//! \param ulMessage is type of event
//! \param lX is a x location of touch
//! \param lY is a y location of cross center
//!
//! \return None.
//
//*****************************************************************************

int32_t size = 1;

int32_t TouchCallback(uint32_t ulMessage, int32_t lX, int32_t lY)
{
//	i32CenterX = lX;
//	i32CenterY = lY;

    bool select = false;
    if(lX< 20 && lY<20)
    {
        GrContextForegroundSet(&sContext, ClrRed);
        select = true;
    }
    if(lX> 20 && lX < 40 && lY<20)
    {
        GrContextForegroundSet(&sContext, ClrGreen);
        select = true;
    }
    if(lX> 40 && lX < 60 && lY<20)
        {
            GrContextForegroundSet(&sContext, ClrYellow);
            select = true;
        }
    if(lX> 60 && lX < 80 && lY<20)
        {
            GrContextForegroundSet(&sContext, ClrBlue);
            select = true;
        }
    if(lX> 80 && lX < 100 && lY<20)
            {
                GrContextForegroundSet(&sContext, ClrWhite);
                select = true;
            }
    if(lX> 100 && lX < 120 && lY<20)
               {
                   if(size != 10) size++;
                   select = true;
               }
    if(lX> 120 && lX < 140 && lY<20)
               {
                   if(size != 1) size--;
                   GrContextForegroundSet(&sContext, ClrWhite);
                   select = true;
               }

    if(!select) GrCircleFill(&sContext, lX, lY, size);
	return 0;
}



int
main(void)
{

    //
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
//    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
//                       SYSCTL_OSC_MAIN);

    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_8MHZ);

    //
    // Initialize the display driver.
    //
    ILI9341_240x320x262K_Init();

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sILI9341_240x320x262K);

    //
    // Initialize touchscreen driver
    //
    TouchScreenInit();
    TouchScreenCalibrate(&sContext);
    TouchScreenCallbackSet(TouchCallback);


    tRectangle color1;
        color1.i16XMin = 0;
        color1.i16YMin = 0;
        color1.i16XMax = 20;
        color1.i16YMax = 20;
        GrContextForegroundSet(&sContext, ClrRed);
        GrRectFill(&sContext, &color1);
    tRectangle color2;
        color2.i16XMin = 20;
        color2.i16YMin = 0;
        color2.i16XMax = 40;
        color2.i16YMax = 20;
        GrContextForegroundSet(&sContext, ClrGreen);
        GrRectFill(&sContext, &color2);
    tRectangle color3;
            color3.i16XMin = 40;
            color3.i16YMin = 0;
            color3.i16XMax = 60;
            color3.i16YMax = 20;
            GrContextForegroundSet(&sContext, ClrYellow);
            GrRectFill(&sContext, &color3);
    tRectangle color4;
                color4.i16XMin = 60;
                color4.i16YMin = 0;
                color4.i16XMax = 80;
                color4.i16YMax = 20;
                GrContextForegroundSet(&sContext, ClrBlue);
                GrRectFill(&sContext, &color4);

    GrContextForegroundSet(&sContext, ClrRed);
    GrCircleFill(&sContext, 90, 10, 10);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrLineDraw(&sContext, 80, 0, 100, 20);
    GrLineDraw(&sContext, 100, 0, 80, 20);

    GrContextForegroundSet(&sContext, ClrRed);
    GrLineDraw(&sContext, 100, 10, 120, 10);
    GrLineDraw(&sContext, 110, 0, 110, 20);
    GrLineDraw(&sContext, 120, 10, 140, 10);

    GrContextForegroundSet(&sContext, ClrRed);

    while(1)
    {

    }

}
