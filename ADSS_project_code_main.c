/*
 * main.c
 *
 *  Created on: 01-Jul-2018
 *  IISc Fellowship Project
 *      Author: Swagath Babu
 * 
 */




#include <stdint.h>
//#include "inc/tm4c123gh6pm.h"
#include <stdbool.h>
#include "stdlib.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.c"
#include <string.h>


void sensor_inputInt();
void Captureinit();
void InitConsole(void);
float read_distance();   ///defined now
int runstepper();

const double temp = 1.0/80.0;

volatile uint32_t present_pulse=0;      //Stores the pulse length

volatile uint8_t echowait=0;    // main code if a pulse is being read at the moment
int count=0;
uint32_t pinVal=0;	// variable to hold the pinRead for switch1 onboard



void sensor_inputInt()
{
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);          //Clear interrupt flag. Since we only enabled on this is enough

    // If it's a rising edge then set he timer to 0
    //It's in periodic mode so it was in some random value

    if ( GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2)
    {
        HWREG(TIMER2_BASE + TIMER_O_TAV) = 0; //Loads value 0 into the timer.
        TimerEnable(TIMER2_BASE,TIMER_A);
        echowait=1;
    }

    // If it's a falling edge that was detected, then get the value of the counter
    else
    {
        present_pulse = TimerValueGet(TIMER2_BASE,TIMER_A); //record value
        TimerDisable(TIMER2_BASE,TIMER_A);
        echowait=0;
    }
}

void Captureinit()
{
    // Set the timer to be periodic.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlDelay(3);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
    TimerEnable(TIMER2_BASE,TIMER_A);
}

void InitConsole(void)
{
    // Enable GPIO port A which is used for UART0 pins.

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);

    // Configure the pin muxing for UART0 functions on port A0 and A1.

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);         // Enable UART0 to configure the clock.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);         // internal 16MHz oscillator as the UART clock source.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);       // Selecting the alternate (UART) function for these pins.
    UARTStdioConfig(0, 115200, 16000000);          // Initialize the UART for console I/O.

}

/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
    int i, j;
    for(i = 0 ; i < n; i++)
        for(j = 0; j < 3180; j++) {}   /* do nothing for 1 ms */
}


int main()
{
//Set system clock to 80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    InitConsole();            //Configures the UART
    Captureinit();            //Configures the timer

    //Configure Trigger pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);  //PORTA PA3

    //Configure Echo pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);  //PORTA PA2
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2,GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTA_BASE,sensor_inputInt);
///**********************************porte for stepper
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlDelay(3);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_0| GPIO_PIN_1|GPIO_PIN_2| GPIO_PIN_3);




////////////////////////////////////////////*********************** //Configure led pin portF
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlDelay(3);
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_0| GPIO_PIN_4);// DECLARING AS INPUT
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);	// enable F4's pullup, the drive strength won't affect the input

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2| GPIO_PIN_3);
//////////////************************************////////////////
    while(1)
    {
        ////////////////////////////////////////////////
        present_pulse = read_distance();

        pinVal= GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);	// read F4

        if( (pinVal & GPIO_PIN_4)==0)
        {
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 14);	// turn on one LED
            delayMs(2000);
            GPIOPinWrite( GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 ,0);
            runstepper();
            delayMs(1000);



        }

        else
        {
            GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);	// turn on a different LED
        }
        delayMs(70);

        if(present_pulse<=12)
        {
            count++;
        }
        else
        {
            count=0;
        }
        delayMs(100);

        UARTprintf("count = %2d \n" , count);

        if(count>25)
        {
            UARTprintf("*****Access Authorised**** \n" );
            GPIOPinWrite( GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 ,8);
            delayMs(1000);
            GPIOPinWrite( GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 ,0);
            runstepper();
            delayMs(1000);

        }
        else
        {
            GPIOPinWrite( GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 ,0 );
        }

    }
}

float read_distance()
{
    if(echowait != 1)                   //Checks if a pulse read is in progress
    {
        //Does the required pulse of 10uS
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
        SysCtlDelay(266);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);

        while(echowait != 0);              //This makes the code wait for a reading to finish

        //Converts the counter value to centimeter.
        present_pulse =(uint32_t)(temp * present_pulse);
        present_pulse = present_pulse / 58;
        //Prints out the distance measured.
        UARTprintf("distance = %2dcm \n" , present_pulse);
    }
    //SysCtlDelay(400000);
    //delayMs(10);
}


int runstepper()
{
    int del=80;
    int val=6;
    int mul=12;
    //int val=mul*20;
    delayMs(500);
    UARTprintf(".............Please wait........................................\n" );

    for(float count=1; count<=val; count++)             //count={(3,90),(6,180),(9,270),(12,360)
    {
        GPIO_PORTE_DATA_R =0b00001000;
        delayMs(del);
        GPIO_PORTE_DATA_R =0b00000100;
        delayMs(del);
        GPIO_PORTE_DATA_R =0b00000010;
        delayMs(del);
        GPIO_PORTE_DATA_R =0b00000001;
        delayMs(del);


        if (count==val)
        {
            UARTprintf("..........Pick up the key.........\n" );
            GPIO_PORTF_DATA_R =0b00000100;
            delayMs(4000);
            GPIO_PORTF_DATA_R =0b00000000;

            for(float count1=1; count1<=val; count1++)
            {

                GPIO_PORTE_DATA_R =0b00000001;
                delayMs(del);
                GPIO_PORTE_DATA_R =0b00000010;
                delayMs(del);
                GPIO_PORTE_DATA_R =0b00000100;
                delayMs(del);
                GPIO_PORTE_DATA_R =0b00001000;
                delayMs(del);

            }
        }

    }

    UARTprintf("...........Welcome...............\n" );
    GPIO_PORTF_DATA_R =0b00000010;
    delayMs(2000);
    GPIO_PORTF_DATA_R =0b00000000;
}




