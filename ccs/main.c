
/**
 * main.c
 */



#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "led_driver_tlc5955.h"


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

#if 1
void delay_main(volatile uint32_t loop_count);
#endif

//*****************************************************************************
//
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//
//*****************************************************************************
void main(void)
{
    //
    // Setup the system clock to run at 50 Mhz from PLL with crystal reference
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
    SYSCTL_OSC_MAIN);

    tlc5955_init();

    int8_t led_count;
    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_WHITE);
    }
    refresh_led();
    START_GSCLK;

    //
    //Main Loop
    //
    for (;;)
    {

        static uint8_t led_count = 0;
        static uint16_t color_count;

        set_led_color(((led_count + 0) % 15), COLOR_WHITE);
        set_led_color(((led_count + 1) % 15), COLOR_RED);
        set_led_color(((led_count + 2) % 15), COLOR_ORANGE);
        set_led_color(((led_count + 3) % 15), COLOR_YELLOW);
        set_led_color(((led_count + 4) % 15), COLOR_GREEN);
        set_led_color(((led_count + 5) % 15), COLOR_BLUE);
        set_led_color(((led_count + 6) % 15), COLOR_CYAN);
        set_led_color(((led_count + 7) % 15), COLOR_MAGENTA);
        set_led_color(((led_count + 8) % 15), COLOR_OFF);
        set_led_color(((led_count + 9) % 15), COLOR_RED);
        set_led_color(((led_count + 10) % 15), COLOR_ORANGE);
        set_led_color(((led_count + 11) % 15), COLOR_GREEN);
        set_led_color(((led_count + 12) % 15), COLOR_RED);
        set_led_color(((led_count + 13) % 15), COLOR_ORANGE);
        set_led_color(((led_count + 14) % 15), COLOR_GREEN);
        set_led_color(((led_count + 15) % 15), COLOR_YELLOW);

        led_count++;
        refresh_led();
        delay_main(0x00001FFF);
    }
}

#if 1
/**
 * \fn delay(volatile uint32_t loop_count)
 * Simple delay function.
 */
void delay_main(volatile uint32_t loop_count)
{
    while (0u != loop_count)
    {
        loop_count--;    //do nothing
    }
}

#endif

