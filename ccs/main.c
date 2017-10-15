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


void delay_main (volatile uint32_t loop_count);


void set_all_leds (uint16_t red, uint16_t green, uint16_t blue);

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

#if 0
    int8_t led_count;
    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_WHITE);
    }
    refresh_led();
    START_GSCLK;
    delay_main(0x005FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_RED);
    }
    refresh_led();
    delay_main(0x005FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_GREEN);
    }
    refresh_led();
    delay_main(0x005FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_BLUE);
    }
    refresh_led();
    delay_main(0x005FFFFF);
#else
    START_GSCLK;
#endif

    //
    //Main Loop
    //

#if 0
    for (;;)
    {

        static uint8_t led_count = 0;
        static uint16_t color_count;

        set_led_color(((led_count + 0) % 16), COLOR_RED);
        set_led_color(((led_count + 1) % 16), COLOR_GREEN);
        set_led_color(((led_count + 2) % 16), COLOR_BLUE);
        set_led_color(((led_count + 3) % 16), COLOR_WHITE);
        refresh_led();
        delay_main(0x000FFFFF);


        set_led_color(((led_count + 0) % 16), COLOR_OFF);
        set_led_color(((led_count + 1) % 16), COLOR_OFF);
        set_led_color(((led_count + 2) % 16), COLOR_OFF);
        set_led_color(((led_count + 3) % 16), COLOR_OFF);
        set_led_color(((led_count + 4) % 16), COLOR_OFF);
        set_led_color(((led_count + 5) % 16), COLOR_OFF);
        set_led_color(((led_count + 6) % 16), COLOR_OFF);
        set_led_color(((led_count + 7) % 16), COLOR_OFF);
        set_led_color(((led_count + 8) % 16), COLOR_OFF);
        set_led_color(((led_count + 9) % 16), COLOR_OFF);
        set_led_color(((led_count + 10) % 16), COLOR_OFF);
        set_led_color(((led_count + 11) % 16), COLOR_OFF);
        set_led_color(((led_count + 12) % 16), COLOR_OFF);
        set_led_color(((led_count + 13) % 16), COLOR_OFF);
        set_led_color(((led_count + 14) % 16), COLOR_OFF);
        set_led_color(((led_count + 15) % 16), COLOR_OFF);
        refresh_led();
        delay_main(0x000FFFFF);
        led_count++;


    }

#else
    for (;;)
    {
        uint16_t red_count = 0;
        uint16_t green_count = 0;
        uint16_t blue_count = 0;

        for (red_count = 0; red_count != 0xFFFF; red_count = red_count + 0xFF)
        {
            set_all_leds(red_count, green_count, blue_count);
        }
        for (green_count = 0; green_count != 0xFFFF; green_count = green_count + 0xFF)
        {
            set_all_leds(red_count, green_count, blue_count);
        }
        for (red_count = 0xFFFF; red_count != 0; red_count = red_count - 0xFF)
        {
            set_all_leds(red_count, green_count, blue_count);
        }
        for (blue_count = 0; blue_count != 0xFFFF; blue_count = blue_count + 0xFF)
        {
            set_all_leds(red_count, green_count, blue_count);
        }
        for (green_count = 0xFFFF; green_count != 0xFF; green_count = green_count - 0xFF)
        {
            set_all_leds(red_count, green_count, blue_count);
        }
        for (blue_count = 0xFFFF; blue_count != 0xFF; blue_count = blue_count - 0xFF)
        {
            set_all_leds(red_count, green_count, blue_count);
        }
    }
#endif
}


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

void set_all_leds(uint16_t red, uint16_t green, uint16_t blue)
{
    int16_t led_count;

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, red, green, blue);
        refresh_led();
        // delay_main(0x0000001);
    }
}



