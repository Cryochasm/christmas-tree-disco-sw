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


typedef struct LFSR_T
{
    uint16_t state;
    uint16_t taps;
    uint16_t ymask;
} lfsr_t;

bool lfsr_step(lfsr_t *plfsr);
void lfsr_init(lfsr_t *plfsr);

void delay_main (volatile uint32_t loop_count);


void set_all_leds (uint16_t red, uint16_t green, uint16_t blue);

void set_row(int8_t row, uint16_t red, uint16_t green, uint16_t blue);

uint16_t bounce_red (uint16_t color, uint16_t shift);
uint16_t bounce_green (uint16_t color, uint16_t shift);
uint16_t bounce_blue (uint16_t color, uint16_t shift);

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

#if 0 //Enable Color Test
    int8_t led_count;
    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_WHITE);
    }
    refresh_led();

    START_GSCLK;
    delay_main(0x009FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_RED);
    }
    refresh_led();
    delay_main(0x003FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_ORANGE);
    }
    refresh_led();
    delay_main(0x003FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_YELLOW);
    }
    refresh_led();
    delay_main(0x003FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_GREEN);
    }
    refresh_led();
    delay_main(0x003FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_BLUE);
    }
    refresh_led();
    delay_main(0x003FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_CYAN);
    }
    refresh_led();
    delay_main(0x003FFFFF);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        set_led_color(led_count, COLOR_MAGENTA);
    }
    refresh_led();
    delay_main(0x003FFFFF);
#else
    START_GSCLK;
#endif

    //
    //Main Loop
    //

    for (;;)
    {

        //
        //-------Cycle Up Rows--------------
        //
#define MAIN_LOOPS              ((uint32_t) 0x000002F)
        uint32_t main_counter = MAIN_LOOPS;
        int8_t temp;
#if 1//skip to random;
        while (main_counter != 5)
        {
            temp = main_counter % 5;
            set_row(temp, 0x0000, 0x0000, 0x0000);
            temp++;
            if (temp >= 5)
            {
                temp = 0;
            }
            set_row(temp, 0x08FF, 0x0000, 0x0000);
            temp++;
            if (temp >= 5)
            {
                temp = 0;
            }
            set_row(temp, COLOR_RED);
            temp++;
            if (temp >= 5)
            {
                temp = 0;
            }
            set_row(temp, 0x08FF, 0x0000, 0x000);
            temp++;
            if (temp >= 5)
            {
                temp = 0;
            }
            set_row(temp, 0x0000, 0x0000, 0x000);

            refresh_led();

            delay_main(0x0007FFFF);



            main_counter--;
        }


        //
        //--------------Snake Run-----------------
        //
        main_counter = 6;
        while (main_counter != 0)
        {
            int8_t index;

            if (main_counter < 2)
            {
                for (index = ARRAY_MAX; index > 0; index--)
                {
                    set_all_leds(0x0000, 0x0000, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 0)], 0xFFFF,
                                  0x8FFF, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 1)], 0x7FFF,
                                  0x1FFF, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 2)], 0x0FFF,
                                  0x08FF, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 3)], 0x0000,
                                  0x0000, 0x0000);
                    refresh_led();
                    delay_main(0x0002FFFF);
                }
            }
            else if (main_counter < 4)
            {
                for (index = ARRAY_MAX; index > 0; index--)
                {
                    set_all_leds(0x0000, 0x0000, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 0)], 0x0000,
                                  0x8FFF, 0xFFFF);
                    set_led_color(g_led_array[array_wrap(index, 1)], 0x0000,
                                  0x1FFF, 0x7FFF);
                    set_led_color(g_led_array[array_wrap(index, 2)], 0x0000,
                                  0x08FF, 0x0FFF);
                    set_led_color(g_led_array[array_wrap(index, 3)], 0x0000,
                                  0x0000, 0x00FF);
                    refresh_led();
                    delay_main(0x0002FFFF);
                }
            }
            else
            {
                for (index = ARRAY_MAX; index > 0; index--)
                {
                    set_all_leds(0x0000, 0x0000, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 0)], 0x0000,
                                  0xFFFF, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 1)], 0x0000,
                                  0x1FFF, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 2)], 0x0000,
                                  0x08FF, 0x0000);
                    set_led_color(g_led_array[array_wrap(index, 3)], 0x0000,
                                  0x00FF, 0x0000);
                    refresh_led();
                    delay_main(0x0002FFFF);
                }
            }

            main_counter--;
        }


        //
        //---------------Color Cycle------------------------------------
        //
        set_all_leds(0xFFFF, 0x0000, 0x0000);
        main_counter = 0x0001;

        while (main_counter != 0)
        {
            uint16_t red_count = 0xFFFF;
            uint16_t green_count = 0;
            uint16_t blue_count = 0;

            for (green_count = 0; green_count != 0xFFFF;
                    green_count = green_count + 0x0F)
            {
                set_all_leds(red_count, green_count, blue_count);
            }

            for (red_count = 0xFFFF; red_count != 0;
                    red_count = red_count - 0x0F)
            {
                set_all_leds(red_count, green_count, blue_count);
            }


            for (blue_count = 0; blue_count != 0xFFFF;
                    blue_count = blue_count + 0x0F)
            {
                set_all_leds(red_count, green_count, blue_count);
            }

            for (green_count = 0xFFFF; green_count != 0xFF;
                    green_count = green_count - 0x0F)
            {
                set_all_leds(red_count, green_count, blue_count);
            }


            for (red_count = 0; red_count != 0xFFFF;
                    red_count = red_count + 0x0F)
            {
                set_all_leds(red_count, green_count, blue_count);
            }

            for (blue_count = 0xFFFF; blue_count != 0xFF;
                    blue_count = blue_count - 0x0F)
            {
                set_all_leds(red_count, green_count, blue_count);
            }
            main_counter--;
        }
#endif
        //
        //----------Randomness-----------------
        //

        main_counter = 0x001FF;
        lfsr_t random;
        lfsr_init(&random);
        led_rgb_t color;

        color.red = (uint16_t) random.state;
        lfsr_step(&random);
        color.blue = (uint16_t) random.state;
        lfsr_step(&random);
        color.green = (uint16_t) random.state;
        lfsr_step(&random);

        while (main_counter != 0)
        {
            set_led_color(0, color.red, color.green, color.blue);
            color.red = 0;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(1, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = 0;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(2, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = 0;
            lfsr_step(&random);

            set_led_color(3, color.red, color.green, color.blue);
            color.red = 0;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(4, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = 0;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(5, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = 0;
            lfsr_step(&random);

            set_led_color(6, color.red, color.green, color.blue);
            color.red = 0;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(7, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = 0;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(9, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = 0;
            lfsr_step(&random);

            set_led_color(10, color.red, color.green, color.blue);
            color.red = 0;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(11, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = 0;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(12, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = 0;
            lfsr_step(&random);

            set_led_color(13, color.red, color.green, color.blue);
            color.red = 0;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(14, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = 0;
            lfsr_step(&random);
            color.green = (uint16_t) random.state;
            lfsr_step(&random);

            set_led_color(15, color.red, color.green, color.blue);
            color.red = (uint16_t) random.state;
            lfsr_step(&random);
            color.blue = (uint16_t) random.state;
            lfsr_step(&random);
            color.green = 0;
            lfsr_step(&random);

            refresh_led();
            delay_main(0x0008FFFF);

            main_counter--;
        }

    }

}

//
// ---------------------------Functions----------------------------------------
//


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
    }
    refresh_led();
}

void set_row(int8_t row, uint16_t red, uint16_t green, uint16_t blue)
{
    uint8_t col_count = 0;
    for (col_count = 0; col_count < COL_MAX; col_count++)
    {
        set_led_color(g_led_2d_array[row][col_count], red, green, blue);
    }
}

uint16_t bounce_red(uint16_t color, uint16_t shift)
{
    static bool direction = true;
    int32_t temp;

    if (direction == true)
    {
        temp = color + shift;

        if(temp > 0xFFFF)
        {
            temp = 0xFFFF;
            direction = false;
        }

    }
    else
    {
        temp = color - shift;

        if (temp < 0)
        {
            temp = 0;
            direction = true;
        }
    }
    return (uint16_t) temp;
}

uint16_t bounce_green(uint16_t color, uint16_t shift)
{
    static bool direction = true;
    int32_t temp;

    if (direction == true)
    {
        temp = color + shift;

        if(temp > 0xFFFF)
        {
            temp = 0xFFFF;
            direction = false;
        }

    }
    else
    {
        temp = color - shift;

        if (temp < 0)
        {
            temp = 0;
            direction = true;
        }
    }
    return (uint16_t) temp;
}

uint16_t bounce_blue(uint16_t color, uint16_t shift)
{
    static bool direction = true;
    int32_t temp;

    if (direction == true)
    {
        temp = color + shift;

        if(temp > 0xFFFF)
        {
            temp = 0xFFFF;
            direction = false;
        }

    }
    else
    {
        temp = color - shift;

        if (temp < 0)
        {
            temp = 0;
            direction = true;
        }
    }
    return (uint16_t) temp;
}




//
//----------------------------RANDOM NUMBER GENERATOR--------------------------
//



void lfsr_init(lfsr_t *plfsr)
{
    plfsr->state = 1;
    plfsr->taps  = 0x4050; // a0 = 1, a2 = 1
    plfsr->ymask = 0x0100; // bit 8 is the highest significant state bit
}

/* Galois implementation of LFSR */
bool lfsr_step(lfsr_t *plfsr)
{
    bool out = (plfsr->state & plfsr->ymask) != 0;
    plfsr->state <<= 1;    // shift left
    if (out)
    {
        plfsr->state ^= plfsr->taps;
        /* flip bits connected to the taps if the output is 1.
           Note that the state bits beyond bit 4 are ignored */
    }
    return out;
}




