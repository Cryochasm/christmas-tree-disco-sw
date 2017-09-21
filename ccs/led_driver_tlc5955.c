/*
 * \file
 * led_driver_tlc5955.c
 *
 *  Created on: Sep 14, 2017
 *      Author: Levi
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

//HAL function prototypes
void init_gpio(void);
void sclk_high(void);
void sclk_low(void);
void gsclk_high(void);
void gsclk_low(void);
void data_high(void);
void data_low(void);
void lat_high(void);
void lat_low(void);
void red_led_on(void);
void red_led_off(void);
void blue_led_on(void);
void blue_led_off(void);
void green_led_on(void);
void green_led_off(void);

#define DELAY_TIME      ((uint32_t) 5000000)

//private function prototypes
void delay (uint32_t loop_count);

//typedef
typedef struct
{
    uint8_t MSB :1;

} grayscale_latch_t;

typedef struct
{
    uint8_t MSB :1;
} control_latch_t;

typedef struct
{
    uint8_t MSB :1;
} dot_correction_latch_t;

typedef struct
{
    uint8_t MSB :1;
} common_latch_t;

//Public Functions
void tlc5955_init(void)
{
    init_gpio();

    red_led_on();

    delay(DELAY_TIME);

    blue_led_on();

    delay(DELAY_TIME);

    green_led_on();

    delay(DELAY_TIME);
    delay(DELAY_TIME);

    red_led_off();
    blue_led_off();
    green_led_off();
}

void set_led_color(uint8_t led_id, uint16_t red, uint16_t green, uint16_t blue)
{

}

void refresh_led(void)
{

}

void test_tlc5955(void)
{

    sclk_high();

    sclk_low();

    gsclk_high();

    gsclk_low();

    data_high();

    data_low();

    lat_high();

    lat_low();

}

/*
 * HAL for the EK-TM4C123GXL Launch Pad
 * General-Purpose Input/Output Run Mode Clock Gating Control (RCGCGPIO)
 * Base             Offset
 * 0x400F.E000      0x608
 * Type RW, reset 0x0000.0000
 *
 * Pin connections      AHB Memory Base Offset
 * PB3 - SCLK           0x4005.9000
 * PC4 - GSCLK          0x4005.A000
 * PD6 - DATA           0x4005.B000
 * PD7 - LAT            0x4005.B000
 * PF4 - ERROR_RD       0x4005.D000
 */
#define PIN0                (0x1u)
#define PIN1                (0x1u << 1)
#define PIN2                (0x1u << 2)
#define PIN3                (0x1u << 3)
#define PIN4                (0x1u << 4)
#define PIN5                (0x1u << 5)
#define PIN6                (0x1u << 6)
#define PIN7                (0x1u << 7)

#define SCLK                (0x1u << 3)  //PORT B
#define GSCLK               (0x1u << 4)  //PORT C
#define DATA                (0x1u << 6)  //PORT D
#define LAT                 (0x1u << 7)  //PORT D
#define ERROR_RD            (0x1u << 4)  //PORT F
#define USR_SW1             (0x1u     )  //PORT F
#define LED_RED             (0x1u << 1)  //PORT F
#define LED_BLUE            (0x1u << 2)  //PORT F
#define LED_GREEN           (0x1u << 3)  //PORT F
#define USR_SW2             (0x1u << 4)  //PORT F

#define GPIO_O_LOCK             0x00000520  // GPIO Lock
#define GPIO_O_CR               0x00000524  // GPIO Commit
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register


void init_gpio(void)
{

    //
    // Enable and wait for the port to be ready for access
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable AHB Bus
    //
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Configure the GPIO port for the LED operation.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_AHB_BASE, SCLK);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_AHB_BASE, GSCLK);
    HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_CR) = 0xFF;
    GPIOPinTypeGPIOOutput(GPIO_PORTD_AHB_BASE, 0xff);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_AHB_BASE, LED_RED | LED_BLUE | LED_GREEN);

}


void sclk_high(void)
{
    GPIOPinWrite(GPIO_PORTB_AHB_BASE, SCLK, SCLK);
}

void sclk_low(void)
{
    GPIOPinWrite(GPIO_PORTB_AHB_BASE, SCLK, 0x00);
}

void gsclk_high(void)
{
    GPIOPinWrite(GPIO_PORTC_AHB_BASE, GSCLK, GSCLK);
}

void gsclk_low(void)
{
    GPIOPinWrite(GPIO_PORTC_AHB_BASE, GSCLK, 0x00);
}

void data_high(void)
{
    GPIOPinWrite(GPIO_PORTD_AHB_BASE, DATA, DATA);
}

void data_low(void)
{
    GPIOPinWrite(GPIO_PORTD_AHB_BASE, DATA, 0x00);
}

void lat_high(void)
{
    GPIOPinWrite(GPIO_PORTD_AHB_BASE, LAT, LAT);
}

void lat_low(void)
{
    GPIOPinWrite(GPIO_PORTD_AHB_BASE, LAT, 0x00);
}

void red_led_on(void)
{
    GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_RED, LED_RED);
}

void red_led_off(void)
{
    GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_RED, 0x00);
}

void blue_led_on(void)
{
    GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_BLUE, LED_BLUE);
}

void blue_led_off(void)
{
    GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_BLUE, 0x00);
}

void green_led_on(void)
{
    GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_GREEN, LED_GREEN);
}

void green_led_off(void)
{
    GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_GREEN, 0x00);
}

//Private Functions

void delay (uint32_t loop_count)
{
    while(0u != loop_count)
    {
        loop_count--;//do nothing
    }
}

