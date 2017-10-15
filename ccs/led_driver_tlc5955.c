/*
 * \file led_driver_tlc5955.c
 *
 * \version 0.1
 * \date Sep 14, 2017
 * \author Levi
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"

/*
 * HAL for the EK-TM4C123GXL Launch Pad
 *
 * Pin connections
 * PB3 - SCLK
 * PC4 - GSCLK
 * PD6 - DATA
 * PD7 - LAT
 * PF4 - ERROR_RD
 */
#define PIN0                (0x1u)
#define PIN1                (0x1u << 1)
#define PIN2                (0x1u << 2)
#define PIN3                (0x1u << 3)
#define PIN4                (0x1u << 4)
#define PIN5                (0x1u << 5)
#define PIN6                (0x1u << 6)
#define PIN7                (0x1u << 7)

#define SCLK                (GPIO_PIN_3)  //PORT B
#define GSCLK               (GPIO_PIN_4)  //PORT C
#define DATA                (GPIO_PIN_6)  //PORT D
#define LAT                 (GPIO_PIN_7)  //PORT D
#define ERROR_RD            (GPIO_PIN_4)  //PORT F
#define USR_SW1             (GPIO_PIN_0)  //PORT F
#define LED_RED             (GPIO_PIN_1)  //PORT F
#define LED_BLUE            (GPIO_PIN_2)  //PORT F
#define LED_GREEN           (GPIO_PIN_3)  //PORT F
#define USR_SW2             (GPIO_PIN_4)  //PORT F

#define SCLK_HIGH       (GPIOPinWrite(GPIO_PORTB_AHB_BASE, SCLK, SCLK))
#define SCLK_LOW        (GPIOPinWrite(GPIO_PORTB_AHB_BASE, SCLK, 0x00))
#define GSCLK_HIGH      (GPIOPinWrite(GPIO_PORTC_AHB_BASE, GSCLK, GSCLK))
#define GSCLK_LOW       (GPIOPinWrite(GPIO_PORTC_AHB_BASE, GSCLK, 0x00))
#define DATA_HIGH       (GPIOPinWrite(GPIO_PORTD_AHB_BASE, DATA, DATA))
#define DATA_LOW        (GPIOPinWrite(GPIO_PORTD_AHB_BASE, DATA, 0x00))
#define LAT_HIGH        (GPIOPinWrite(GPIO_PORTD_AHB_BASE, LAT, LAT))
#define LAT_LOW         (GPIOPinWrite(GPIO_PORTD_AHB_BASE, LAT, 0x00))
#define RED_LED_ON      (GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_RED, LED_RED))
#define RED_LED_OFF     (GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_RED, 0x00))
#define BLUE_LED_ON     (GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_BLUE, LED_BLUE))
#define BLUE_LED_OFF    (GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_BLUE, 0x00))
#define GREEN_LED_ON    (GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_GREEN, LED_GREEN))
#define GREEN_LED_OFF   (GPIOPinWrite(GPIO_PORTF_AHB_BASE, LED_GREEN, 0x00))

//HAL function prototypes
void init_gpio(void);
void init_pwm(void);

#define DELAY_TIME                  ((uint32_t) 500000u)
#define MAX_COLOR_CORRECTION        ((uint8_t) 0x7Fu)
#define MAX_RED_CURRENT             ((uint8_t) 0x00u)
#define MAX_GREEN_CURRENT            ((uint8_t) 0x00u)
#define MAX_BLUE_CURRENT            ((uint8_t) 0x00u)

#define NDEBUG

#ifndef NDEBUG
#define SERIAL_DELAY            ((uint32_t) 0x00000010)
#endif

//
//private function prototypes
//
void delay (volatile uint32_t loop_count);
void serial_out_1bit (uint8_t data);
void serial_out_3bit (uint8_t data);
void serial_out_7bit (uint8_t data);
void serial_out_8bit (uint8_t data);
void serial_out_16bit (uint16_t data);

//
//typedefs
//
//
//Base Structs
//

//#pragma pack (1) //force no filler bits in the structs so that it is easier to output serially

/**
 * \typedef gs_led_t
 * \struct gs_led_t
 *
 * \var red
 * Sets the duty cylce for the grayscale PWM module.
 *
 * \var green
 * Sets the duty cylce for the grayscale PWM module.
 *
 * \var blue
 * Sets the duty cylce for the grayscale PWM module.
 *
 */
typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} gs_led_t;

/**
 * \typedef dc_led_t
 * \struct dc_led_t
 *
 * \var red
 * Reduces max brightness for red on an individual LED.
 *
 * \var green
 * Reduces max brightness for green on an individual LED.
 *
 * \var blue
 * Reduces max brightness for blue on an individual LED.
 *
 * 0x7F = 100%
 * ...
 * 0x00 = 26.2%
 */
typedef struct
{
    uint8_t red :7;
    uint8_t green :7;
    uint8_t blue :7;
} dc_led_t;

/**
 * \struct funct_t
 * Contains the function registers
 * \typedef funct_t
 * 
 * \var dsprpt
 * Auto display repeat mode enable bit
 * 0 = Disabled, 1 = Enabled
 * When this bit is 0, the auto display repeat function is disabled. Each
 * constant-current output is turned on and off for one display period.
 * When this bit is 1, each output repeats the PWM control every 65,536 GSCLKs.
 * 
 * \var tmgrst
 * Display timing reset mode enable bit
 * 0 = Disabled, 1 = Enabled
 * When this bit is 0, the GS counter is not reset and the outputs are not 
 * forced off even when a LAT rising edge is input for a GS data write.
 * When this bit is 1, the GS counter is reset to 0 and all outputs are forced off at the
 * LAT rising edge for a GS data write. Afterwards, PWM control resumes from the next
 * GSCLK rising edge.
 *
 * \var rfresh
 * Auto data refresh mode enable bit
 * 0 = Disabled, 1 = Enabled
 * When this bit is 0, the auto data refresh function is disabled. The data in the common
 * shift register are copied to the GS data latch at the next LAT rising edge for a GS data write. DC data in the control data latch are copied to the DC data latch at the same time. 
 * When this bit is 1, the auto data refresh function is enabled. The data in the common
 * shift register are copied to the GS data latch at the 65,536th GSCLK after the LAT
 * rising edge for a GS data write. DC data in the control data latch are copied to the
 * DC data latch at the same time.
 *
 * \var espwm
 * ES-PWM mode enable bit
 * 0 = Disabled, 1 = Enabled
 * When this bit is 0, the conventional PWM control mode is selected. If the TLC5955 is used for multiplexing a drive, the conventional PWM mode should be selected to
 * prevent excess on or off switching.
 * When this bit is 1, ES-PWM control mode is selected.
 *
 * \var lsdvlt
 * LSD detection voltage selection bit
 * LED short detection (LSD) detects a fault caused by a shorted LED by comparing the
 * OUTXn voltage to the LSD detection threshold voltage. The threshold voltage is selected by this bit.
 * When this bit is 0, the LSD voltage is VCC × 70%. When this bit is 1, the LSD voltage
 * is VCC × 90%.
 *
 */
typedef struct
{
    uint8_t dsprpt :1;
    uint8_t tmgrst :1;
    uint8_t rfresh :1;
    uint8_t espwm :1;
    uint8_t lsdvlt :1;
} func_t;

/**
 * \struct maximum_current_t
 * \typedef maximum_current_t
 * \var red_i maximum drive current for the red LED
 * \var green_i maximum drive current for the green LED
 * \var blue_i maximum drive current for the blue LED
 *
 * 0x00 =  3.2 mA default
 * 0x01 =  8.0 mA
 * 0x02 = 11.2 mA
 * 0x03 = 15.9 mA
 * 0x04 = 19.1 mA
 * 0x05 = 23.9 mA
 * \warning hardware does not support these values
 * 0x06 = 27.1 mA VCC is not high enough to use this
 * 0x07 = 31.9 mA VCC is not high enough to use this
 */
typedef struct
{
    uint8_t red_i :3;
    uint8_t green_i :3;
    uint8_t blue_i :3;
} maximum_current_t;

//
//Super Structs
//

/**
 * \struct gs_common_latch_t
 * Grayscale Latch
 *
 * \var msb
 * 0 for grayscale latch
 *
 * \var led_gs
 * 16-bit PWM brightness for each color and each LED.
 *
 */
typedef struct GS_COMMON_LATCH_T
{
    uint32_t msb :1;
    gs_led_t led_gs[16];
} gs_common_latch_t; //768 bits long + 1 bit for latch

/**
 * \struct control_common_latch_t
 * Control latch + the Dot Correction Latch.
 * 
 *\var msb
 * Equals 1 for control data.
 *
 *\var key
 * Allows writing to the control data registers.
 *
 *\var fc 
 * Function Control
 *
 *\var bc
 * Brightness for individual colors for all LEDs.
 *
 *\var mc
 * Maximum current limits
 *
 *\var led_dc[16]
 * Brightness for individual colors of individual LEDs.
 *
 *
 */
typedef struct CONTROL_COMMON_LATCH_T
{
    uint32_t msb :1;
    uint8_t key;
    uint8_t dummy_byte[48];
    uint8_t dummy_bit :6;
    func_t fc;
    dc_led_t bc;
    maximum_current_t mc;
    dc_led_t led_dc[16];
} control_common_latch_t;

//
//Global Variables
//
//! \var g_latch Grayscale latch.
static volatile gs_common_latch_t g_grayscale_latch;
static volatile control_common_latch_t g_control_latch;

extern const uint8_t g_led_2d_array[5][8] = {
        { 12, 12, 12, 12, 12, 12, 12, 12 }, { 0, 0, 0, 0, 9, 9, 9, 9 },
        { 5, 5, 5, 4, 4, 13, 13, 13 }, { 1, 6, 6, 6, 15, 15, 15, 14 },
        { 2, 3, 3, 7, 7, 11, 11, 10 }, };

//
//Public Functions
//

/**
 * /fn tlc5955_init
 * /brief
 * Loads the control_common_latch and then outputs the contents of the 
 * control_common_latch to the serial port.
 */
void tlc5955_init(void)
{

    init_gpio();
    init_pwm();

    //declare structure for initialization.
    g_control_latch.msb = 1;
    g_control_latch.key = 0x96;

    //load g_control_latch

    g_control_latch.fc.dsprpt = 1;
    g_control_latch.fc.tmgrst = 0;
    g_control_latch.fc.rfresh = 0;
    g_control_latch.fc.espwm = 1;
    g_control_latch.fc.lsdvlt = 1;

    g_control_latch.bc.red = MAX_COLOR_CORRECTION;
    g_control_latch.bc.green = MAX_COLOR_CORRECTION;
    g_control_latch.bc.blue = MAX_COLOR_CORRECTION;

    g_control_latch.mc.red_i = MAX_RED_CURRENT;
    g_control_latch.mc.blue_i = MAX_BLUE_CURRENT;
    g_control_latch.mc.green_i = MAX_GREEN_CURRENT;

    g_control_latch.led_dc[0].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[0].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[0].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[1].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[1].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[1].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[2].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[2].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[2].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[3].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[3].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[3].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[4].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[4].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[4].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[5].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[5].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[5].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[6].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[6].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[6].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[7].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[7].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[7].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[8].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[8].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[8].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[9].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[9].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[9].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[10].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[10].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[10].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[11].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[11].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[11].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[12].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[12].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[12].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[13].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[13].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[13].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[14].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[14].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[14].blue = MAX_COLOR_CORRECTION;

    g_control_latch.led_dc[15].red = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[15].green = MAX_COLOR_CORRECTION;
    g_control_latch.led_dc[15].blue = MAX_COLOR_CORRECTION;

    //
    //Output g_control_latch to serial
    //
    serial_out_1bit(g_control_latch.msb);
    serial_out_8bit(g_control_latch.key);
    uint16_t dummy_count;
    for (dummy_count = 0u; dummy_count < 389u; dummy_count++)
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }
    serial_out_1bit(g_control_latch.fc.lsdvlt);
    serial_out_1bit(g_control_latch.fc.espwm);
    serial_out_1bit(g_control_latch.fc.rfresh);
    serial_out_1bit(g_control_latch.fc.tmgrst);
    serial_out_1bit(g_control_latch.fc.dsprpt);
    serial_out_7bit(g_control_latch.bc.blue);
    serial_out_7bit(g_control_latch.bc.green);
    serial_out_7bit(g_control_latch.bc.red);
    serial_out_3bit(g_control_latch.mc.blue_i);
    serial_out_3bit(g_control_latch.mc.green_i);
    serial_out_3bit(g_control_latch.mc.red_i);

    int8_t led_count;
    for (led_count = 15u; led_count >= 0; led_count--)
    {
        serial_out_7bit(g_control_latch.led_dc[led_count].blue);
        serial_out_7bit(g_control_latch.led_dc[led_count].green);
        serial_out_7bit(g_control_latch.led_dc[led_count].red);
    }

    LAT_HIGH;
#ifndef NDEBUG
    delay(SERIAL_DELAY);
#endif
    LAT_LOW;
#ifndef NDEBUG
    delay(SERIAL_DELAY);
#endif

    //output g_control_latch twice to lock in the maximum current
    serial_out_1bit(g_control_latch.msb);
    serial_out_8bit(g_control_latch.key);
    for (dummy_count = 0u; dummy_count < 389u; dummy_count++)
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }
    serial_out_1bit(g_control_latch.fc.lsdvlt);
    serial_out_1bit(g_control_latch.fc.espwm);
    serial_out_1bit(g_control_latch.fc.rfresh);
    serial_out_1bit(g_control_latch.fc.tmgrst);
    serial_out_1bit(g_control_latch.fc.dsprpt);
    serial_out_7bit(g_control_latch.bc.blue);
    serial_out_7bit(g_control_latch.bc.green);
    serial_out_7bit(g_control_latch.bc.red);
    serial_out_3bit(g_control_latch.mc.blue_i);
    serial_out_3bit(g_control_latch.mc.green_i);
    serial_out_3bit(g_control_latch.mc.red_i);

    for (led_count = 15u; led_count >= 0; led_count--)
    {
        serial_out_7bit(g_control_latch.led_dc[led_count].blue);
        serial_out_7bit(g_control_latch.led_dc[led_count].green);
        serial_out_7bit(g_control_latch.led_dc[led_count].red);
    }

    LAT_HIGH;
#ifndef NDEBUG
    delay(SERIAL_DELAY);
#endif
    LAT_LOW;
#ifndef NDEBUG
    delay(SERIAL_DELAY);
#endif
}

void set_led_color(uint8_t led_id, uint16_t red, uint16_t green, uint16_t blue)
{
    g_grayscale_latch.led_gs[led_id].red = red;
    g_grayscale_latch.led_gs[led_id].green = green;
    g_grayscale_latch.led_gs[led_id].blue = blue;
}

/**
 * \fn refesh_led
 * \brief
 * Outputs the contents of the g_grayscale_latch to the serial port and then latches the
 * data into the LED driver's internal buffers 
 *
 *
 */
void refresh_led(void)
{
    serial_out_1bit(g_grayscale_latch.msb);
    int8_t led_count;
    for (led_count = 15u; led_count >= 0; led_count--)
    {
        serial_out_16bit(g_grayscale_latch.led_gs[led_count].blue);
        serial_out_16bit(g_grayscale_latch.led_gs[led_count].green);
        serial_out_16bit(g_grayscale_latch.led_gs[led_count].red);
    }
    LAT_HIGH;
#ifndef NDEBUG
    delay(SERIAL_DELAY);
#endif
    LAT_LOW;
#ifndef NDEBUG
    delay(SERIAL_DELAY);
#endif
}

void test_tlc5955(void)
{

}

/**
 * \fn init_gpio
 * \brief
 * A part of the HAL. Initializes the GPIOs used to communicate with the LED
 * driver.
 */
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
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTC_AHB_BASE, GSCLK);
    HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_CR) = 0xFF;
    GPIOPinTypeGPIOOutput(GPIO_PORTD_AHB_BASE, 0xFF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_AHB_BASE, LED_RED | LED_BLUE | LED_GREEN);

    SCLK_LOW;
    GSCLK_LOW;
    DATA_LOW;
    LAT_LOW;

}

/**
 * \fn init_pwm
 * Sets up pwm module for GSCLK on Port C Pin 4
 * 50% duty cycle with 33 MHz maximum speed.
 * 4 MHz gives roughly 60Hz refresh rate on the LEDs
 *
 */
void init_pwm(void)
{
    //
    // Enable the PWM6 peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Wait for the PWM0 module to be ready.
    //
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }
    //
    // Configure the PWM generator for count down mode with immediate updates
    // to the parameters.
    //
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    //
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
    // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
    // Use this value to set the period.
    //
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 1000);
    //
    // Set the pulse width of PWM0 for a 50% duty cycle.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 500);

    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

}

//Private Functions

/**
 * \fn delay(volatile uint32_t loop_count) 
 * Simple delay function.
 */
void delay(volatile uint32_t loop_count)
{
    while (0u != loop_count)
    {
        loop_count--;    //do nothing
    }
}

/**
 * \fn serial_out_1bit(uint8_t data)
 * Outputs 1 bit onto the serial bus.
 */
void serial_out_1bit(uint8_t data)
{
    if ((data & 0x01) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }
}

void serial_out_3bit(uint8_t data)
{

    if ((data & 0x04) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x02) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x01) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }
}

void serial_out_7bit(uint8_t data)
{
    if ((data & 0x40) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x20) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x10) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x08) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x04) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x02) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x01) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

}

void serial_out_8bit(uint8_t data)
{
    if ((data & 0x80) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x40) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x20) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x10) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x08) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x04) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x02) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }

    if ((data & 0x01) != 0u)
    {

        DATA_HIGH;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif

    }
    else
    {
        DATA_LOW;
        SCLK_HIGH;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
        SCLK_LOW;
#ifndef NDEBUG
        delay(SERIAL_DELAY);
#endif
    }
}

void serial_out_16bit(uint16_t data)
{
    uint8_t temp_l = ((data >> 8) & 0x00FF);
    uint8_t temp_h = ((uint8_t) data) & 0x00FF;
    serial_out_8bit((uint8_t) temp_h);
    serial_out_8bit((uint8_t) temp_l);
}

