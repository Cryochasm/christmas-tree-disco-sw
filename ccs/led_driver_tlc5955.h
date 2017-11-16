/*
 * \file led_driver_tlc5955.h
 *
 * \version 0.1
 * \date Sep 14, 2017
 * \author  Levi
 */
#include <stdint.h>

#ifndef LED_DRIVER_TLC5955_H_
#define LED_DRIVER_TLC5955_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

/*
 * \def LED_WHITE
 *
 * \def LED_OFF
 *
 *
 */
//                                 | RED  |          | GREEN|          | BLUE |
#define COLOR_WHITE     (uint16_t) 0xFFFF,(uint16_t) 0xFFFF,(uint16_t) 0xFFFF
#define COLOR_RED       (uint16_t) 0xFFFF,(uint16_t) 0x0000,(uint16_t) 0x0000
#define COLOR_ORANGE    (uint16_t) 0xFFFF,(uint16_t) 0x7FFF,(uint16_t) 0x0000
#define COLOR_YELLOW    (uint16_t) 0xFFFF,(uint16_t) 0xFFFF,(uint16_t) 0x0000
#define COLOR_GREEN     (uint16_t) 0x0000,(uint16_t) 0xFFFF,(uint16_t) 0x0000
#define COLOR_BLUE      (uint16_t) 0x0000,(uint16_t) 0x0000,(uint16_t) 0xFFFF
#define COLOR_CYAN      (uint16_t) 0x0000,(uint16_t) 0xFFFF,(uint16_t) 0xFFFF
#define COLOR_MAGENTA   (uint16_t) 0xFFFF,(uint16_t) 0x0000,(uint16_t) 0xFFFF
#define COLOR_OFF       (uint16_t) 0x0000,(uint16_t) 0x0000,(uint16_t) 0x0000


#define COL_MAX     ((int8_t) 8)
#define ROW_MAX     ((int8_t) 5)
extern const int8_t g_led_2d_array[ROW_MAX][COL_MAX];

#define ARRAY_MAX ((int8_t) 15)
extern const int8_t g_led_array[ARRAY_MAX];

/**
 * \typedef led_rgb_t
 * \struct led_rgb_t
 *
 * \var red
 * Sets the duty cycle for the grayscale PWM module.
 *
 * \var green
 * Sets the duty cycle for the grayscale PWM module.
 *
 * \var blue
 * Sets the duty cycle for the grayscale PWM module.
 *
 */
typedef struct LED_RGB_T
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} led_rgb_t;

/**
 * \typedef led_hsv_t
 * \struct led_hsv_t
 *
 * \var hue
 * Sets the hue for the color ranges from 0-360 degrees.
 *
 * \var sat
 * Sets the saturation for the color.
 *
 * \var value
 * Sets the value for the color.
 *
 */
typedef struct LED_HSV_T
{
    uint16_t hue;
    uint16_t sat;
    uint16_t value;
} led_hsv_t;


/*
 * \fn tlc5955_init();
 *
 * \brief
 * Initializes LED driver, sets max current limits, and sets dot correction.
 *
 */
void tlc5955_init (void);

/*
 * \fn set_led_color(led_id, red, green, blue);
 *
 * \brief
 *
 * \pre
 * tlc5955_init
 *
 * \param led_id
 * The id number of the led being controlled.
 * Valid id numbers are 0,1,2,3,4,5,6,7,9,10,11,12,13,14,15 (0-15 minus LED 8)
 * The locations of the LEDs are as follows:
 * \code
 *               (12)
 *             (0)  (9)
 *          (13)  (4)  (5)
 *       (14)  (15)  (6)  (1)
 *    (10)  (11)   (7)  (3)  (2)
 * \endcode
 *
 * \param red
 * Amount of red light. Ranges from 0x0000 - 0xFFFF.
 *
 * \param green
 * Amount of green light. Ranges from 0x0000 - 0xFFFF.
 *
 * \param blue
 * Amount of blue light. Ranges from 0x0000 - 0xFFFF.
 *
 */
void set_led_color (int8_t led_id,
                    uint16_t red,
                    uint16_t green,
                    uint16_t blue);

/**
 * \fn get_led_color
 *
 * \brief
 * Reads the 16-bit R,G,B value of the selected led.
 *
 * \param led_id
 * Led ID number.
 *
 * \return
 * led_rgb_t containing the color RGB color information.
 */
led_rgb_t get_led_color (int8_t led_id);




/**
 * \fn set_led_hsv
 *
 * \brief
 * Sets led color by using hue, saturation, and value.
 *
 * \param led_id
 * Led ID number.
 *
 * \param color
 * Pointer to the led_hsv_t struct containing the Hue, Saturation, and Value.
 *
 */
void set_led_hsv (int8_t led_id, led_hsv_t* color);


/**
 * \fn get_led_hsv
 *
 * \brief
 * Get led color by reading hue, saturation, and value.
 *
 * \param led_id
 * Led ID number
 *
 * \return
 * led_hsv_t struct containing the Hue, Saturation, and Value.
 *
 */
led_hsv_t get_led_hsv (int8_t led_id);



/**
 * \def Starts PWM for gsclk
 */
#define START_GSCLK         (PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true))

/**
 * \def Stops PWM for gsclk
 */
#define STOP_GSCLK          (PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false))

/*
 * \fn refresh_led();
 *
 * \brief
 * Outputs the contents of the g_grayscale_latch to the serial port and then latches the
 * data into the LED driver's internal buffers
 *
 * \pre
 * set_led_color(led_id, red, green, blue)
 *
 *
 */
void refresh_led (void);


/**
 * \fn col_wrap
 *
 * \brief
 *
 *
 * \param col
 *
 *
 * \param shift
 *
 * \returns
 * A value in the bounds of the array.
 */
int8_t col_wrap(int8_t col, int8_t shift);

/**
 * \fn row_wrap
 *
 *
 * \returns
 * A value in the bounds of the array.
 */
int8_t row_wrap(int8_t row, int8_t shift);

/**
 * \fn array_wrap
 *
 *
 * \returns
 * A value in the bounds of the array.
 */
int8_t array_wrap(int8_t index, int8_t shift);

/*
 * Public wrapper for private functions for debug use.
 */
void test_tlc5955 (void);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif


#endif /* LED_DRIVER_TLC5955_H_ */
