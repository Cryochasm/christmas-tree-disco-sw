/*
 * \file
 * led_driver_tlc5955.h
 *
 *  Created on: Sep 14, 2017
 *      Author: Levi
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
 * \param LED_WHITE
 *
 * \param LED_OFF
 *
 *
 */
//                                 | RED  |          | GREEN|          | BLUE |
#define COLOR_WHITE     ((uint16_t) 0xFFFF,(uint16_t) 0xFFFF,(uint16_t) OxFFFF)
#define COLOR_RED       ((uint16_t) 0xFFFF,(uint16_t) 0x0000,(uint16_t) Ox0000)
#define COLOR_ORANGE    ((uint16_t) 0xFFFF,(uint16_t) 0x7FFF,(uint16_t) Ox0000)
#define COLOR_YELLOW    ((uint16_t) 0xFFFF,(uint16_t) 0xFFFF,(uint16_t) Ox0000)
#define COLOR_GREEN     ((uint16_t) 0x0000,(uint16_t) 0xFFFF,(uint16_t) Ox0000)
#define COLOR_BLUE      ((uint16_t) 0x0000,(uint16_t) 0x0000,(uint16_t) 0xFFFF)
#define COLOR_CYAN      ((uint16_t) 0x0000,(uint16_t) 0xFFFF,(uint16_t) OxFFFF)
#define COLOR_MAGENTA   ((uint16_t) 0xFFFF,(uint16_t) 0x0000,(uint16_t) OxFFFF)
#define COLOR_OFF       ((uint16_t) 0x0000,(uint16_t) 0x0000,(uint16_t) Ox0000)

/*
 * \func
 * tlc5955_init();
 *
 * \brief
 * Initializes LED driver, sets max current limits, and sets dot correction.
 *
 */
void tlc5955_init (void);

/*
 * \func
 * set_led_color(led_id, red, green, blue);
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
 *                   (12)
 *                 (0)   (9)
 *              (5)   (4)   (13)
 *           (1)   (6)   (15)   (14)
 *        (2)   (3)   (7)    (11)   (10)
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
void set_led_color (uint8_t led_id,
                    uint16_t red,
                    uint16_t green,
                    uint16_t blue);

/*
 * \func
 * refresh_led();
 *
 * \brief
 * Sends
 *
 * \pre
 * set_led_color(led_id, red, green, blue)
 *
 *
 */
void refresh_led (void);

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
