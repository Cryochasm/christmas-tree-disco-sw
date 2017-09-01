/**
 * \file
 * main.c
 *
 * \brief
 * main
 *
 * \author
 * Levi Hancock www.github.com/cryochasm
 *
 */
#include <stdint.h>
#include <stdbool.h>


/**
 * main.c
 */
void main(void)
{
    uint32_t counter = 0xDEADBEEF;
   for (;;)
   {
        while (counter != 0)
        {
            counter--;
        }
    }
}
