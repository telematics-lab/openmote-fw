/**
 * @file       main.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "board.h"

#include "Callback.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void timer0b_callback(void);

static PlainCallback timer0bCallback(timer0b_callback);

/*=============================== variables =================================*/

uint16_t pwmDutyCycle = 0;
uint16_t pwmStep = 128;
bool pwmIsRising = true;

/*================================= public ==================================*/


int main (void)
{
    // Initialize board
    board.init();

    led_green.init(true);
    led_green.setFrequency(32768);

    led_red.init(true);
    led_red.setFrequency(32768);

    led_yellow.init(true);
    led_yellow.setFrequency(32768);

    led_orange.init(true);
    led_orange.setFrequency(32768);

    timer0b.init();
    timer0b.setFrequency(100);
    timer0b.setCallback(&timer0bCallback);
    timer0b.enableInterrupts();

    board.enableInterrupts();

    led_green.start();
    led_red.start();
    led_yellow.start();
    led_orange.start();

    led_green.setDutyCycle(pwmDutyCycle);  
    led_red.setDutyCycle(pwmDutyCycle);  
    led_yellow.setDutyCycle(pwmDutyCycle);  
    led_orange.setDutyCycle(pwmDutyCycle);  

    timer0b.start();

    while(true)
    {
        // Sleep
        board.sleep();
    }
}

/*================================ private ==================================*/

static void timer0b_callback(void)
{
    if (pwmDutyCycle < 129) {
        pwmIsRising = true;
    } else if (pwmDutyCycle > 32639) {
        pwmIsRising = false;
    }

    if (pwmIsRising == false) {
        pwmDutyCycle -= pwmStep;
    } else {
        pwmDutyCycle += pwmStep;
    }

    led_green.setDutyCycle(pwmDutyCycle);
    led_red.setDutyCycle(pwmDutyCycle);
    led_yellow.setDutyCycle(pwmDutyCycle);
    led_orange.setDutyCycle(pwmDutyCycle);
}
