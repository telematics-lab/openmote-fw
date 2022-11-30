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
#include "Scheduler.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static void timer0_callback(void);
static void timer1_callback(void);
static void timer2_callback(void);
static void timer3_callback(void);

static PlainCallback timerCallback0(timer0_callback);
static PlainCallback timerCallback1(timer1_callback);
static PlainCallback timerCallback2(timer2_callback);
static PlainCallback timerCallback3(timer3_callback);

/*=============================== variables =================================*/

/*================================= public ==================================*/

int main (void)
{
    // Initialize board
    board.init();

    // Initialize Timer0
    timer0b.init();
    timer0b.setFrequency(2);
    timer0b.setCallback(&timerCallback0);
    timer0b.enableInterrupts();

    // Initialize Timer1
    timer1b.init();
    timer1b.setFrequency(4);
    timer1b.setCallback(&timerCallback1);
    timer1b.enableInterrupts();

    // Initialize Timer2
    timer2b.init();
    timer2b.setPrescaler(255);
    timer2b.setFrequency(8);
    timer2b.setCallback(&timerCallback2);
    timer2b.enableInterrupts();

    // Initialize Timer3
    timer3b.init();
    timer3b.setFrequency(16);
    timer3b.setCallback(&timerCallback3);
    timer3b.enableInterrupts();

    // Start Timer0, 1, 2 and 3
    timer0b.start();
    timer1b.start();
    timer2b.start();
    timer3b.start();
    
    // Enable interrupts
    board.enableInterrupts();

    // Forever
    while(true)
    {
        // Sleep
        board.sleep();
    }
}

/*=============================== protected =================================*/

/*================================ private ==================================*/

static void timer0_callback(void)
{
    // Toggle green LED and AD0 debug pin
    led_green.toggle();
}

static void timer1_callback(void)
{
    // Toggle yellow LED and AD1 debug pin
    led_yellow.toggle();
}

static void timer2_callback(void)
{
    // Toggle orange LED and AD2 debug pin
    led_orange.toggle();
}

static void timer3_callback(void)
{
    // Toggle red LED and AD3 debug pin
    led_red.toggle();
}

