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

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== prototypes ================================*/

static bool checkBuffer(uint8_t* buffer1, uint8_t* buffer2, uint8_t length);

/*=============================== variables =================================*/

static uint8_t key[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t input[16] = {0x6c, 0x5f, 0x51, 0x74, 0x53, 0x53, 0x77, 0x5a,
        0x5a, 0x5f, 0x57, 0x58, 0x55, 0x53, 0x06, 0x0f};

static uint8_t output[16] = {0x83, 0x78, 0x10, 0x60, 0x0e, 0x13, 0x93, 0x9b,
        0x27, 0xe0, 0xd7, 0xe4, 0x58, 0xf0, 0xa9, 0xd1};

static uint8_t buffer[16];

/*================================= public ==================================*/

int main (void)
{
    bool status;

	// Initialize board
	board.init();

    // Activate the AES module
    aes.enable();

    // Load the AES key
    aes.loadKey(key);

    // Encrypt a message
    status = aes.encrypt(input, buffer, sizeof(input));
    if (status)
    {
        led_green.on();
        // Check valid encryption
        status = checkBuffer(output, buffer, sizeof(output));
        if (status)
        {
            led_yellow.on();
        }   
    }

    // Decrypt a message
    status = aes.decrypt(output, buffer, sizeof(output));
    if (status)
    {
        led_orange.on();
        // Check valid decryption
        status = checkBuffer(input, buffer, sizeof(input));
        if (status)
        {
            led_red.on();
        }
    }
}

/*================================ private ==================================*/

static bool checkBuffer(uint8_t* buffer1, uint8_t* buffer2, uint8_t length)
{
    for (uint8_t i = 0; i < length; ++i)
    {
        if (buffer1[i] != buffer2[i])
        {
            return false;
        }
    }

    return true;
}