/**
 * @file       Gpio.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include "Gpio.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Gpio::Gpio(const GpioConfig& config) : config_(config)
{
}

bool Gpio::operator==(const Gpio& other)
{
	return (&config_ == &other.config_);
}

const GpioConfig& Gpio::getGpioConfig(void)
{
	return config_;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
