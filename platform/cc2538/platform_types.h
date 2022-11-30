/**
 * @file       platform_types.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef PLATFORM_TYPES_H_
#define PLATFORM_TYPES_H_

/*================================ include ==================================*/

#include <stdint.h>
#include <stdbool.h>

#include "platform_includes.h"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

typedef struct
{
  bool bExternalOsc32k;
  bool bInternalOsc32M;
  uint32_t SysClkDivider;
  uint32_t IOClkDivider;
} BoardParams;

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

/*================================ private ==================================*/

#endif  /* PLATFORM_TYPES_H_ */
