 /**
 * @file       RadioBuffer.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <stdio.h>
#include <string.h>

#include "CriticalSection.hpp"

#include "RadioBuffer.hpp"

/*================================ define ===================================*/

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

RadioBufferManager::RadioBufferManager()
{
}

void RadioBufferManager::init(void)
{
  CriticalSection cs;
  
  for (uint16_t i = 0; i < elements_; i++)
  { 
    /* Clear RadioBuffer buffer */
    memset(rb_[i].buffer, 0, sizeof(rb_[i].buffer));
    
    /* Initialize RadioBuffer structure */
    rb_[i].size = sizeof(rb_[i].buffer);
    rb_[i].length = 0;
    rb_[i].used = false;
  }
}

bool RadioBufferManager::get(RadioBuffer** rb)
{
  CriticalSection cs;
  
  /* Search for an empty buffer */
  for (uint16_t i = 0; i < elements_; i++)
  {
    /* If buffer is not used */
    if (rb_[i].used == false)
    {
      /* Mark RadioBuffer as used */
      rb_[i].used = true;
      
      /* Update pointer to RadioBuffer */
      *rb = &rb_[i];
      
      return true;
    }
  }
  
  /* Update pointer to RadioBuffer */
  *rb = nullptr;
  
  return false;
}

void RadioBufferManager::release(RadioBuffer* rb)
{ 
  CriticalSection cs;
  
  /* Clear the buffer */
  rb->length = 0;
  rb->used = false;
}

/*=============================== protected =================================*/

/*================================ private ==================================*/
