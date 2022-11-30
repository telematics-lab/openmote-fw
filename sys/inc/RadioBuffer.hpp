/**
 * @file       RadioBuffer.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2016
 * @brief
 *
 * @copyright  Copyright 2016, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef RADIO_BUFFER_HPP_
#define RADIO_BUFFER_HPP_

#include <stdint.h>
#include <stdbool.h>

#define RADIO_BUFFER_LENGTH         ( 256 )
#define RADIO_BUFFER_ELEMENTS       ( 8 ) 

typedef struct
{
  uint8_t buffer[RADIO_BUFFER_LENGTH];
  uint16_t size;
  uint16_t length;
  bool used;
} RadioBuffer;

class RadioBufferManager
{
public:
  RadioBufferManager();
  void init(void);
  bool get(RadioBuffer** rb);
  void release(RadioBuffer* rb);
private:
  RadioBuffer rb_[RADIO_BUFFER_ELEMENTS];
  uint16_t elements_ = RADIO_BUFFER_ELEMENTS;
};

#endif /* RADIO_BUFFER_HPP_ */
