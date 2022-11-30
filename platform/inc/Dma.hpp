/**
 * @file       Dma.hpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef DMA_HPP_
#define DMA_HPP_

#include <stdint.h>

#include "Callback.hpp"
#include "Mutex.hpp"
#include "Semaphore.hpp"

class Dma
{

friend class InterruptHandler;

public:
  Dma();
  void init(void);
  uint32_t memcpy(uint8_t* dst, uint8_t* src, uint32_t length, bool dma = true);
  uint32_t memset(uint8_t* dst, uint8_t val, uint32_t length, bool dma = true);
private:
  uint32_t memcpyDma(uint8_t* dst, uint8_t* src, uint32_t length);
  uint32_t memsetDma(uint8_t* dst, uint8_t val, uint32_t length);
protected:
  void interruptHandler(void);
  void errorHandler(void);
private:
  Mutex mutex_;
  SemaphoreBinary semaphore_;
  bool is_initialized;
};

#endif /* DMA_HPP_ */
