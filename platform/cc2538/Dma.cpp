/**
 * @file       Dma.cpp
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================ include ==================================*/

#include <string.h>

#include "InterruptHandler.hpp"
#include "Dma.hpp"

#include "platform_includes.h"
#include "platform_types.hpp"

/*================================ define ===================================*/

#define DMA_CHANNEL             ( UDMA_CH30_SW )
#define DMA_CHANNEL_SEL         ( UDMA_CH30_SW | UDMA_PRI_SELECT )
#define DMA_CONTROL_MEMCPY      ( UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_8 | UDMA_ARB_1 )
#define DMA_CONTROL_MEMSET      ( UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1 )
#define DMA_MODE                ( UDMA_MODE_AUTO )
#define DMA_ATTRIBUTES          ( UDMA_ATTR_ALL )

#define DMA_TRANSFER_LIMIT      ( 1024 )

/*================================ typedef ==================================*/

/*=============================== variables =================================*/

#if defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
static unsigned char ucDMAControlTable[1024];
#else
static unsigned char ucDMAControlTable[1024] __attribute__ ((aligned(1024)));
#endif

/*=============================== prototypes ================================*/

/*================================= public ==================================*/

Dma::Dma():
  mutex_(), semaphore_(false), is_initialized(false)
{  
}

void Dma::init(void)
{
  if(!is_initialized)
  {
    /* Enable the uDMA controller */
    uDMAEnable();

    /* Set the base for the channel control table */
    uDMAControlBaseSet(&ucDMAControlTable[0]);
    
    /* Register the interrupt handler */
    InterruptHandler::getInstance().setInterruptHandler(*this);

    /* Set the uDMA interrupt priorities */
    IntPrioritySet(INT_UDMA, 0xF0);
    IntPrioritySet(INT_UDMAERR, 0xF0);

    /* Enable the uDMA interrupts */
    IntEnable(INT_UDMA);
    IntEnable(INT_UDMAERR);
    
    /* Mark as initialized */
    is_initialized = true;
  }
}

uint32_t Dma::memcpy(uint8_t* dst, uint8_t* src, uint32_t length, bool dma)
{
  if (dma)
  {
    memcpyDma(dst, src, length);
  }
  else
  {
    memcpy(dst, src, length);
  }
  
  return length;
}

uint32_t Dma::memset(uint8_t* dst, uint8_t val, uint32_t length, bool dma)
{
  if (dma)
  {
    memsetDma(dst, val, length);
  }
  else
  {
    memsetDma(dst, val, length);
  }
  
  return length;
}

/*=============================== protected =================================*/

void Dma::interruptHandler(void)
{  
  semaphore_.giveFromInterrupt();
}

void Dma::errorHandler(void)
{
}

/*================================ private ==================================*/

uint32_t Dma::memsetDma(uint8_t* dst, uint8_t val, uint32_t length)
{
  uint32_t size;
  uint32_t total = 0;

  /* Take the mutex */
  mutex_.take();
  
  /* Run while there are bytes */
  while (length > 0)
  {
    /* If length is above limit */
    if (length > DMA_TRANSFER_LIMIT)
    {
      size = DMA_TRANSFER_LIMIT;
    }
    else
    {
      size = length;
    }
    
    /* Setup DMA control for memory copy */
    uDMAChannelControlSet(DMA_CHANNEL_SEL, DMA_CONTROL_MEMSET);
    
    /* Setup DMA buffers for memory copy using BASIC mode */
    uDMAChannelTransferSet(DMA_CHANNEL_SEL, DMA_MODE, &val, dst, size);
    
    /* Enable DMA channel */
    uDMAChannelEnable(DMA_CHANNEL);
    
    /* Request DMA transfer */
    uDMAChannelRequest(DMA_CHANNEL);
    
    /* Wait until transfer is complete */
    semaphore_.take();
    
    /* Update pointers, length and total bytes */
    dst += size;
    length -= size;
    total += size;
  }
  
  /* Disable DMA channel */
  uDMAChannelAttributeDisable(DMA_CHANNEL, DMA_ATTRIBUTES);
  
  /* Give the mutex */
  mutex_.give();
  
  return total;
}

uint32_t Dma::memcpyDma(uint8_t* dst, uint8_t* src, uint32_t length)
{
  uint32_t size;
  uint32_t total = 0;

  /* Take the mutex */
  mutex_.take();
  
  /* Run while there are bytes */
  while (length > 0)
  {
    /* If length is above limit */
    if (length > DMA_TRANSFER_LIMIT)
    {
      size = DMA_TRANSFER_LIMIT;
    }
    else
    {
      size = length;
    }
    
    /* Setup DMA control for memory copy */
    uDMAChannelControlSet(DMA_CHANNEL_SEL, DMA_CONTROL_MEMCPY);
    
    /* Setup DMA buffers for memory copy using BASIC mode */
    uDMAChannelTransferSet(DMA_CHANNEL_SEL, DMA_MODE, src, dst, size);
    
    /* Enable DMA channel */
    uDMAChannelEnable(DMA_CHANNEL);
    
    /* Request DMA transfer */
    uDMAChannelRequest(DMA_CHANNEL);
    
    /* Wait until transfer is complete */
    semaphore_.take();
    
    /* Update pointers, length and total bytes */
    src += size;
    dst += size;
    length -= size;
    total += size;
  }
  
  /* Disable DMA channel */
  uDMAChannelAttributeDisable(DMA_CHANNEL, DMA_ATTRIBUTES);
  
  /* Give the mutex */
  mutex_.give();
  
  return total;
}