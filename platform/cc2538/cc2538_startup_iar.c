/**
 * @file       cc2538_startup_iar.c
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       November, 2018
 * @brief
 *
 * @copyright  Copyright 2018, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

/*================================= checks ==================================*/

/* Check if compiler is IAR */
#if !(defined(__IAR_SYSTEMS_ICC__))
#error "startup_iar.c: Unsupported compiler!"
#endif

/*================================ include ==================================*/

#include <stdint.h>

#include "gpio.h"
#include "ioc.h"
#include "sys_ctrl.h"

#include "hw_memmap.h"
#include "hw_sys_ctrl.h"

/*================================ define ===================================*/

#define FLASH_START_ADDR                ( 0x00200000 )
#define BOOTLOADER_BACKDOOR_DISABLE     ( 0xEFFFFFFF )
#define BOOTLOADER_BACKDOOR_ENABLE      ( 0xF6FFFFFF )

/* Macro for hardware access, both direct and via the bit-band region. */
#ifndef HWREG
#define HWREG(x)                                                              \
        (*((volatile uint32_t *)(x)))
#endif

/* Enable the IAR extensions for this source file. */
#pragma language=extended

/*================================ typedef ==================================*/

/* Customer Configuration Area in Lock Page */
typedef struct
{
  uint32_t ui32BootldrCfg;
  uint32_t ui32ImageValid;
  uint32_t ui32ImageVectorAddr;
} lockPageCCA_t;

/*=============================== prototypes ================================*/

/* Forward declaration of the default fault handlers */
static void ResetISR(void);
static void NmiISR(void);
static void FaultISR(void);
static void IntDefaultHandler(void);

/* Handlers that can potentially be registered directly by application */
extern void SVCallIntHandler(void);
extern void PendSVIntHandler(void);
extern void SysTickIntHandler(void);
extern void GPIOAIntHandler(void);
extern void GPIOBIntHandler(void);
extern void GPIOCIntHandler(void);
extern void GPIODIntHandler(void);
extern void UART0IntHandler(void);
extern void UART1IntHandler(void);
extern void SSI0IntHandler(void);
extern void SSI1IntHandler(void);
extern void I2CIntHandler(void);
extern void ADCIntHandler(void);
extern void WatchdogIntHandler(void);
extern void Timer0AIntHandler(void);
extern void Timer0BIntHandler(void);
extern void Timer1AIntHandler(void);
extern void Timer1BIntHandler(void);
extern void Timer2AIntHandler(void);
extern void Timer2BIntHandler(void);
extern void Timer3AIntHandler(void);
extern void Timer3BIntHandler(void);
extern void CompIntHandler(void);
extern void RFCoreTxIntHandler(void);
extern void RFCoreErrIntHandler(void);
extern void IcePickIntHandler(void);
extern void FlashIntHandler(void);
extern void AESIntHandler(void);
extern void PKAIntHandler(void);
extern void SleepModeIntHandler(void);
extern void MacTimerIntHandler(void);
extern void USBIntHandler(void);
extern void uDMAIntHandler(void);
extern void uDMAErrIntHandler(void);

/* Default interrupt handlers, these can be overwritten if defined elsewhere */
#pragma weak SVCallIntHandler = IntDefaultHandler
#pragma weak PendSVIntHandler = IntDefaultHandler
#pragma weak SysTickIntHandler = IntDefaultHandler
#pragma weak GPIOAIntHandler = IntDefaultHandler
#pragma weak GPIOBIntHandler = IntDefaultHandler
#pragma weak GPIOCIntHandler = IntDefaultHandler
#pragma weak GPIODIntHandler = IntDefaultHandler
#pragma weak UART0IntHandler = IntDefaultHandler
#pragma weak UART1IntHandler = IntDefaultHandler
#pragma weak SSI0IntHandler = IntDefaultHandler
#pragma weak SSI1IntHandler = IntDefaultHandler
#pragma weak I2CIntHandler = IntDefaultHandler
#pragma weak ADCIntHandler = IntDefaultHandler
#pragma weak WatchdogIntHandler = IntDefaultHandler
#pragma weak Timer0AIntHandler = IntDefaultHandler
#pragma weak Timer0BIntHandler = IntDefaultHandler
#pragma weak Timer1AIntHandler = IntDefaultHandler
#pragma weak Timer1BIntHandler = IntDefaultHandler
#pragma weak Timer2AIntHandler = IntDefaultHandler
#pragma weak Timer2BIntHandler = IntDefaultHandler
#pragma weak Timer3AIntHandler = IntDefaultHandler
#pragma weak Timer3BIntHandler = IntDefaultHandler
#pragma weak CompIntHandler = IntDefaultHandler
#pragma weak RFCoreTxIntHandler = IntDefaultHandler
#pragma weak RFCoreErrIntHandler = IntDefaultHandler
#pragma weak IcePickIntHandler = IntDefaultHandler
#pragma weak FlashIntHandler = IntDefaultHandler
#pragma weak AESIntHandler = IntDefaultHandler
#pragma weak PKAIntHandler = IntDefaultHandler
#pragma weak SleepModeIntHandler = IntDefaultHandler
#pragma weak MacTimerIntHandler = IntDefaultHandler
#pragma weak USBIntHandler = IntDefaultHandler
#pragma weak uDMAIntHandler = IntDefaultHandler
#pragma weak uDMAErrIntHandler = IntDefaultHandler

/*=============================== variables =================================*/

/* Flash CCA area */
__root const lockPageCCA_t __cca @ ".flashcca" =
{
  BOOTLOADER_BACKDOOR_ENABLE,   // Bootloader backdoor enabled
  0,                            // Image valid bytes
  FLASH_START_ADDR              // Vector table located at flash start address
};

/* FreeRTOS callbacks */
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

/* The entry point for the application startup code */
extern void __iar_program_start(void);

/* Get stack start (highest address) symbol from linker file */
extern const void* STACK_TOP;

/* Stack */
__root static void* dummy_stack @ ".stack";

/* Vector table */
__root void (* const __vector_table[])(void) @ ".intvec" =
{
  (void (*)(void))&STACK_TOP,             // 0 The initial stack pointer
  ResetISR,                               // 1 The reset handler
  NmiISR,                                 // The NMI handler
  FaultISR,                               // The hard fault handler
  IntDefaultHandler,                      // 4 The MPU fault handler
  IntDefaultHandler,                      // 5 The bus fault handler
  IntDefaultHandler,                      // 6 The usage fault handler
  0,0,0,0,                                // 7-10 Reserved
  vPortSVCHandler,                        // 11 SVCall handler
  IntDefaultHandler,                      // 12 Debug monitor handler
  0,                                      // 13 Reserved
  xPortPendSVHandler,                     // 14 The PendSV handler
  xPortSysTickHandler,                    // 15 The SysTick handler
  GPIOAIntHandler,                        // 16 GPIO Port A
  GPIOBIntHandler,                        // 17 GPIO Port B
  GPIOCIntHandler,                        // 18 GPIO Port C
  GPIODIntHandler,                        // 19 GPIO Port D
  0,                                      // 20 none
  UART0IntHandler,                        // 21 UART0 Rx and Tx
  UART1IntHandler,                        // 22 UART1 Rx and Tx
  SSI0IntHandler,                         // 23 SSI0 Rx and Tx
  I2CIntHandler,                          // 24 I2C Master and Slave
  0,0,0,0,0,                              // 25-29 Reserved
  ADCIntHandler,                          // 30 ADC Sequence 0
  0,0,0,                                  // 31-33 Reserved
  WatchdogIntHandler,                     // 34 Watchdog timer, timer 0
  Timer0AIntHandler,                      // 35 Timer 0 subtimer A
  Timer0BIntHandler,                      // 36 Timer 0 subtimer B
  Timer1AIntHandler,                      // 37 Timer 1 subtimer A
  Timer1BIntHandler,                      // 38 Timer 1 subtimer B
  Timer2AIntHandler,                      // 39 Timer 2 subtimer A
  Timer2BIntHandler,                      // 40 Timer 2 subtimer B
  CompIntHandler,                         // 41 Analog Comparator 0
  RFCoreTxIntHandler,                     // 42 RFCore Rx/Tx
  RFCoreErrIntHandler,                    // 43 RFCore Error
  IcePickIntHandler,                      // 44 IcePick
  FlashIntHandler,                        // 45 FLASH Control
  AESIntHandler,                          // 46 AES
  PKAIntHandler,                          // 47 PKA
  SleepModeIntHandler,                    // 48 Sleep Timer
  MacTimerIntHandler,                     // 49 MacTimer
  SSI1IntHandler,                         // 50 SSI1 Rx and Tx
  Timer3AIntHandler,                      // 51 Timer 3 subtimer A
  Timer3BIntHandler,                      // 52 Timer 3 subtimer B
  0,0,0,0,0,0,0,                          // 53-59 Reserved
  USBIntHandler,                          // 60 USB 2538
  0,                                      // 61 Reserved
  uDMAIntHandler,                         // 62 uDMA
  uDMAErrIntHandler,                      // 63 uDMA Error
#ifndef CC2538_USE_ALTERNATE_INTERRUPT_MAP
  0,0,0,0,0,0,                            // 64-69 64-155 are not in use
  0,0,0,0,0,0,0,0,0,0,                    // 70-79
  0,0,0,0,0,0,0,0,0,0,                    // 80-89
  0,0,0,0,0,0,0,0,0,0,                    // 90-99
  0,0,0,0,0,0,0,0,0,0,                    // 100-109
  0,0,0,0,0,0,0,0,0,0,                    // 110-119
  0,0,0,0,0,0,0,0,0,0,                    // 120-129
  0,0,0,0,0,0,0,0,0,0,                    // 130-139
  0,0,0,0,0,0,0,0,0,0,                    // 140-149
  0,0,0,0,0,0,                            // 150-155
  USBIntHandler,                          // 156 USB
  RFCoreTxIntHandler,                     // 157 RFCORE RX/TX
  RFCoreErrIntHandler,                    // 158 RFCORE Error
  AESIntHandler,                          // 159 AES
  PKAIntHandler,                          // 160 PKA
  SleepModeIntHandler,                    // 161 SMTimer
  MacTimerIntHandler,                     // 162 MACTimer
#endif
};

/*================================= public ==================================*/

/*================================ private ==================================*/

/* This is the code that gets called when the processor is reset */
static void ResetISR(void)
{
  /* Workaround for PM debug issue */
  HWREG(SYS_CTRL_EMUOVR) = 0xFF;
  
#ifdef DEBUG
  /* Workaround for System Reset debug issue */
  uint32_t ui32Timeout = 5000000;
  volatile uint32_t* pui32StopAtResetIsr = (uint32_t*)0x20003000;
  while((*pui32StopAtResetIsr == 0xA5F01248) && (ui32Timeout--));
#endif

#ifdef CC2538_USE_ALTERNATE_INTERRUPT_MAP
  /* Enable alternate interrupt mapping */
  HWREG(SYS_CTRL_I_MAP) |= 1;
#endif

  /* Set all GPIOs as input */
  GPIOPinTypeGPIOInput(GPIO_A_BASE, 0xFF);
  GPIOPinTypeGPIOInput(GPIO_B_BASE, 0xFF);
  GPIOPinTypeGPIOInput(GPIO_C_BASE, 0xFF);
  GPIOPinTypeGPIOInput(GPIO_D_BASE, 0xFF);
  
  /* Jump to IAR initialization routine */
  __iar_program_start();
}


/* This is the code that gets called when the processor receives a NMI  */
static void NmiISR(void)
{
  /* Enter an infinite loop */
  while(1)
  {
  }
}

/* This is the code that gets called when the processor receives a fault interrupt */
static void FaultISR(void)
{
  /* Enter an infinite loop */
  while(1)
  {
  }
}

/* This is the code that gets called when the processor receives an unexpected interrupt */
static void IntDefaultHandler(void)
{
  /* Enter an infinite loop */
  while(1)
  {
  }
}
