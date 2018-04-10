//###########################################################################
//
// FILE:   uart.h
//
// TITLE:  Stellaris style wrapper driver for C28x SCI peripheral.
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef __UART_H__
#define __UART_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "hal_obj.h"
#include "sw/drivers/sci/src/32b/f28x/f2806x/sci.h"

//*****************************************************************************
//
// Values that can be passed to UARTIntEnable, UARTIntDisable, and UARTIntClear
// as the ulIntFlags parameter, and returned from UARTIntStatus.
//
//*****************************************************************************
#define UART_INT_RXERR          0x01
#define UART_INT_RXRDY_BRKDT    0x02
#define UART_INT_TXRDY          0x04
#define UART_INT_TXFF           0x08
#define UART_INT_RXFF           0x10

//#define INT_UART0               0x92 // UART0 Rx and Tx
//#define INT_UART1               0xA2 // UART1 Rx and Tx

// Imported ByPark
//#define SYSCTL_PERIPH_UART0     0x1
//#define SYSCTL_PERIPH_UART1     0x2
//#define SYSTEM_CLOCK_SPEED      90000000
//#define UART0_BASE              0x00007050  // SCIA_BASE_ADDR
//#define UART1_BASE              0x00007750  // SCIB_BASE_ADDR
#define SCI_TX                  0
#define SCI_RX                  1
#define SCI_A                   0
#define SCI_B                   1
//*****************************************************************************
//
// Values that can be passed to UARTConfigSetExpClk as the ulConfig parameter
// and returned by UARTConfigGetExpClk in the pulConfig parameter.
// Additionally, the UART_CONFIG_PAR_* subset can be passed to
// UARTParityModeSet as the ulParity parameter, and are returned by
// UARTParityModeGet.
//
//*****************************************************************************
#define UART_CONFIG_WLEN_MASK   0x00000007  // Mask for extracting word length
#define UART_CONFIG_WLEN_8      0x00000007  // 8 bit data
#define UART_CONFIG_WLEN_7      0x00000006  // 7 bit data
#define UART_CONFIG_WLEN_6      0x00000005  // 6 bit data
#define UART_CONFIG_WLEN_5      0x00000004  // 5 bit data
#define UART_CONFIG_STOP_MASK   0x00000080  // Mask for extracting stop bits
#define UART_CONFIG_STOP_ONE    0x00000000  // One stop bit
#define UART_CONFIG_STOP_TWO    0x00000080  // Two stop bits
#define UART_CONFIG_PAR_MASK    0x00000060  // Parity Mask
#define UART_CONFIG_PAR_NONE    0x00000000  // No parity
#define UART_CONFIG_PAR_EVEN    0x00000060  // Even parity
#define UART_CONFIG_PAR_ODD     0x00000020  // Odd parity
#define UART_CONFIG_PAR_ONE     0x00000020  // Parity bit is one
#define UART_CONFIG_PAR_ZERO    0x00000060  // Parity bit is zero

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ulTxLevel parameter and
// returned by UARTFIFOLevelGet in the pulTxLevel.
//
//*****************************************************************************
#define UART_FIFO_TX1_8         0x00000001  // Transmit interrupt at 1/4 Full
#define UART_FIFO_TX2_8         0x00000002  // Transmit interrupt at 1/2 Full
#define UART_FIFO_TX4_8         0x00000003  // Transmit interrupt at 3/4 Full
#define UART_FIFO_TX6_8         0x00000004  // Transmit interrupt Full

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ulRxLevel parameter and
// returned by UARTFIFOLevelGet in the pulRxLevel.
//
//*****************************************************************************
#define UART_FIFO_RX1_8         0x00000001  // Receive interrupt at 1/4 Full
#define UART_FIFO_RX2_8         0x00000002  // Receive interrupt at 1/2 Full
#define UART_FIFO_RX4_8         0x00000003  // Receive interrupt at 3/4 Full
#define UART_FIFO_RX6_8         0x00000004  // Receive interrupt at Full

//*****************************************************************************
//
// Values that can be passed to UARTDMAEnable() and UARTDMADisable().
//
//*****************************************************************************
#define UART_DMA_ERR_RXSTOP     0x00000004  // Stop DMA receive if UART error
#define UART_DMA_TX             0x00000002  // Enable DMA for transmit
#define UART_DMA_RX             0x00000001  // Enable DMA for receive

//*****************************************************************************
//
// Values returned from UARTRxErrorGet().
//
//*****************************************************************************
#define UART_RXERROR_OVERRUN    0x00000008
#define UART_RXERROR_BREAK      0x00000020
#define UART_RXERROR_PARITY     0x00000004
#define UART_RXERROR_FRAMING    0x00000010

//*****************************************************************************
//
// Values that can be passed to UARTHandshakeOutputsSet() or returned from
// UARTHandshakeOutputGet().
//
//*****************************************************************************
#define UART_OUTPUT_RTS         0x00000800
#define UART_OUTPUT_DTR         0x00000400

//*****************************************************************************
//
// Values that can be returned from UARTHandshakeInputsGet().
//
//*****************************************************************************
#define UART_INPUT_RI           0x00000100
#define UART_INPUT_DCD          0x00000004
#define UART_INPUT_DSR          0x00000002
#define UART_INPUT_CTS          0x00000001

//*****************************************************************************
//
// Values that can be passed to UARTTxIntModeSet() or returned from
// UARTTxIntModeGet().
//
//*****************************************************************************
#define UART_TXINT_MODE_FIFO_M  0x0000001F
#define UART_TXINT_MODE_EOT     0x00000000

typedef unsigned char tBoolean;

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
    extern void UARTConfigSetExpClk(SCI_Handle sciHandle, unsigned long ulBaud);
    extern void UARTEnable(SCI_Handle sciHandle);
    extern void UARTDisable(SCI_Handle sciHandle);
    extern tBoolean UARTCharsAvail(SCI_Handle sciHandle);
    extern tBoolean UARTSpaceAvail(SCI_Handle sciHandle);
    extern long UARTCharGetNonBlocking(SCI_Handle sciHandle);
    extern long UARTCharGet(SCI_Handle sciHandle);
    extern tBoolean UARTCharPutNonBlocking(SCI_Handle sciHandle, unsigned char ucData);
    extern void UARTCharPut(SCI_Handle sciHandle, unsigned char ucData);
    extern tBoolean UARTBusy(SCI_Handle sciHandle);
    extern void UARTRXIntRegister(HAL_Handle handle, int ulPortNum, void (*pfnHandler)(void));
    extern void UARTTXIntRegister(HAL_Handle handle, int ulPortNum, void (*pfnHandler)(void));
    extern void UARTIntEnable(SCI_Handle sciHandle, unsigned long ulIntFlags);
    extern void UARTIntDisable(SCI_Handle sciHandle, unsigned long ulIntFlags);
    extern unsigned long UARTIntStatus(SCI_Handle sciHandle, tBoolean bMasked);
    extern void UARTIntClear(SCI_Handle sciHandle, unsigned long ulIntFlags);
    extern tBoolean IsModbusTxBufferEmpty();

// API for Interrupt setting
    extern tBoolean IntMasterEnable(void);
    extern tBoolean IntMasterDisable(void);
    extern void IntRegister(unsigned long ulInterrupt, void (*pfnHandler)(void));
    extern void IntEnable(HAL_Handle handle, int ulPortNum, int intMode);
    extern void IntDisable(HAL_Handle handle, int ulPortNum, int intMode);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  __UART_H__


