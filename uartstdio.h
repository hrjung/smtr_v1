//###########################################################################
//
// FILE:   uartstdio.h
//
// TITLE:  Prototypes for the UART console functions.
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef __UARTSTDIO_H__
#define __UARTSTDIO_H__

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
#include "hal.h"
#include "uart.h"


//*****************************************************************************
//
// If built for buffered operation, the following labels define the sizes of
// the transmit and receive buffers respectively.
//
//*****************************************************************************
#ifndef UART_RX_BUFFER_SIZE
#define UART_RX_BUFFER_SIZE     128
#endif
#ifndef UART_TX_BUFFER_SIZE
//#define UART_TX_BUFFER_SIZE     12800
#define UART_TX_BUFFER_SIZE     8192
#endif

//typedef unsigned char tBoolean;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void UARTStdioInit(HAL_Handle handle, int ulPortNum);
extern void UARTStdioInitExpClk(HAL_Handle handle, int ulPortNum, unsigned long ulBaud);
extern int UARTgets(char *pcBuf, unsigned long ulLen);
extern unsigned char UARTgetc(void);
extern void UARTputc(char c); //hrjung add
extern void UARTprintf(char *pcString, ...);
extern int UARTwrite(unsigned char *pcBuf, unsigned long ulLen);
extern void UARTFlushTx(CPU_Handle cpuHandle);
extern void UARTFlushRx(CPU_Handle cpuHandle);
extern int UARTRxBytesAvail(void);
extern int UARTTxBytesFree(void);
extern void UARTEchoSet(tBoolean bEnable);
__interrupt void UARTStdioIntHandler(void);
extern void PrintWave(int ch0_mode, void *ch0_data, int ch1_mode, void *ch1_data, int ch2_mode, void *ch2_data);
#ifdef FLOAT_PRINT
extern void reverse(char *str, int len);
extern int intToStr(int x, char str[], int d);
extern void ftoa(float f, char *buffer, long length);
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __UARTSTDIO_H__


