//###########################################################################
//
// FILE:   modebusstdio.h
//
// TITLE:  Prototypes for the MODEBUS functions.
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

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


//*****************************************************************************
//
// If built for buffered operation, the following labels define the sizes of
// the transmit and receive buffers respectively.
//
//*****************************************************************************
//#define


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

extern int SPI_isFull(void); //큐가 꽉 찼는지 확인
extern int SPI_isEmpty(void); //큐가 비었는지 확인
extern int SPI_enqueue(uint16_t data); //큐에 보관
extern int SPI_dequeue(uint16_t *data); //큐에서 꺼냄

extern void setupSpiA(SPI_Handle spiHandle);
extern void setupSpiB(SPI_Handle spiHandle);

extern int SPI_isPacketReceived(void);
extern void SPI_clearPacketReceived(void);
extern void SPI_enableInterrupt(void);

extern uint16_t SPI_readMCU(uint16_t *rxData);
extern uint16_t SPI_writeMCU(uint16_t *txData);
//extern void spi_fifo_init(SPI_Handle spiHandle);
//extern void spi_init(SPI_Handle spiHandle);
//extern uint16_t SPI_Write8(uint16_t addr, uint16_t data);
//extern uint16_t SPI_Read8(uint16_t addr);
//extern uint16_t SPI_Read16(uint16_t chNo);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __DRV_SPI_H__


