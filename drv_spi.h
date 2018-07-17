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

#define SPI_NAK		(0x0000)
#define SPI_ACK		(0x0001)


#define SPICMD_CTRL_RUN		(0x0001)
#define SPICMD_CTRL_STOP	(0x0002)
#define SPICMD_CTRL_DIR_F	(0x0004)
#define SPICMD_CTRL_DIR_R	(0x0008)

#define SPICMD_PARAM_W		(0x0010)
#define SPICMD_PARAM_R		(0x0020)

#define SPICMD_REQ_ST		(0x0040)
#define SPICMD_REQ_ERR		(0x0080)


#define SPICMD_RESP_ACK		(0x0100)
#define SPICMD_RESP_ST		(0x0200)
#define SPICMD_RESP_ERR		(0x0400)
#define SPICMD_RESP_PARAM	(0x0800)


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


