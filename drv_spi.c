//###########################################################################
//
// FILE:   nara_inv.c
//
// TITLE:  Utility driver to provide several functions to inverter of NARA.
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <Assert.h>
#include "drv_spi.h"
#include "spi.h"

//*****************************************************************************
//
//! \addtogroup modebus_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If buffered mode is defined, set aside RX and TX buffers and read/write
// pointers to control them.
//
//*****************************************************************************

//*****************************************************************************
//
// This global controls whether or not we are echoing characters back to the
// transmitter.  By default, echo is enabled but if using this module as a
// convenient method of implementing a buffered serial interface over which
// you will be running an application protocol, you are likely to want to
// disable echo by calling UARTEchoSet(false).
//
//*****************************************************************************
//#define
extern HAL_Handle halHandle;

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

void spi_fifo_init(SPI_Handle spiHandle)
{
    spiHandle->SPIFFTX = 0xE040;
    spiHandle->SPIFFRX = 0x2044;        // RXFIFO Reset, RXFFINT CLR, FXFFIL2
    spiHandle->SPIFFCT = 0x0;           // FFTXDLYx
}

void spi_init(SPI_Handle spiHandle)
{
    spiHandle->SPICCR = 0x000F;         // SPI CHAR3-0, character length 16, for 8 0x0007;
    spiHandle->SPICTL = 0x0006;         // MASTER, TALK
    spiHandle->SPIBRR = 0x007F;         // SPIBRR, for 127, baud rate = (LSPCLK)/(SPIBRR + 1)
    spiHandle->SPICCR = 0x009F;         // SPI SW RESET, SPILBK, SPICHAR3-0
//    SPI_setClkPolarity(spiHandle, SPI_ClkPolarity_OutputFallingEdge_InputRisingEdge);
    SPI_disableLoopBack(spiHandle);
    SPI_setPriority(spiHandle, SPI_Priority_FreeRun);
}

void setupSpiA(SPI_Handle spiHandle)
{
    SPI_reset(spiHandle);
    SPI_setMode(spiHandle,SPI_Mode_Slave);
    SPI_setClkPolarity(spiHandle,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    SPI_enableTx(spiHandle);
    SPI_enableTxFifoEnh(spiHandle);
    SPI_enableTxFifo(spiHandle);
    SPI_setTxDelay(spiHandle,0x0018);
    SPI_setBaudRate(spiHandle,(SPI_BaudRate_e)(0x000d));
    SPI_setCharLength(spiHandle,SPI_CharLength_16_Bits);
    SPI_setSuspend(spiHandle,SPI_TxSuspend_free);
    SPI_enable(spiHandle);

  return;
}

void setupSpiB(SPI_Handle spiHandle)
{
    SPI_reset(spiHandle);
    SPI_setMode(spiHandle,SPI_Mode_Master);
    SPI_setClkPolarity(spiHandle,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    SPI_enableTx(spiHandle);
    SPI_enableTxFifoEnh(spiHandle);
    SPI_enableTxFifo(spiHandle);
    SPI_setTxDelay(spiHandle,0x0018);
    SPI_setBaudRate(spiHandle,(SPI_BaudRate_e)(0x000d));
    SPI_setCharLength(spiHandle,SPI_CharLength_16_Bits);
    SPI_setSuspend(spiHandle,SPI_TxSuspend_free);
    SPI_enable(spiHandle);

  return;
}

uint16_t SPI_Write8(uint16_t addr, uint16_t data)
{
    uint16_t adata;
    uint16_t rdata;

    adata = ((addr << 8) & 0xFF00) | (data & 0x00FF);
    SPI_write(halHandle->spiBHandle, adata);
    while(SPI_getRxFifoStatus(halHandle->spiBHandle) != 1) { }
    rdata = SPI_read(halHandle->spiBHandle);

    return rdata;
}

uint16_t SPI_Read8(uint16_t addr)
{
    uint16_t rdata;

    addr = (0x80 | addr) << 8;
    SPI_write(halHandle->spiBHandle, addr);
    while(SPI_getRxFifoStatus(halHandle->spiBHandle) != 1) { }
    rdata = SPI_read(halHandle->spiBHandle);
    return (0x00FF & rdata);
}

uint16_t SPI_Read16(uint16_t chNo)
{
    uint16_t addr;
    uint16_t rdata;

    addr = (0x80 | (chNo << 3)) << 8;
    SPI_write(halHandle->spiBHandle, addr);
    while(SPI_getRxFifoStatus(halHandle->spiBHandle) != 1) { }
    rdata = SPI_read(halHandle->spiBHandle);
    rdata = rdata >> 2;
    return (0x03FF & rdata);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


