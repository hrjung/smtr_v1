//###########################################################################
//
// FILE:   drv_spi.c
//
// TITLE:  SPI driver for F28069F motorware
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


int16_t spi_index=-1;
extern uint16_t spiRxFlag;
extern uint16_t spiBuff[];
extern HAL_Handle halHandle;

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

#if 0
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
#endif

#if 1
void setupSpiA(SPI_Handle spiHandle)
{
	// Initialize SPI FIFO registers
//	SPIPORT.SPIFFTX.bit.SPIRST     = 1;       // enable fifo - 0xE040;
//	SPIPORT.SPIFFTX.bit.SPIFFENA   = 1;       // enable fifo enhancements
//	SPIPORT.SPIFFTX.bit.TXFIFO     = 1;       // re-enable tx fifo
//	SPIPORT.SPIFFTX.bit.TXFFIENA   = 0;       // disable tx fifo int based txffil match
//	SPIPORT.SPIFFTX.bit.TXFFINTCLR = 1;       // clr TXFFINT flag in SPIFFTX
	SPI_enableChannels(spiHandle);
	SPI_enableTxFifoEnh(spiHandle);
	SPI_enableTxFifo(spiHandle);
	SPI_disableTxFifoInt(spiHandle);
	SPI_clearTxFifoInt(spiHandle);

	//SPIPORT.SPIFFRX.bit.RXFFOVFCLR = 1;     // clr RXFFOVF flag in SPIFFRX - 0x2042;
//	SPIPORT.SPIFFRX.bit.RXFIFORESET = 1;      // re-enable rx fifo
//	SPIPORT.SPIFFRX.bit.RXFFINTCLR  = 1;      // clr RXFFINT flag in SPIFFRX
//	SPIPORT.SPIFFRX.bit.RXFFIENA    = 0;      // disable rx fifo int based rxffil match
//	SPIPORT.SPIFFRX.bit.RXFFIL      = 2;      // two rx data
	SPI_enableRxFifo(spiHandle);
	SPI_clearRxFifoInt(spiHandle);
	SPI_disableRxFifoInt(spiHandle);
	SPI_setRxFifoIntLevel(spiHandle, SPI_FifoLevel_1_Word);

//	spiHandle->SPIFFCT.all=0x0;                  // no time space between consecutive words in a packet
	SPI_setTxDelay(spiHandle, 0);

	// Initialize SPI
//	SPIPORT.SPICCR.bit.SPISWRESET  = 0;       // reset SPI - 0x000F
//	SPIPORT.SPICCR.bit.SPILBK      = 0;       // no loop back
//	SPIPORT.SPICCR.bit.CLKPOLARITY = 0;       // clk polarity is rising edge
//	SPIPORT.SPICCR.bit.SPICHAR     = 0xf;     // data length = 16b
	SPI_reset(spiHandle);
	SPI_disableLoopBack(spiHandle);
	SPI_setClkPolarity(spiHandle, SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
	SPI_setCharLength(spiHandle,SPI_CharLength_16_Bits);

//	SPIPORT.SPICTL.bit.CLK_PHASE     = 0;     // Enable normal phase - 0x0002
//	SPIPORT.SPICTL.bit.MASTER_SLAVE  = 0;     // slave mode
//	SPIPORT.SPICTL.bit.OVERRUNINTENA = 0;     // disable rx overrun flag bit interrupts (SPISTS.7)
//	SPIPORT.SPICTL.bit.TALK          = 1;     // enable talk
//	SPIPORT.SPICTL.bit.SPIINTENA     = 0;     // SPI int disable
	SPI_setClkPhase(spiHandle, SPI_ClkPhase_Delayed);
	SPI_setMode(spiHandle,SPI_Mode_Slave);
	SPI_disableOverRunInt(spiHandle);
	SPI_enableTx(spiHandle);
	SPI_disableInt(spiHandle);

	//SPIPORT.SPIBRR =0x00F;                    // Baud rate

//	SPIPORT.SPICCR.bit.SPISWRESET    = 1;       // Relinquish SPI from Reset
	SPI_enable(spiHandle);

//	SPIPORT.SPIPRI.bit.FREE    = 1;             // Set so breakpoints don't disturb xmission
//	SPIPORT.SPIPRI.bit.STEINV  = 0;             // SPISTE pin in normal mode (no inversion)
//	SPIPORT.SPIPRI.bit.TRIWIRE = 0;             // 4 wire SPI
	SPI_setPriority(spiHandle, SPI_Priority_FreeRun);
	SPI_setSteInv(spiHandle, SPI_SteInv_ActiveLow);
	SPI_setTriWire(spiHandle, SPI_TriWire_NormalFourWire);
}
#else
void setupSpiA(SPI_Handle spiHandle)
{
    SPI_reset(spiHandle);
    SPI_setMode(spiHandle,SPI_Mode_Slave);
    SPI_setClkPolarity(spiHandle,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
    //SPI_setClkPhase(spiHandle, SPI_ClkPhase_Normal); // mode 0 0
    //SPI_setClkPolarity(spiHandle,SPI_ClkPolarity_OutputFallingEdge_InputRisingEdge);
    SPI_setClkPhase(spiHandle, SPI_ClkPhase_Delayed); // mode 1 1
    SPI_enableTx(spiHandle);
    SPI_enableTxFifoEnh(spiHandle);
    SPI_enableTxFifo(spiHandle);
    SPI_enableRxFifo(spiHandle);
    SPI_enableChannels(spiHandle);
    //SPI_setTxDelay(spiHandle,0x0020);
    SPI_setTxDelay(spiHandle,0x00);
    SPI_setBaudRate(spiHandle,(SPI_BaudRate_e)(0x000d)); // no effect in slave mode
    SPI_setCharLength(spiHandle,SPI_CharLength_16_Bits); // 8 bit transfer
    SPI_setSuspend(spiHandle,SPI_TxSuspend_free);
    SPI_enable(spiHandle);
    SPI_setPriority(spiHandle, SPI_Priority_FreeRun);
    SPI_reset(spiHandle);

  return;
}
#endif

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
    SPI_setCharLength(spiHandle,SPI_CharLength_8_Bits);
    SPI_setSuspend(spiHandle,SPI_TxSuspend_free);
    SPI_enable(spiHandle);

  return;
}

//function for reading from spi
uint16_t SPI_readMCU(uint16_t *rxData)
{
    volatile uint16_t WaitTimeOut = 0;
    volatile SPI_FifoStatus_e RxFifoCnt = SPI_FifoStatus_Empty;
    uint16_t i, ret=0;

    // reset the Rx fifo pointer to zero
    SPI_resetRxFifo(halHandle->spiAHandle);
    SPI_enableRxFifo(halHandle->spiAHandle);

    for(i=0; i<4; i++)
    {
		// wait for two words to populate the RX fifo, or a wait timeout will occur
		while((RxFifoCnt < SPI_FifoStatus_1_Word) && (WaitTimeOut < 0xff))
		{
			RxFifoCnt = SPI_getRxFifoStatus(halHandle->spiAHandle);
			WaitTimeOut++;
		}

		if(WaitTimeOut >= 0xff)
			ret = 1;
		else
		{
			//read the spi word
			rxData[i] = SPI_read(halHandle->spiAHandle);
		}
    }

    return ret;
}

uint16_t SPI_writeMCU(uint16_t *txData)
{
    volatile uint16_t WaitTimeOut = 0;
    volatile SPI_FifoStatus_e TxFifoCnt;
    uint16_t ret=0;

    // reset the Rx fifo pointer to zero
    SPI_resetTxFifo(halHandle->spiAHandle);
    SPI_enableTxFifo(halHandle->spiAHandle);

	//read the spi word
	SPI_write(halHandle->spiAHandle, txData[0]);

	TxFifoCnt = SPI_getTxFifoStatus(halHandle->spiAHandle);
    while((TxFifoCnt != SPI_FifoStatus_Empty) && (WaitTimeOut < 0xff))
    {
        TxFifoCnt = SPI_getTxFifoStatus(halHandle->spiAHandle);
        WaitTimeOut++;
    }

    if(WaitTimeOut >= 0xff)
    	ret = 1;


    return ret;
}

interrupt void spiARxISR(void)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;

	HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);

	spi_index++;
	spiBuff[spi_index] = SPI_read(halHandle->spiAHandle);


	// Clr RXFIFO interrupts
//	SPI_Regs.SPIFFRX.bit.RXFFOVFCLR = 1;
//	SPI_Regs.SPIFFRX.bit.RXFFINTCLR = 1;
	SPI_clearRxFifoOvf(halHandle->spiAHandle);
	SPI_clearRxFifoInt(halHandle->spiAHandle);

//	PieCtrlRegs.PIEACK.all         = PIEACK_GROUP6;    // Issue PIE ack
	PIE_clearInt(obj->pieHandle,PIE_GroupNumber_6);

	spiRxFlag=1; //notify SPI data received
}

interrupt void spiATxISR(void)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;

	//HAL_toggleLed(halHandle,(GPIO_Number_e)HAL_Gpio_LED_R);


	PIE_clearInt(obj->pieHandle,PIE_GroupNumber_6);
}

#if 0
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
#endif

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


