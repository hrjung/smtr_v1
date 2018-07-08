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

#define QUEUE_SIZE  1024

uint16_t spiRxBuf[QUEUE_SIZE], spiTxBuf[128];
int16_t spiRxIdx=0, spiTxIdx=0;
uint16_t spiPacketReceived=0, tx_flag=0, txLen=0, spi_tx_cnt=0;
extern HAL_Handle halHandle;

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************


void SPI_initRxBuf(void)
{
	int i;

	for(i=0; i<QUEUE_SIZE; i++)	spiRxBuf[i] = 0;
	for(i=0; i<128; i++) spiTxBuf[i] = i;
}

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
	//add for enable Tx int
	SPI_setTxFifoIntLevel(spiHandle, SPI_FifoLevel_1_Word);
	//SPI_enableTxFifoInt(spiHandle);

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

	SPI_initRxBuf();
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

int SPI_isPacketReceived(void)
{
	return spiPacketReceived;
}

void SPI_clearPacketReceived(void)
{
	spiPacketReceived=0;
}

void SPI_enableInterrupt(void)
{
	  SPI_enableRxFifoInt(halHandle->spiAHandle);
	  SPI_enableTxFifoInt(halHandle->spiAHandle);
	  SPI_enableInt(halHandle->spiAHandle);
	  PIE_enableInt(halHandle->pieHandle, PIE_GroupNumber_6, PIE_InterruptSource_SPIARX);
	  PIE_enableInt(halHandle->pieHandle, PIE_GroupNumber_6, PIE_InterruptSource_SPIATX);
	  CPU_enableInt(halHandle->cpuHandle, CPU_IntNumber_6);
}

uint16_t spi_find_first=0, spi_chk_ok=0, rx_cnt=0;
interrupt void spiARxISR(void)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;
	uint16_t i, data, checksum=0, buf[15];

	data = SPI_read(halHandle->spiAHandle);

	if(spi_find_first == 0)
	{
		if(spiRxIdx == 0 && data == 0xAAAA)
		{
			spiRxBuf[spiRxIdx++]=data;
		}
		else if(spiRxIdx == 1 && data == 0x5555 && spiRxBuf[0] == 0xAAAA)
		{
			spiRxBuf[spiRxIdx++]=data;
			spi_find_first=1;
		}
		else
			spiRxIdx=0;
	}
	else
	{
		spiRxBuf[spiRxIdx++]=data;

		if(spiRxIdx >= spiRxBuf[2]) // last data
		{
			// verify checksum
			for(i=0; i<spiRxIdx-1; i++) checksum += spiRxBuf[i];
			if(checksum == spiRxBuf[spiRxIdx-1]) spi_chk_ok=1;
			else spi_chk_ok=0;
	#if 1
			if(spiRxBuf[4] == 0x100 || spiRxBuf[4] == 0x80)
			{
				//if(spiRxBuf[2] == 0x100)  // request ACK
				{
					buf[0]=0x5555;
					buf[1]=0xAAAA;
					buf[2]=spiRxBuf[2];
					buf[3]=spiRxBuf[3];
					buf[4]=0x1; //ACK
					checksum = 0;
					for(i=0; i<5; i++) checksum += buf[i];

					for(i=0; i<5; i++) spiTxBuf[i] = buf[i];
					spiTxBuf[5] = checksum;
					txLen = 6;

				}
	//			else if(spiRxBuf[2] == 0x80)  // report error
	//			{
	//				buf[0]=spiRxBuf[0];
	//				buf[1]=spiRxBuf[1];
	//				buf[2]=spiRxBuf[2];
	//				buf[3]=0x1;
	//				buf[4]=0x2;
	//				buf[5]=0x4;
	//				buf[6]=0x8;
	//				buf[7]=0x10;
	//				buf[8]=0x20;
	//				checksum = 0;
	//				for(i=0; i<9; i++) checksum += buf[i];
	//
	//				for(i=0; i<9; i++) spiTxBuf[i] = buf[i];
	//				spiTxBuf[9] = checksum;
	//				txLen = 10;
	//			}
	//			SPI_disableInt(halHandle->spiAHandle);
	//			SPI_resetTxFifo(halHandle->spiAHandle);
	//			SPI_enableTxFifo(halHandle->spiAHandle);
	//			SPI_enableTxFifoInt(halHandle->spiAHandle);
	//			SPI_disableRxFifoInt(halHandle->spiAHandle);
	//			SPI_enableInt(halHandle->spiAHandle);
			}
	#endif

			for(i=0; i<spiRxIdx; i++) spiRxBuf[i] = 0;

			spiRxIdx = 0;
			spi_find_first=0;
			spiPacketReceived=1; //notify SPI data received

			SPI_resetTxFifo(halHandle->spiAHandle);
			SPI_enableTxFifo(halHandle->spiAHandle);
		}
	}
	rx_cnt++;

spi_rx_exit:
	// Clr RXFIFO interrupts
//	SPI_Regs.SPIFFRX.bit.RXFFOVFCLR = 1;
//	SPI_Regs.SPIFFRX.bit.RXFFINTCLR = 1;
	SPI_clearRxFifoOvf(halHandle->spiAHandle);
	SPI_clearRxFifoInt(halHandle->spiAHandle);

//	PieCtrlRegs.PIEACK.all         = PIEACK_GROUP6;    // Issue PIE ack
	PIE_clearInt(obj->pieHandle,PIE_GroupNumber_6);
}

interrupt void spiATxISR(void)
{
	int i;
	HAL_Obj *obj = (HAL_Obj *)halHandle;

	SPI_write(halHandle->spiAHandle, spiTxBuf[spiTxIdx]);
	spiTxIdx++;

	if(spiTxIdx >= txLen)
	{
		spiTxIdx=0;
		txLen=0;
		for(i=0; i<15; i++) spiTxBuf[i]=0;
//		SPI_disableInt(halHandle->spiAHandle);
//		SPI_resetRxFifo(halHandle->spiAHandle);
//		SPI_enableRxFifo(halHandle->spiAHandle);
//		SPI_disableTxFifoInt(halHandle->spiAHandle);
//		SPI_enableRxFifoInt(halHandle->spiAHandle);
//		SPI_enableInt(halHandle->spiAHandle);
	}

spi_tx_exit:

	spi_tx_cnt++;
	SPI_clearTxFifoInt(halHandle->spiAHandle);
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


