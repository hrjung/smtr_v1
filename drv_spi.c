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
#include "common_tools.h"
//*****************************************************************************
//
//! \addtogroup modebus_api
//! @{
//
//*****************************************************************************

#define QUEUE_SIZE  64

typedef struct
{
	uint16_t buf[QUEUE_SIZE];
	uint16_t idx;
	uint16_t cnt;
} spi_queue_st;

spi_queue_st spiRx, spiTx;

//uint16_t spiRxBuf[QUEUE_SIZE], spiTxBuf[QUEUE_SIZE];
//int16_t spiRxIdx=0, spiTxIdx=0;
uint16_t spiPacketReceived=0, rx_seq_no, txLen=0;
//uint16_t spi_tx_cnt=0, spi_rx_cnt=0;
uint16_t spi_find_first=0, spi_chk_ok=0;
uint16_t spi_checksum=0;

extern HAL_Handle halHandle;

//*****************************************************************************
//
// Function implementation
//
//*****************************************************************************

void SPI_initRxBuf(void)
{
	int i;

	spiRx.idx=0;
	spiRx.cnt=0;
	spiTx.idx=0;
	spiTx.cnt=0;
	for(i=0; i<QUEUE_SIZE; i++)	spiRx.buf[i] = 0;
	for(i=0; i<QUEUE_SIZE; i++) spiTx.buf[i] = i;
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
	//SPI_setTxFifoIntLevel(spiHandle, SPI_FifoLevel_1_Word);
	SPI_setTxFifoIntLevel(spiHandle, SPI_FifoLevel_Empty);
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
	//SPI_disableTx(spiHandle); // disable Tx until Rx completed
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

#if 0
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
#endif

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

void SPI_makeStatusResponse(uint16_t seq_no)
{

}

void SPI_makeErrorResponse(uint16_t seq_no)
{

}

void SPI_makeParamResponse(uint16_t seq_no, uint16_t index)
{
	uint16_t i, checksum=0;
	uint16_t buf[10];

	txLen = 9;
	buf[0] = 0xAAAA;
	buf[1] = 0x5555;
	buf[2] = txLen;
	buf[3] = seq_no;
	buf[4] = SPICMD_RESP_PARAM;
	buf[5] = index;
	UTIL_packingParam(index, (uint16_t *)&buf[6]);

	for(i=0; i<txLen-1; i++) checksum += buf[i];
	buf[txLen-1] = checksum;

	for(i=0; i<txLen; i++) spiTx.buf[i] = buf[i];
}

void SPI_makeResponse(uint16_t seq_no, uint16_t resp)
{
	uint16_t i, checksum=0;
	uint16_t buf[10];

	txLen = 7;
	buf[0] = 0xAAAA;
	buf[1] = 0x5555;
	buf[2] = txLen;
	buf[3] = seq_no;
	buf[4] = SPICMD_RESP_ACK;
	buf[5] = resp;

	for(i=0; i<txLen-1; i++) checksum += buf[i];
	buf[6] = checksum;
	for(i=0; i<txLen; i++) spiTx.buf[i] = buf[i];

}

interrupt void spiARxISR(void)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;
	uint16_t i, data, seq_no, cmd, checksum=0;

	UTIL_testbit(1);

	data = SPI_read(halHandle->spiAHandle);

	if(spi_find_first == 0)
	{
		if(spiRx.idx == 0 && data == 0xAAAA)
		{
			spiRx.buf[spiRx.idx++]=data;
		}
		else if(spiRx.idx == 1 && data == 0x5555 && spiRx.buf[0] == 0xAAAA)
		{
			spiRx.buf[spiRx.idx++]=data;
			spi_find_first=1;
		}
		else
			spiRx.idx=0;
	}
	else
	{
		spiRx.buf[spiRx.idx++]=data;

		if(spiRx.idx >= spiRx.buf[2]) // last data
		{
			// verify checksum
			for(i=0; i<spiRx.idx-1; i++) checksum += spiRx.buf[i];
			if(checksum == spiRx.buf[spiRx.idx-1]) spi_chk_ok=1;
			else spi_chk_ok=0;
#if 1
			seq_no = spiRx.buf[3];
			cmd = spiRx.buf[4]&0x00FF;
			if(cmd&0x001F) // command
			{
				switch(cmd&0x001F)
				{
				case SPICMD_CTRL_RUN:
				case SPICMD_CTRL_STOP:
				case SPICMD_CTRL_DIR_F:
				case SPICMD_CTRL_DIR_R:
					break;

				case SPICMD_PARAM_W:

					break;
				}

				//generate ack response
				SPI_makeResponse(seq_no, SPI_ACK);

			}
			else if(cmd&0x00E0) // status request
			{
				switch(cmd&0x00E0)
				{
				case SPICMD_REQ_ST:
					SPI_makeStatusResponse(seq_no);
					break;

				case SPICMD_REQ_ERR:
					SPI_makeErrorResponse(seq_no);
					break;

				case SPICMD_PARAM_R:
					SPI_makeParamResponse(seq_no, spiRx.buf[5]);
				}

			}
			else
			{
				//NAK response
				SPI_makeResponse(seq_no, SPI_NAK); // seq no
			}
#else
			if(spiRxBuf[4] == 0x100 || spiRxBuf[4] == 0x80)
			{
				//if(spiRxBuf[2] == 0x100)  // request ACK
				{
					SPI_makeResponse(spiRx.buf[3], SPI_ACK);
				}
			}

#endif

			for(i=0; i<spiRx.idx; i++) spiRx.buf[i] = 0;

			spiRx.idx = 0;
			spi_find_first=0;
			spiPacketReceived=1; //notify SPI data received
			rx_seq_no = seq_no;
			spi_checksum = checksum;

//			SPI_resetTxFifo(halHandle->spiAHandle);
//			SPI_enableTxFifo(halHandle->spiAHandle);
			// write 1st word for next transmit
//			spiTxIdx=0;
//			SPI_write(halHandle->spiAHandle, spiTxBuf[spiTxIdx]);
//			spiTxIdx++;
			//SPI_enableTx(halHandle->spiAHandle);
		}
	}
	spiRx.cnt++;

	// Clr RXFIFO interrupts
//	SPI_Regs.SPIFFRX.bit.RXFFOVFCLR = 1;
//	SPI_Regs.SPIFFRX.bit.RXFFINTCLR = 1;
	SPI_clearRxFifoOvf(halHandle->spiAHandle);
	SPI_clearRxFifoInt(halHandle->spiAHandle);

//	PieCtrlRegs.PIEACK.all         = PIEACK_GROUP6;    // Issue PIE ack
	PIE_clearInt(obj->pieHandle,PIE_GroupNumber_6);

	UTIL_testbit(0);
}

interrupt void spiATxISR(void)
{
	int i;
	HAL_Obj *obj = (HAL_Obj *)halHandle;

	SPI_write(halHandle->spiAHandle, spiTx.buf[spiTx.idx]);
	spiTx.idx++;

	if(spiTx.idx >= txLen)
	{
		spiTx.idx=0;
		txLen=0;
		for(i=0; i<15; i++) spiTx.buf[i]=0;
		//SPI_disableTx(halHandle->spiAHandle);
//		SPI_disableInt(halHandle->spiAHandle);
//		SPI_resetRxFifo(halHandle->spiAHandle);
//		SPI_enableRxFifo(halHandle->spiAHandle);
//		SPI_disableTxFifoInt(halHandle->spiAHandle);
//		SPI_enableRxFifoInt(halHandle->spiAHandle);
//		SPI_enableInt(halHandle->spiAHandle);
	}

	spiTx.cnt++;
	SPI_clearTxFifoInt(halHandle->spiAHandle);
	PIE_clearInt(obj->pieHandle,PIE_GroupNumber_6);
}


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


