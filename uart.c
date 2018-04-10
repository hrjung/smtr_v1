//###########################################################################
//
// FILE:   uart.c
//
// TITLE:  Stellaris style wrapper driver for C28x SCI peripheral.
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//*****************************************************************************
//
//! \addtogroup uart_api
//! @{
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "uart.h"

//*****************************************************************************
//
//! Sets the configuration of a UART.
//!
//! \param ulBase is the base address of the UART port.
//! \param ulUARTClk is the rate of the clock supplied to the UART module.
//! \param ulBaud is the desired baud rate.
//! \param ulConfig is the data format for the port (number of data bits,
//! number of stop bits, and parity).
//!
//! This function configures the UART for operation in the specified data
//! format.  The baud rate is provided in the \e ulBaud parameter and the data
//! format in the \e ulConfig parameter.
//!
//! The \e ulConfig parameter is the logical OR of three values: the number of
//! data bits, the number of stop bits, and the parity.  \b UART_CONFIG_WLEN_8,
//! \b UART_CONFIG_WLEN_7, \b UART_CONFIG_WLEN_6, and \b UART_CONFIG_WLEN_5
//! select from eight to five data bits per byte (respectively).
//! \b UART_CONFIG_STOP_ONE and \b UART_CONFIG_STOP_TWO select one or two stop
//! bits (respectively).  \b UART_CONFIG_PAR_NONE, \b UART_CONFIG_PAR_EVEN,
//! \b UART_CONFIG_PAR_ODD, \b UART_CONFIG_PAR_ONE, and \b UART_CONFIG_PAR_ZERO
//! select the parity mode (no parity bit, even parity bit, odd parity bit,
//! parity bit always one, and parity bit always zero, respectively).
//!
//! The peripheral clock will be the same as the processor clock.  This will be
//! the value returned by SysCtlClockGet(), or it can be explicitly hard coded
//! if it is constant and known (to save the code/execution overhead of a call
//! to SysCtlClockGet()).
//!
//! This function replaces the original UARTConfigSet() API and performs the
//! same actions.  A macro is provided in <tt>uart.h</tt> to map the original
//! API to this API.
//!
//! \return None.
//
//*****************************************************************************
//Changed for C28x
void
UARTConfigSetExpClk(SCI_Handle sciHandle, unsigned long ulBaud)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
//    unsigned long ulDiv;

    // Stop the UART.
    UARTDisable(sci);

    // Compute the baud rate divider.
//    ulDiv = ((90,000,000  / (ulBaud * 8)) - 1);

    SCI_disableParity(sci);
    SCI_setNumStopBits(sci, SCI_NumStopBits_One);
    SCI_setCharLength(sci, SCI_CharLength_8_Bits);
    // set baud rate to 115200
    if(ulBaud == 115200)
        SCI_setBaudRate(sci, (SCI_BaudRate_e)(0x0061));
    else if(ulBaud == 9600)
        SCI_setBaudRate(sci, (SCI_BaudRate_e)(0x00493));
    SCI_setPriority(sci, SCI_Priority_FreeRun);

    // Start the UART.
    UARTEnable(sci);
}

//*****************************************************************************
//
//! Enables transmitting and receiving.
//!
//! \param ulBase is the base address of the UART port.
//!
//! Sets the UARTEN, TXE, and RXE bits, and enables the transmit and receive
//! FIFOs.
//!
//! \return None.
//
//*****************************************************************************
void
UARTEnable(SCI_Handle sciHandle)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;

    // Enable RX, TX, and the UART.
    SCI_enableTx(sci);
    SCI_enableRx(sci);
    SCI_enable(sci);
}

//*****************************************************************************
//
//! Disables transmitting and receiving.
//!
//! \param ulBase is the base address of the UART port.
//!
//! Clears the UARTEN, TXE, and RXE bits, then waits for the end of
//! transmission of the current character, and flushes the transmit FIFO.
//!
//! \return None.
//
//*****************************************************************************
void
UARTDisable(SCI_Handle sciHandle)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
    // Wait for end of TX.
    while(!(sci->SCICTL2 & SCI_SCICTL2_TXEMPTY_BITS))
    {
    }

    // Disable the FIFO.
    SCI_disableTxFifoEnh(sci);

    // Disable the UART.
    SCI_disableTx(sci);
    SCI_disableRx(sci);
}

//*****************************************************************************
//
//! Determines if there are any characters in the receive FIFO.
//!
//! \param ulBase is the base address of the UART port.
//!
//! This function returns a flag indicating whether or not there is data
//! available in the receive FIFO.
//!
//! \return Returns \b true if there is data in the receive FIFO or \b false
//! if there is no data in the receive FIFO.
//
//*****************************************************************************
tBoolean
UARTCharsAvail(SCI_Handle sciHandle)
{
    // Return the availability of characters.
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
    bool status;

    status = SCI_rxDataReady(sci);

    return status;
}

//*****************************************************************************
//
//! Determines if there is any space in the transmit FIFO.
//!
//! \param ulBase is the base address of the UART port.
//!
//! This function returns a flag indicating whether or not there is space
//! available in the transmit FIFO.
//!
//! \return Returns \b true if there is space available in the transmit FIFO
//! or \b false if there is no space available in the transmit FIFO.
//
//*****************************************************************************
tBoolean
UARTSpaceAvail(SCI_Handle sciHandle)
{
    // Return the availability of space.
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
    bool status;

    status = SCI_txReady(sci);

    return status;
}

//*****************************************************************************
//
//! Receives a character from the specified port.
//!
//! \param ulBase is the base address of the UART port.
//!
//! Gets a character from the receive FIFO for the specified port.
//!
//! This function replaces the original UARTCharNonBlockingGet() API and
//! performs the same actions.  A macro is provided in <tt>uart.h</tt> to map
//! the original API to this API.
//!
//! \return Returns the character read from the specified port, cast as a
//! \e long.  A \b -1 is returned if there are no characters present in the
//! receive FIFO.  The UARTCharsAvail() function should be called before
//! attempting to call this function.
//
//*****************************************************************************
long
UARTCharGetNonBlocking(SCI_Handle sciHandle)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
    uint16_t dataRx, success;

    // See if there are any characters in the receive FIFO.
    dataRx = SCI_getDataNonBlocking(sci, &success);
    // Read and return the next character.
   if(success)
        return dataRx;
    else
        return (-1);
}

//*****************************************************************************
//
//! Waits for a character from the specified port.
//!
//! \param ulBase is the base address of the UART port.
//!
//! Gets a character from the receive FIFO for the specified port.  If there
//! are no characters available, this function waits until a character is
//! received before returning.
//!
//! \return Returns the character read from the specified port, cast as a
//! \e long.
//
//*****************************************************************************
long
UARTCharGet(SCI_Handle sciHandle)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
    uint16_t data;

    // Wait until a char is available.
    while(!SCI_rxDataReady(sci))
    {
    }

    // Now get the char.
    data = SCI_read(sci);

    return data;
}

//*****************************************************************************
//
//! Sends a character to the specified port.
//!
//! \param ulBase is the base address of the UART port.
//! \param ucData is the character to be transmitted.
//!
//! Writes the character \e ucData to the transmit FIFO for the specified port.
//! This function does not block, so if there is no space available, then a
//! \b false is returned, and the application must retry the function later.
//!
//! This function replaces the original UARTCharNonBlockingPut() API and
//! performs the same actions.  A macro is provided in <tt>uart.h</tt> to map
//! the original API to this API.
//!
//! \return Returns \b true if the character was successfully placed in the
//! transmit FIFO or \b false if there was no space available in the transmit
//! FIFO.
//
//*****************************************************************************
tBoolean
UARTCharPutNonBlocking(SCI_Handle sciHandle, unsigned char ucData)
{
    // See if there is space in the transmit FIFO.
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
    return(SCI_putDataNonBlocking(sci, ucData));
}

//*****************************************************************************
//
//! Waits to send a character from the specified port.
//!
//! \param ulBase is the base address of the UART port.
//! \param ucData is the character to be transmitted.
//!
//! Sends the character \e ucData to the transmit FIFO for the specified port.
//! If there is no space available in the transmit FIFO, this function waits
//! until there is space available before returning.
//!
//! \return None.
//
//*****************************************************************************
void
UARTCharPut(SCI_Handle sciHandle, unsigned char ucData)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;

    // Wait until space is available.
    while(!SCI_txReady(sci));
    {
    }

    // Send the char.
    SCI_write(sci, ucData);
}

//*****************************************************************************
//
//! Determines whether the UART transmitter is busy or not.
//!
//! \param ulBase is the base address of the UART port.
//!
//! Allows the caller to determine whether all transmitted bytes have cleared
//! the transmitter hardware.  If \b false is returned, the transmit FIFO is
//! empty and all bits of the last transmitted character, including all stop
//! bits, have left the hardware shift register.
//!
//! \return Returns \b true if the UART is transmitting or \b false if all
//! transmissions are complete.
//
//*****************************************************************************
tBoolean
UARTBusy(SCI_Handle sciHandle)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;

    return ((sci->SCICTL2 & SCI_SCICTL2_TXEMPTY_BITS) ? false : true);
}

    //*****************************************************************************
//
//! Registers an interrupt handler for a UART RX interrupt.
//!
//! \param ulBase is the base address of the UART port.
//! \param pfnHandler is a pointer to the function to be called when the
//! UART interrupt occurs.
//!
//! This function does the actual registering of the interrupt handler.  This
//! will enable the global interrupt in the interrupt controller; specific UART
//! interrupts must be enabled via UARTIntEnable().  It is the interrupt
//! handler's responsibility to clear the interrupt source.
//!
//! \sa IntRegister() for important information about registering interrupt
//! handlers.
//!
//! \return None.
//
//*****************************************************************************
void
UARTRXIntRegister(HAL_Handle handle, int ulPortNum, void (*pfnHandler)(void))
{
    HAL_Obj *obj = (HAL_Obj*)handle;
    PIE_Obj *pie = (PIE_Obj*)obj->pieHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Register the interrupt handler.
    if(ulPortNum == SCI_A)
        pie->SCIRXINTA = pfnHandler;
    else
        pie->SCIRXINTB = pfnHandler;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Enable the UART interrupt.
    IntEnable(handle, ulPortNum, SCI_RX);
}

//*****************************************************************************
//
//! Registers an interrupt handler for a UART TX interrupt.
//!
//! \param ulBase is the base address of the UART port.
//! \param pfnHandler is a pointer to the function to be called when the
//! UART interrupt occurs.
//!
//! This function does the actual registering of the interrupt handler.  This
//! will enable the global interrupt in the interrupt controller; specific UART
//! interrupts must be enabled via UARTIntEnable().  It is the interrupt
//! handler's responsibility to clear the interrupt source.
//!
//! \sa IntRegister() for important information about registering interrupt
//! handlers.
//!
//! \return None.
//
//*****************************************************************************
void
UARTTXIntRegister(HAL_Handle handle, int ulPortNum, void (*pfnHandler)(void))
{
    HAL_Obj *obj = (HAL_Obj*)handle;
    PIE_Obj *pie = (PIE_Obj*)obj->pieHandle;

    ENABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Register the interrupt handler.
    if(ulPortNum == SCI_A)
        pie->SCITXINTA = pfnHandler;
    else
        pie->SCITXINTB = pfnHandler;

    DISABLE_PROTECTED_REGISTER_WRITE_MODE;

    // Enable the UART interrupt.
    IntEnable(handle, ulPortNum, SCI_TX);
}

//*****************************************************************************
//
//! Enables individual UART interrupt sources.
//!
//! \param ulBase is the base address of the UART port.
//! \param ulIntFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated UART interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! The \e ulIntFlags parameter is the logical OR of any of the following:
//!
//! - \b UART_INT_OE - Overrun Error interrupt
//! - \b UART_INT_BE - Break Error interrupt
//! - \b UART_INT_PE - Parity Error interrupt
//! - \b UART_INT_FE - Framing Error interrupt
//! - \b UART_INT_RT - Receive Timeout interrupt
//! - \b UART_INT_TX - Transmit interrupt
//! - \b UART_INT_RX - Receive interrupt
//! - \b UART_INT_DSR - DSR interrupt
//! - \b UART_INT_DCD - DCD interrupt
//! - \b UART_INT_CTS - CTS interrupt
//! - \b UART_INT_RI - RI interrupt
//!
//! \return None.
//
//*****************************************************************************
void
UARTIntEnable(SCI_Handle sciHandle, unsigned long ulIntFlags)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;

    // Enable the specified interrupts.
    if(ulIntFlags & UART_INT_RXERR)
        SCI_enableRxErrorInt(sci);

    if(ulIntFlags & UART_INT_RXRDY_BRKDT)
        SCI_enableRxInt(sci);

    if(ulIntFlags & UART_INT_TXRDY)
        SCI_enableTxInt(sci);

    if(ulIntFlags & UART_INT_TXFF)
        SCI_enableTxFifoInt(sci);

    if(ulIntFlags & UART_INT_RXFF)
        SCI_enableRxFifoInt(sci);
}

//*****************************************************************************
//
//! Disables individual UART interrupt sources.
//!
//! \param ulBase is the base address of the UART port.
//! \param ulIntFlags is the bit mask of the interrupt sources to be disabled.
//!
//! Disables the indicated UART interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! The \e ulIntFlags parameter has the same definition as the \e ulIntFlags
//! parameter to UARTIntEnable().
//!
//! \return None.
//
//*****************************************************************************
void
UARTIntDisable(SCI_Handle sciHandle, unsigned long ulIntFlags)
{
    SCI_Obj *sci = (SCI_Obj*) sciHandle;

    // Disable the specified interrupts.
    if(ulIntFlags & UART_INT_RXERR)
        SCI_disableRxErrorInt(sci);

    if(ulIntFlags & UART_INT_RXRDY_BRKDT)
        SCI_disableRxInt(sci);
        
    if(ulIntFlags & UART_INT_TXRDY)
        SCI_disableTxInt(sci);

    if(ulIntFlags & UART_INT_TXFF)
        SCI_disableTxFifoInt(sci);

    if(ulIntFlags & UART_INT_RXFF)
        SCI_disableRxFifoInt(sci);
}

//*****************************************************************************
//
//! Gets the current interrupt status.
//!
//! \param ulBase is the base address of the UART port.
//! \param bMasked is \b false if the raw interrupt status is required and
//! \b true if the masked interrupt status is required.
//!
//! This returns the interrupt status for the specified UART.  Either the raw
//! interrupt status or the status of interrupts that are allowed to reflect to
//! the processor can be returned.
//!
//! \return Returns the current interrupt status, enumerated as a bit field of
//! values described in UARTIntEnable().
//
//*****************************************************************************
unsigned long
UARTIntStatus(SCI_Handle sciHandle, tBoolean bMasked)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;
    unsigned long temp = 0;

    // Return either the interrupt status or the raw interrupt status as
    // requested.
    if(SCI_txReady(sci))
        temp |= UART_INT_TXRDY;

    if(sci->SCIRXST & SCI_SCIRXST_RXERROR_BITS)
        temp |= UART_INT_RXERR;

    if(SCI_rxDataReady(sci) | (sci->SCIRXST & SCI_SCIRXST_BRKDT_BITS))
        temp |= UART_INT_RXRDY_BRKDT;

    if(sci->SCIFFTX & SCI_SCIFFTX_INT_BITS)
        temp |= UART_INT_TXFF;

    if(sci->SCIFFRX & SCI_SCIFFRX_INT_BITS)
        temp |= UART_INT_RXFF;

    return temp;
}

//*****************************************************************************
//
//! Clears UART interrupt sources.
//!
//! \param ulBase is the base address of the UART port.
//! \param ulIntFlags is a bit mask of the interrupt sources to be cleared.
//!
//! The specified UART interrupt sources are cleared, so that they no longer
//! assert.  This function must be called in the interrupt handler to keep the
//! interrupt from being recognized again immediately upon exit.
//!
//! The \e ulIntFlags parameter has the same definition as the \e ulIntFlags
//! parameter to UARTIntEnable().
//!
//! \note Because there is a write buffer in the Cortex-M3 processor, it may
//! take several clock cycles before the interrupt source is actually cleared.
//! Therefore, it is recommended that the interrupt source be cleared early in
//! the interrupt handler (as opposed to the very last action) to avoid
//! returning from the interrupt handler before the interrupt source is
//! actually cleared.  Failure to do so may result in the interrupt handler
//! being immediately reentered (because the interrupt controller still sees
//! the interrupt source asserted).
//!
//! \return None.
//
//*****************************************************************************
void
UARTIntClear(SCI_Handle sciHandle, unsigned long ulIntFlags)
{
    SCI_Obj *sci = (SCI_Obj*)sciHandle;

    // Clear the requested interrupt sources.
    // SWRST reset RXBUF, modified ByPark
    if(ulIntFlags & (UART_INT_RXERR))
    {
        SCI_reset(sci);
        __asm(" nop");
        __asm(" nop");
        __asm(" nop");
        __asm(" nop");
        SCI_enable(sci);
    }

    if(ulIntFlags & UART_INT_TXFF)
        SCI_clearTxFifoInt(sci);

    if(ulIntFlags & UART_INT_RXFF)
        SCI_clearRxFifoInt(sci);
}

//*****************************************************************************
//
//! Enables the processor interrupt.
//!
//! Allows the processor to respond to interrupts.  This does not affect the
//! set of interrupts enabled in the interrupt controller; it just gates the
//! single interrupt from the controller to the processor.
//!
//! \note Previously, this function had no return value.  As such, it was
//! possible to include <tt>interrupt.h</tt> and call this function without
//! having included <tt>hw_types.h</tt>.  Now that the return is a
//! <tt>tBoolean</tt>, a compiler error will occur in this case.  The solution
//! is to include <tt>hw_types.h</tt> before including <tt>interrupt.h</tt>.
//!
//! \return Returns \b true if interrupts were disabled when the function was
//! called or \b false if they were initially enabled.
//
//*****************************************************************************
tBoolean
IntMasterEnable(void)
{
    // Enable processor interrupts.
   __asm(" CLRC INTM");

    //TODO: Return previous interrupt status
    return 0;
}

//*****************************************************************************
//
//! Disables the processor interrupt.
//!
//! Prevents the processor from receiving interrupts.  This does not affect the
//! set of interrupts enabled in the interrupt controller; it just gates the
//! single interrupt from the controller to the processor.
//!
//! \note Previously, this function had no return value.  As such, it was
//! possible to include <tt>interrupt.h</tt> and call this function without
//! having included <tt>hw_types.h</tt>.  Now that the return is a
//! <tt>tBoolean</tt>, a compiler error will occur in this case.  The solution
//! is to include <tt>hw_types.h</tt> before including <tt>interrupt.h</tt>.
//!
//! \return Returns \b true if interrupts were already disabled when the
//! function was called or \b false if they were initially enabled.
//
//*****************************************************************************
tBoolean
IntMasterDisable(void)
{
    // Disable processor interrupts.
   __asm(" SETC INTM");

    //TODO: Return previous interrupt status
    return 0;
}

//*****************************************************************************
//
//! Enables an interrupt.
//!
//! \param ulInterrupt specifies the interrupt to be enabled.
//!
//! The specified interrupt is enabled in the interrupt controller.  Other
//! enables for the interrupt (such as at the peripheral level) are unaffected
//! by this function.
//!
//! \return None.
//
//*****************************************************************************
void
IntEnable(HAL_Handle handle, int ulPortNum, int intMode)
{
    HAL_Obj *obj = (HAL_Obj*)handle;
    CPU_Obj *cpu = (CPU_Obj*)obj->cpuHandle;
    PIE_Obj *pie = (PIE_Obj*)obj->pieHandle;

    CPU_enableProtectedRegisterWrite(cpu);

    //Ensure that PIE is enabled
    PIE_enable(pie);

    //Enable Individual PIE interrupt
    if(ulPortNum == SCI_A) {
        if(intMode==SCI_TX)
            PIE_enableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIATX);
        else    // SCI_RX
            PIE_enableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIARX);
    }
    else {      // SCI_B
        if(intMode==SCI_TX)
            PIE_enableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIBTX);
        else    // SCI_RX
            PIE_enableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIBRX);
    }

    //Enable PIE Group Interrupt
    CPU_enableInt(cpu, CPU_IntNumber_9);

    CPU_disableProtectedRegisterWrite(cpu);
}

//*****************************************************************************
//
//! Disables an interrupt.
//!
//! \param ulInterrupt specifies the interrupt to be disabled.
//!
//! The specified interrupt is disabled in the interrupt controller.  Other
//! enables for the interrupt (such as at the peripheral level) are unaffected
//! by this function.
//!
//! \return None.
//
//*****************************************************************************
void
IntDisable(HAL_Handle handle, int ulPortNum, int intMode)
{
    HAL_Obj *obj = (HAL_Obj*)handle;
    CPU_Obj *cpu = (CPU_Obj*)obj->cpuHandle;
    PIE_Obj *pie = (PIE_Obj*)obj->pieHandle;
    
    CPU_enableProtectedRegisterWrite(cpu);
    
    if(ulPortNum == SCI_A) {
        if(intMode==SCI_TX)
            PIE_disableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIATX);
        else    // SCI_RX
            PIE_disableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIARX);
    }
    else {      // SCI_B
        if(intMode==SCI_TX)
            PIE_disableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIBTX);
        else    // SCI_RX
            PIE_disableInt(pie, PIE_GroupNumber_9, PIE_InterruptSource_SCIBRX);
    }


    CPU_disableProtectedRegisterWrite(cpu);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


