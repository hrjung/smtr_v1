//###########################################################################
//
// FILE:   uartstdio.c
//
// TITLE:  Utility driver to provide simple UART console functions.
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
#include <stdlib.h>
#include <math.h>
#include "uartstdio.h"
#include "uart.h"

#define UART_BUFFERED

//*****************************************************************************
//
//! \addtogroup uartstdio_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// If buffered mode is defined, set aside RX and TX buffers and read/write
// pointers to control them.
//
//*****************************************************************************
#ifdef UART_BUFFERED

//*****************************************************************************
//
// This global controls whether or not we are echoing characters back to the
// transmitter.  By default, echo is enabled but if using this module as a
// convenient method of implementing a buffered serial interface over which
// you will be running an application protocol, you are likely to want to
// disable echo by calling UARTEchoSet(false).
//
//*****************************************************************************
static tBoolean g_bDisableEcho;
// needed to notify new command from PC, added ByPark
tBoolean g_bNewCmd = false;
extern HAL_Handle halHandle;
//*****************************************************************************
//
// Output ring buffer.  Buffer is full if g_ulUARTTxReadIndex is one ahead of
// g_ulUARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//
//*****************************************************************************
static unsigned char g_pcUARTTxBuffer[UART_TX_BUFFER_SIZE];
static volatile unsigned long g_ulUARTTxWriteIndex = 0;
static volatile unsigned long g_ulUARTTxReadIndex = 0;

//*****************************************************************************
//
// Input ring buffer.  Buffer is full if g_ulUARTTxReadIndex is one ahead of
// g_ulUARTTxWriteIndex.  Buffer is empty if the two indices are the same.
//
//*****************************************************************************
static unsigned char g_pcUARTRxBuffer[UART_RX_BUFFER_SIZE];
static volatile unsigned long g_ulUARTRxWriteIndex = 0;
static volatile unsigned long g_ulUARTRxReadIndex = 0;

unsigned char waveData[24];
bool printWaveFlag = false;

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the transmit buffer.
//
//*****************************************************************************
#define TX_BUFFER_USED          (GetBufferCount(&g_ulUARTTxReadIndex,  \
                                                &g_ulUARTTxWriteIndex, \
                                                UART_TX_BUFFER_SIZE))
#define TX_BUFFER_FREE          (UART_TX_BUFFER_SIZE - TX_BUFFER_USED)
#define TX_BUFFER_EMPTY         (IsBufferEmpty(&g_ulUARTTxReadIndex,   \
                                               &g_ulUARTTxWriteIndex))
#define TX_BUFFER_FULL          (IsBufferFull(&g_ulUARTTxReadIndex,  \
                                              &g_ulUARTTxWriteIndex, \
                                              UART_TX_BUFFER_SIZE))
#define ADVANCE_TX_BUFFER_INDEX(Index) \
                                (Index) = ((Index) + 1) % UART_TX_BUFFER_SIZE

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the receive buffer.
//
//*****************************************************************************
#define RX_BUFFER_USED          (GetBufferCount(&g_ulUARTRxReadIndex,  \
                                                &g_ulUARTRxWriteIndex, \
                                                UART_RX_BUFFER_SIZE))
#define RX_BUFFER_FREE          (UART_RX_BUFFER_SIZE - RX_BUFFER_USED)
#define RX_BUFFER_EMPTY         (IsBufferEmpty(&g_ulUARTRxReadIndex,   \
                                               &g_ulUARTRxWriteIndex))
#define RX_BUFFER_FULL          (IsBufferFull(&g_ulUARTRxReadIndex,  \
                                              &g_ulUARTRxWriteIndex, \
                                              UART_RX_BUFFER_SIZE))
#define ADVANCE_RX_BUFFER_INDEX(Index) \
                                (Index) = ((Index) + 1) % UART_RX_BUFFER_SIZE
#endif

//*****************************************************************************
//
// The base address of the chosen UART.
//
//*****************************************************************************
static SCI_Obj *g_ulSci = NULL;
static HAL_Obj *g_ulHal = NULL;
//*****************************************************************************
//
// A mapping from an integer between 0 and 15 to its ASCII character
// equivalent.
//
//*****************************************************************************
static const char * const g_pcHex = "0123456789abcdef";

#ifdef UART_BUFFERED
//*****************************************************************************
//
// The port number in use.
//
//*****************************************************************************
static int g_ulPortNum;
#endif

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is full or not.
//!
//! \param pulRead points to the read index for the buffer.
//! \param pulWrite points to the write index for the buffer.
//! \param ulSize is the size of the buffer in bytes.
//!
//! This function is used to determine whether or not a given ring buffer is
//! full.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is full or \b false otherwise.
//
//*****************************************************************************
#ifdef UART_BUFFERED
static tBoolean
IsBufferFull(volatile unsigned long *pulRead,
             volatile unsigned long *pulWrite, unsigned long ulSize)
{
    unsigned long ulWrite;
    unsigned long ulRead;

    ulWrite = *pulWrite;
    ulRead = *pulRead;

    return((((ulWrite + 1) % ulSize) == ulRead) ? true : false);
}
#endif

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is empty or not.
//!
//! \param pulRead points to the read index for the buffer.
//! \param pulWrite points to the write index for the buffer.
//!
//! This function is used to determine whether or not a given ring buffer is
//! empty.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is empty or \b false otherwise.
//
//*****************************************************************************
#ifdef UART_BUFFERED
static tBoolean
IsBufferEmpty(volatile unsigned long *pulRead,
              volatile unsigned long *pulWrite)
{
    unsigned long ulWrite;
    unsigned long ulRead;

    ulWrite = *pulWrite;
    ulRead = *pulRead;

    return((ulWrite  == ulRead) ? true : false);
}
#endif

//*****************************************************************************
//
//! Determines the number of bytes of data contained in a ring buffer.
//!
//! \param pulRead points to the read index for the buffer.
//! \param pulWrite points to the write index for the buffer.
//! \param ulSize is the size of the buffer in bytes.
//!
//! This function is used to determine how many bytes of data a given ring
//! buffer currently contains.  The structure of the code is specifically to
//! ensure that we do not see warnings from the compiler related to the order
//! of volatile accesses being undefined.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************
#ifdef UART_BUFFERED
static unsigned long
GetBufferCount(volatile unsigned long *pulRead,
               volatile unsigned long *pulWrite, unsigned long ulSize)
{
    unsigned long ulWrite;
    unsigned long ulRead;

    ulWrite = *pulWrite;
    ulRead = *pulRead;

    return((ulWrite >= ulRead) ? (ulWrite - ulRead) :
                                 (ulSize - (ulRead - ulWrite)));
}
#endif

//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the UART transmit FIFO.
//
//*****************************************************************************
#ifdef UART_BUFFERED
static void
UARTPrimeTransmit(SCI_Handle sciHandle)
{
    // Do we have any data to transmit?
    if(!TX_BUFFER_EMPTY)
    {
        // Disable the UART interrupt. If we don't do this there is a race
        // condition which can cause the read index to be corrupted.
        IntDisable(g_ulHal, g_ulPortNum, SCI_TX);    // TX disable

        // Yes - take some characters out of the transmit buffer and feed
        // them to the UART transmit FIFO.
        while(UARTSpaceAvail(sciHandle) && !TX_BUFFER_EMPTY)
        {
            UARTCharPutNonBlocking(sciHandle,
                                       g_pcUARTTxBuffer[g_ulUARTTxReadIndex]);
            ADVANCE_TX_BUFFER_INDEX(g_ulUARTTxReadIndex);
        }

        // Reenable the UART interrupt.
        IntEnable(g_ulHal, g_ulPortNum, SCI_TX);     // TX enable
    }
}
#endif

//*****************************************************************************
//
//! Initializes the UART console.
//!
//! \param ulPortNum is the number of UART port to use for the serial console
//! (0-2)
//!
//! This function will initialize the specified serial port to be used as a
//! serial console.  The serial parameters will be set to 115200, 8-N-1.
//! An application wishing to use a different baud rate may call
//! UARTStdioInitExpClk() instead of this function.
//!
//! This function or UARTStdioInitExpClk() must be called prior to using any
//! of the other UART console functions: UARTprintf() or UARTgets().  In order
//! for this function to work correctly, SysCtlClockSet() must be called prior
//! to calling this function.
//!
//! It is assumed that the caller has previously configured the relevant UART
//! pins for operation as a UART rather than as GPIOs.
//!
//! \return None.
//
//*****************************************************************************
void
UARTStdioInit(HAL_Handle handle, int ulPortNum)
{
    // Pass this call on to the version of the function allowing the baud rate
    // to be specified.
    UARTStdioInitExpClk(handle, ulPortNum, 115200);

    // needed for echo back, added ByPark
    UARTEchoSet(true);
}

//*****************************************************************************
//
//! Initializes the UART console and allows the baud rate to be selected.
//!
//! \param ulPortNum is the number of UART port to use for the serial console
//! (0-2)
//! \param ulBaud is the bit rate that the UART is to be configured to use.
//!
//! This function will initialize the specified serial port to be used as a
//! serial console.  The serial parameters will be set to 8-N-1 and the bit
//! rate set according to the value of the \e ulBaud parameter.
//!
//! This function or UARTStdioInit() must be called prior to using any of the
//! other UART console functions: UARTprintf() or UARTgets().  In order for
//! this function to work correctly, SysCtlClockSet() must be called prior to
//! calling this function.  An application wishing to use 115,200 baud may call
//! UARTStdioInit() instead of this function but should not call both
//! functions.
//!
//! It is assumed that the caller has previously configured the relevant UART
//! pins for operation as a UART rather than as GPIOs.
//!
//! \return None.
//
//*****************************************************************************
void UARTStdioInitExpClk(HAL_Handle handle, int ulPortNum, unsigned long ulBaud)
{
    HAL_Obj *obj = (HAL_Obj*)handle;
    SCI_Obj *sci;

#ifdef SUPPORT_V08_HW
    sci = (SCI_Obj*)obj->sciAHandle;
#else
    if(ulPortNum == SCI_A)
        sci = (SCI_Obj*)obj->sciAHandle;
    else
        sci = (SCI_Obj*)obj->sciBHandle;
#endif

    g_ulHal = handle;
    g_ulSci = sci;
    g_ulPortNum = ulPortNum;

    UARTConfigSetExpClk(sci, ulBaud);

#ifdef UART_BUFFERED
    // Set the UART to interrupt whenever the TX FIFO is almost empty or
    // when any character is received.
    //
    // FIFO not used, modified ByPark
    //UARTFIFOLevelSet(g_ulBase, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    // Flush both the buffers.
    UARTFlushRx(obj->cpuHandle);
    UARTFlushTx(obj->cpuHandle);

    // We are configured for buffered output so enable the master interrupt
    // for this UART and the receive interrupts.  We don't actually enable the
    // transmit interrupt in the UART itself until some data has been placed
    // in the transmit buffer.
    UARTIntDisable(sci, 0xFFFFFFFF);
    UARTIntEnable(sci, UART_INT_RXRDY_BRKDT | UART_INT_TXRDY);     // not using FIFO mode
#endif

    // to register ISR for SCI_A, added ByPark
    UARTTXIntRegister(handle, ulPortNum, &UARTStdioIntHandler);
    UARTRXIntRegister(handle, ulPortNum, &UARTStdioIntHandler);

    // Enable the UART operation.
    UARTEnable(sci);
}


//*****************************************************************************
//
//! Writes a string of characters to the UART output.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//! \param ulLen is the length of the string to transmit.
//!
//! This function will transmit the string to the UART output.  The number of
//! characters transmitted is determined by the \e ulLen parameter.  This
//! function does no interpretation or translation of any characters.  Since
//! the output is sent to a UART, any LF (/n) characters encountered will be
//! replaced with a CRLF pair.
//!
//! Besides using the \e ulLen parameter to stop transmitting the string, if a
//! null character (0) is encountered, then no more characters will be
//! transmitted and the function will return.
//!
//! In non-buffered mode, this function is blocking and will not return until
//! all the characters have been written to the output FIFO.  In buffered mode,
//! the characters are written to the UART transmit buffer and the call returns
//! immediately.  If insufficient space remains in the transmit buffer,
//! additional characters are discarded.
//!
//! \return Returns the count of characters written.
//
//*****************************************************************************
int
UARTwrite(unsigned char *pcBuf, unsigned long ulLen)
{
    unsigned int uIdx;

    // Send the characters
    for(uIdx = 0; uIdx < ulLen; uIdx++)
    {
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output.
        if(pcBuf[uIdx] == '\n')
        {
            if(!TX_BUFFER_FULL)
            {
                g_pcUARTTxBuffer[g_ulUARTTxWriteIndex] = '\r';
                ADVANCE_TX_BUFFER_INDEX(g_ulUARTTxWriteIndex);
            }
            else
            {
                // Buffer is full - discard remaining characters and return.
                break;
            }
        }

        // Send the character to the UART output.
        if(!TX_BUFFER_FULL)
        {
            g_pcUARTTxBuffer[g_ulUARTTxWriteIndex] = pcBuf[uIdx];
            ADVANCE_TX_BUFFER_INDEX(g_ulUARTTxWriteIndex);
        }
        else
        {
            // Buffer is full - discard remaining characters and return.
            break;
        }
    }

    // If we have anything in the buffer, make sure that the UART is set
    // up to transmit it.
    if(!TX_BUFFER_EMPTY)
    {
        UARTPrimeTransmit(g_ulSci);
    }

    // Return the number of characters written.
    return(uIdx);
}

//*****************************************************************************
//
//! A simple UART based get string function, with some line processing.
//!
//! \param pcBuf points to a buffer for the incoming string from the UART.
//! \param ulLen is the length of the buffer for storage of the string,
//! including the trailing 0.
//!
//! This function will receive a string from the UART input and store the
//! characters in the buffer pointed to by \e pcBuf.  The characters will
//! continue to be stored until a termination character is received.  The
//! termination characters are CR, LF, or ESC.  A CRLF pair is treated as a
//! single termination character.  The termination characters are not stored in
//! the string.  The string will be terminated with a 0 and the function will
//! return.
//!
//! In both buffered and unbuffered modes, this function will block until
//! a termination character is received.  If non-blocking operation is required
//! in buffered mode, a call to UARTPeek() may be made to determine whether
//! a termination character already exists in the receive buffer prior to
//! calling UARTgets().
//!
//! Since the string will be null terminated, the user must ensure that the
//! buffer is sized to allow for the additional null character.
//!
//! \return Returns the count of characters that were stored, not including
//! the trailing 0.
//
//*****************************************************************************
int
UARTgets(char *pcBuf, unsigned long ulLen)
{
    unsigned long ulCount = 0;
    char cChar;

    // Adjust the length back by 1 to leave space for the trailing
    // null terminator.
    ulLen--;

    // Process characters until a newline is received.
    while(1)
    {
        // Read the next character from the receive buffer.
        if(!RX_BUFFER_EMPTY)
        {
            cChar = g_pcUARTRxBuffer[g_ulUARTRxReadIndex];
            ADVANCE_RX_BUFFER_INDEX(g_ulUARTRxReadIndex);

            // See if a newline or escape character was received.
            if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
            {
                // Stop processing the input and end the line.
                break;
            }

            // Process the received character as long as we are not at the end
            // of the buffer.  If the end of the buffer has been reached then
            // all additional characters are ignored until a newline is
            // received.
            if(ulCount < ulLen)
            {
                // Store the character in the caller supplied buffer.
                pcBuf[ulCount] = cChar;

                // Increment the count of characters received.
                ulCount++;
            }
        }
    }

    // Add a null termination to the string.
    pcBuf[ulCount] = 0;

    // Return the count of chars in the buffer, not counting the trailing 0.
    return(ulCount);
}

//*****************************************************************************
//
//! Read a single character from the UART, blocking if necessary.
//!
//! This function will receive a single character from the UART and store it at
//! the supplied address.
//!
//! In both buffered and unbuffered modes, this function will block until a
//! character is received.  If non-blocking operation is required in buffered
//! mode, a call to UARTRxAvail() may be made to determine whether any
//! characters are currently available for reading.
//!
//! \return Returns the character read.
//
//*****************************************************************************
unsigned char
UARTgetc(void)
{
    unsigned char cChar;

    // Wait for a character to be received.
    while(RX_BUFFER_EMPTY)
    {
        // Block waiting for a character to be received (if the buffer is
        // currently empty).
    }

    // Read a character from the buffer.
    cChar = g_pcUARTRxBuffer[g_ulUARTRxReadIndex];
    ADVANCE_RX_BUFFER_INDEX(g_ulUARTRxReadIndex);

    // Return the character to the caller.
    return(cChar);
}


//hrjung add for putchar
int UARTwrite(unsigned char *pcBuf, unsigned long ulLen);
void UARTputc(char c)
{
	//UARTCharPut(g_ulBase, c);
	UARTwrite((unsigned char *)&c, 1);
}
//*****************************************************************************
//
//! A simple UART based printf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pcString is the format string.
//! \param ... are the optional arguments, which depend on the contents of the
//! format string.
//!
//! This function is very similar to the C library <tt>fprintf()</tt> function.
//! All of its output will be sent to the UART.  Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%l to print unsigned long
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ``\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ``\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments after \e pcString must match the requirements of
//! the format string.  For example, if an integer was passed where a string
//! was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void
UARTprintf(char *pcString, ...)
{
    unsigned long ulIdx, ulValue, ulPos, ulCount, ulBase, ulNeg;
    char cFill;
    unsigned char *pcStr, pcBuf[16];
    va_list vaArgP;

#ifdef FLOAT_PRINT
    float fValue;
    char *fStr;
#endif

    va_start(vaArgP, pcString);

    // Loop while there are more characters in the string.
    while(*pcString)
    {
        // Find the first non-% character, or the end of the string.
        for(ulIdx = 0; (pcString[ulIdx] != '%') && (pcString[ulIdx] != '\0');
            ulIdx++)
        {
        }

        // Write this portion of the string.
        UARTwrite((unsigned char *)pcString, ulIdx);

        // Skip the portion of the string that was written.
        pcString += ulIdx;

        // See if the next character is a %.
        if(*pcString == '%')
        {
            // Skip the %.
            pcString++;

            // Set the digit count to zero, and the fill character to space
            // (i.e. to the defaults).
            ulCount = 0;
            cFill = ' ';

            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
again:

            // Determine how to handle the next character.
            switch(*pcString++)
            {
                // Handle the digit characters.
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    if((pcString[-1] == '0') && (ulCount == 0))
                    {
                        cFill = '0';
                    }

                    // Update the digit count.
                    ulCount *= 10;
                    ulCount += pcString[-1] - '0';

                    // Get the next character.
                    goto again;
                }

                // Handle the %c command.
                case 'c':
                {
                    // Get the value from the varargs.
                    ulValue = va_arg(vaArgP, unsigned int);

                    // Print out the character.
                    UARTwrite((unsigned char *)&ulValue, 1);

                    // This command has been handled.
                    break;
                }

                // Handle the %d command.
                case 'i':
                case 'd':
                {
                    // Get the value from the varargs.
                    ulValue = va_arg(vaArgP, unsigned int);

                    // Reset the buffer position.
                    ulPos = 0;

                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    if((long)ulValue < 0)
                    {
                        // Make the value positive.
                        ulValue = -(long)ulValue;

                        // Indicate that the value is negative.
                        ulNeg = 1;
                    }
                    else
                    {
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        ulNeg = 0;
                    }

                    // Set the base to 10.
                    ulBase = 10;

                    // Convert the value to ASCII.
                    goto convert;
                }

                // Handle the %l command.
                case 'l':
                {
                	// Get the value from the varargs.
                	ulValue = va_arg(vaArgP, unsigned long);

                	// Reset the buffer position.
                	ulPos = 0;

                	// If the value is negative, make it positive and indicate
                	// that a minus sign is needed.
                	if((long)ulValue < 0)
                	{
                		// Make the value positive.
                		ulValue = -(long)ulValue;

                		// Indicate that the value is negative.
                		ulNeg = 1;
                	}
                	else
                	{
                		// Indicate that the value is positive so that a minus
                		// sign isn't inserted.
                		ulNeg = 0;
                	}

                	// Set the base to 10.
                	ulBase = 10;

                	// Convert the value to ASCII.
                	goto convert;
                }

                // Handle the %s command.
                case 's':
                {
                    // Get the string pointer from the varargs.
                    pcStr = (unsigned char *)va_arg(vaArgP, char *);

                    // Determine the length of the string.
                    for(ulIdx = 0; pcStr[ulIdx] != '\0'; ulIdx++)
                    {
                    }

                    // Write the string.
                    UARTwrite(pcStr, ulIdx);

                    // Write any required padding spaces
                    if(ulCount > ulIdx)
                    {
                        ulCount -= ulIdx;
                        while(ulCount--)
                        {
                            UARTwrite(" ", 1);
                        }
                    }
                    // This command has been handled.
                    break;
                }

                // Handle the %u command.
                case 'u':
                {
                    // Get the value from the varargs.
                    ulValue = va_arg(vaArgP, unsigned int);

                    // Reset the buffer position.
                    ulPos = 0;

                    // Set the base to 10.
                    ulBase = 10;

                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    ulNeg = 0;

                    // Convert the value to ASCII.
                    goto convert;
                }

                // Handle the %x and %X commands.  Note that they are treated
                // identically; i.e. %X will use lower case letters for a-f
                // instead of the upper case letters is should use.  We also
                // alias %p to %x.
                case 'x':
                case 'X':
                case 'p':
                {
                    // Get the value from the varargs.
                    ulValue = va_arg(vaArgP, unsigned long);

                    // Reset the buffer position.
                    ulPos = 0;

                    // Set the base to 16.
                    ulBase = 16;

                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    ulNeg = 0;

                    // Determine the number of digits in the string version of
                    // the value.
convert:
                    for(ulIdx = 1;
                        (((ulIdx * ulBase) <= ulValue) &&
                         (((ulIdx * ulBase) / ulBase) == ulIdx));
                        ulIdx *= ulBase, ulCount--)
                    {
                    }

                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    if(ulNeg)
                    {
                        ulCount--;
                    }

                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    if(ulNeg && (cFill == '0'))
                    {
                        // Place the minus sign in the output buffer.
                        pcBuf[ulPos++] = '-';

                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        ulNeg = 0;
                    }

                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    if((ulCount > 1) && (ulCount < 16))
                    {
                        for(ulCount--; ulCount; ulCount--)
                        {
                            pcBuf[ulPos++] = cFill;
                        }
                    }

                    // If the value is negative, then place the minus sign
                    // before the number.
                    if(ulNeg)
                    {
                        // Place the minus sign in the output buffer.
                        pcBuf[ulPos++] = '-';
                    }

                    // Convert the value into a string.
                    for(; ulIdx; ulIdx /= ulBase)
                    {
                        pcBuf[ulPos++] = g_pcHex[(ulValue / ulIdx) % ulBase];
                    }

                    // Write the string.
                    UARTwrite(pcBuf, ulPos);

                    // This command has been handled.
                    break;
                }
#ifdef FLOAT_PRINT
                // Handle the %f command.
                case 'f':
                {
                    // Get the value from the varargs.
                    fValue = va_arg(vaArgP, float);
                    fStr = (char*)malloc(15);
                    memset(fStr, NULL, 15);
                    ftoa(fValue, fStr, 3);
                    ulPos = 15;
                    UARTwrite((unsigned char *)fStr, ulPos);
                    free(fStr);

                    break;
                }
#endif // #ifdef FLOAT_PRINT

                // Handle the %% command.
                case '%':
                {
                    // Simply write a single %.
                    UARTwrite((unsigned char *)(pcString - 1), 1);

                    // This command has been handled.
                    break;
                }

                // Handle all other commands.
                default:
                {
                    // Indicate an error.
                    UARTwrite("ERROR", 5);

                    // This command has been handled.
                    break;
                }
            }
        }
    }

    // End the varargs processing.
    va_end(vaArgP);
}

//*****************************************************************************
//
//! Returns the number of bytes available in the receive buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to determine the number
//! of bytes of data currently available in the receive buffer.
//!
//! \return Returns the number of available bytes.
//
//*****************************************************************************
#if defined(UART_BUFFERED) || defined(DOXYGEN)
int
UARTRxBytesAvail(void)
{
    return(RX_BUFFER_USED);
}
#endif

#if defined(UART_BUFFERED) || defined(DOXYGEN)
//*****************************************************************************
//
//! Returns the number of bytes free in the transmit buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to determine the amount
//! of space currently available in the transmit buffer.
//!
//! \return Returns the number of free bytes.
//
//*****************************************************************************
int
UARTTxBytesFree(void)
{
    return(TX_BUFFER_FREE);
}
#endif

//*****************************************************************************
//
//! Flushes the receive buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to discard any data
//! received from the UART but not yet read using UARTgets().
//!
//! \return None.
//
//*****************************************************************************
#if defined(UART_BUFFERED) || defined(DOXYGEN)
void
UARTFlushRx(CPU_Handle cpuHandle)
{
    // Temporarily turn off interrupts.
    CPU_Obj *cpu = (CPU_Obj*)cpuHandle;

    CPU_disableGlobalInts(cpu);

    // Flush the receive buffer.
    g_ulUARTRxReadIndex = 0;
    g_ulUARTRxWriteIndex = 0;

    // If interrupts were enabled when we turned them off, turn them
    // back on again.
    CPU_enableGlobalInts(cpu);
}
#endif

//*****************************************************************************
//
//! Flushes the transmit buffer.
//!
//! \param bDiscard indicates whether any remaining data in the buffer should
//! be discarded (\b true) or transmitted (\b false).
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to flush the transmit
//! buffer, either discarding or transmitting any data received via calls to
//! UARTprintf() that is waiting to be transmitted.  On return, the transmit
//! buffer will be empty.
//!
//! \return None.
//
//*****************************************************************************
#if defined(UART_BUFFERED) || defined(DOXYGEN)
void UARTFlushTx(CPU_Handle cpuHandle)
{
    CPU_Obj *cpu = (CPU_Obj*) cpuHandle;

    // The remaining data should be discarded, so temporarily turn off interrupts.
    CPU_disableGlobalInts(cpu);

    // Flush the transmit buffer.
    g_ulUARTTxReadIndex = 0;
    g_ulUARTTxWriteIndex = 0;

    // If interrupts were enabled when we turned them off, turn them
    // back on again.
    CPU_enableGlobalInts(cpu);
}
#endif

//*****************************************************************************
//
//! Enables or disables echoing of received characters to the transmitter.
//!
//! \param bEnable must be set to \b true to enable echo or \b false to
//! disable it.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b UART_BUFFERED, may be used to control whether or not
//! received characters are automatically echoed back to the transmitter.  By
//! default, echo is enabled and this is typically the desired behavior if
//! the module is being used to support a serial command line.  In applications
//! where this module is being used to provide a convenient, buffered serial
//! interface over which application-specific binary protocols are being run,
//! however, echo may be undesirable and this function can be used to disable
//! it.
//!
//! \return None.
//
//*****************************************************************************
#if defined(UART_BUFFERED) || defined(DOXYGEN)
void
UARTEchoSet(tBoolean bEnable)
{
    g_bDisableEcho = !bEnable;
}
#endif

//*****************************************************************************
//
//! Handles UART interrupts.
//!
//! This function handles interrupts from the UART.  It will copy data from the
//! transmit buffer to the UART transmit FIFO if space is available, and it
//! will copy data from the UART receive FIFO to the receive buffer if data is
//! available.
//!
//! \return None.
//
//*****************************************************************************
#if defined(UART_BUFFERED) || defined(DOXYGEN)
__interrupt void
UARTStdioIntHandler(void)
{
    unsigned long ulInts;
    char cChar;
    long lChar;
    static tBoolean bLastWasCR = false;
    HAL_Obj* obj = (HAL_Obj*)halHandle;

    // Get and clear the current interrupt source(s)
    ulInts = UARTIntStatus(g_ulSci, true);
    UARTIntClear(g_ulSci, ulInts);

    // Are we being interrupted because the TX FIFO has space available?
    if(ulInts & UART_INT_TXRDY)
    {
        // Move as many bytes as we can into the transmit FIFO.
        UARTPrimeTransmit(g_ulSci);

        // If the output buffer is empty, turn off the transmit interrupt.
        if(TX_BUFFER_EMPTY)
            IntDisable(g_ulHal, g_ulPortNum, SCI_TX);
    }

    // Are we being interrupted due to a received character?
    if(ulInts & UART_INT_RXRDY_BRKDT)
    {
        // Get all the available characters from the UART.
        while(UARTCharsAvail(g_ulSci))
        {
            // Read a character
            lChar = UARTCharGetNonBlocking(g_ulSci);
            cChar = (unsigned char)(lChar & 0xFF);

            // If echo is disabled, we skip the various text filtering
            // operations that would typically be required when supporting a
            // command line.
            if(!g_bDisableEcho)
            {
                // Handle backspace by erasing the last character in the buffer.
                // ByPark,
                if(cChar == '\b')
                {
                    // If there are any characters already in the buffer, then
                    // delete the last.
                    if(!RX_BUFFER_EMPTY)
                    {
                        // Rub out the previous character on the users terminal.
                        UARTwrite("\b \b", 3);

                        // Decrement the number of characters in the buffer.
                        if(g_ulUARTRxWriteIndex == 0)
                        {
                            g_ulUARTRxWriteIndex = UART_RX_BUFFER_SIZE - 1;
                        }
                        else
                        {
                            g_ulUARTRxWriteIndex--;
                        }
                    }

                    // Skip ahead to read the next character.
                    continue;
                }

                // If this character is LF and last was CR, then just gobble up
                // the character since we already echoed the previous CR and we
                // don't want to store 2 characters in the buffer if we don't
                // need to.
                //
                // commented out ByPark
                //if((cChar == '\n') && bLastWasCR)
                //{
                //   bLastWasCR = false;
                //   continue;
                //}

                // See if a newline or escape character was received.
                if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
                {
                    // If the character is a CR, then it may be followed by an
                    // LF which should be paired with the CR.  So remember that
                    // a CR was received.
                    if(cChar == '\r')
                    {
                        bLastWasCR = 1;
                    }

                    // Regardless of the line termination character received,
                    // put a CR in the receive buffer as a marker telling
                    // UARTgets() where the line ends.  We also send an
                    // additional LF to ensure that the local terminal echo
                    // receives both CR and LF.
                        cChar = '\r';
                        UARTwrite("\n", 1);
                }
            }

            // If there is space in the receive buffer, put the character
            // there, otherwise throw it away.
            //
            if(!RX_BUFFER_FULL)
            {
                //
                // Store the new character in the receive buffer
                //
                g_pcUARTRxBuffer[g_ulUARTRxWriteIndex] =
                        (unsigned char)(lChar & 0xFF);
                ADVANCE_RX_BUFFER_INDEX(g_ulUARTRxWriteIndex);

                //
                // If echo is enabled, write the character to the transmit
                // buffer so that the user gets some immediate feedback.
                if(!g_bDisableEcho)
                {
                    UARTwrite((unsigned char *)&cChar, 1);
                }

                // added ByPark for CR processing
                if(cChar == '\r')
                    g_bNewCmd = 1;
            }
        }

        // If we wrote anything to the transmit buffer, make sure it actually
        // gets transmitted.
        UARTPrimeTransmit(g_ulSci);
        // not needed, modified ByPark
        //UARTIntEnable(g_ulBase, UART_INT_TX);
    }
    // to ACK PIE interrupt,  added ByPark
    //    PieCtrlRegs.PIEACK.all = 0x0100;
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);
}
#endif

#ifdef FLOAT_PRINT
// Converts a floating point number to string.
void ftoa(float f, char *buffer, long length)
{
    long i = 0;
    long num = 0;
    long pos = 0;
    long dec = 0;

    if(f < 0)
    {
        buffer[pos++] = '-';
        f *= -1;
    }

    dec = f;

    while(dec > 0)
    {
        num++;
        dec /= 10;
    }

    dec = f;
    f = (f-dec) * 10;

    if(num == 0)
    {
        buffer[pos++] = '0';
    }
    else
    {
        for(i = num; i > 0; i--)
        {
            buffer[i-1 + pos] = dec % 10 + '0';
            dec /= 10;
        }

        pos += num;
    }

    if(f == 0.0f)
        return;
    else
    {
        if(length > 0)
            buffer[pos++] = '.';

        for(i = 0; i < length; i++)
        {
            long value = f;
            buffer[pos++] = value + '0';
            f = (f-value) * 10;
            if(f == 0.0f)
                break;
        }
    }
}
#endif // #ifdef FLOAT_PRINT

#if 0
//*****************************************************************************
//
//! Print wave data
//!
//! This function sends wave data to PC to draw waveform.
//!
//! \return None.
//
//*****************************************************************************
int SendWave(unsigned char *fData, int fCnt)
{
    unsigned char *pData;
    unsigned char *bData;
    int bCnt = 0, i = 0;
    unsigned short checksum = 0;

    pData = (unsigned char*)malloc(fCnt+2);
    bData = (unsigned char*)malloc((fCnt+2)*2+2);

    pData[0] = fCnt;

    for(i=0;i<fCnt;i++)
    {
        pData[i+1] = fData[i];
        checksum += fData[i];
    }
    pData[fCnt+1] = 0xFF - (checksum & 0xFF);

    //Binary Frame
    bData[bCnt++] = 0x7E;

    for(i=0;i<(fCnt+2);i++)
    {
        if((pData[i]==0x7E) || (pData[i]==0x7D) || (pData[i]==0x0D) || (pData[i]==0x0A)  || (pData[i]==0x08) || (pData[i] == 0x1B)) {
            bData[bCnt++] = 0x7D;
            bData[bCnt++] = pData[i] ^ 0x20;
        }
        else
            bData[bCnt++] = pData[i];
    }

    bData[bCnt++] = 0x0D;

    UARTwrite((unsigned char*)bData, bCnt);

    free(pData);
    free(bData);

    return bCnt;
}

void PrintWave(int ch0_mode, void *ch0_data, int ch1_mode, void *ch1_data, int ch2_mode, void *ch2_data)
{
    union FloatParam    fdata;
    union LongParam     ldata;
    union IntParam      idata;
    //int nodata = 0;
    int index = 1;
    unsigned char format = 0;

    // channel 0
    if(ch0_mode==INT_TYPE) {
        idata.ival = *(int*)ch0_data;
        waveData[index++] = idata.cval >> 8;
        waveData[index++] = idata.cval & 0xFF;
        format = format | 1;
    }
    else if(ch0_mode==LONG_TYPE) {
        ldata.lval = *(long*)ch0_data;
        waveData[index++] = ldata.cval[0] >> 8;
        waveData[index++] = ldata.cval[0] & 0xFF;
        waveData[index++] = ldata.cval[1] >> 8;
        waveData[index++] = ldata.cval[1] & 0xFF;
        format = format | 2;
    }
    else if(ch0_mode==FLOAT_TYPE) {
        fdata.fval=*(float*)ch0_data;
        waveData[index++] = fdata.cval[0] >> 8;
        waveData[index++] = fdata.cval[0] & 0xFF;
        waveData[index++] = fdata.cval[1] >> 8;
        waveData[index++] = fdata.cval[1] & 0xFF;
        format = format | 3;
    }

    // channel 1
    if(ch1_mode==INT_TYPE) {
        idata.ival = *(int*)ch1_data;
        waveData[index++] = idata.cval >> 8;
        waveData[index++] = idata.cval & 0xFF;
        format = format | (1 << 2);
    }
    else if(ch1_mode==LONG_TYPE) {
        ldata.lval = *(long*)ch1_data;
        waveData[index++] = ldata.cval[0] >> 8;
        waveData[index++] = ldata.cval[0] & 0xFF;
        waveData[index++] = ldata.cval[1] >> 8;
        waveData[index++] = ldata.cval[1] & 0xFF;
        format = format | (2 << 2);
    }
    else if(ch1_mode==FLOAT_TYPE) {
        fdata.fval=*(float*)ch1_data;
        waveData[index++] = fdata.cval[0] >> 8;
        waveData[index++] = fdata.cval[0] & 0xFF;
        waveData[index++] = fdata.cval[1] >> 8;
        waveData[index++] = fdata.cval[1] & 0xFF;
        format = format | (3 << 2);
    }

    // channel 2
    if(ch2_mode==INT_TYPE) {
        idata.ival = *(int*)ch2_data;
        waveData[index++] = idata.cval >> 8;
        waveData[index++] = idata.cval & 0xFF;
        format = format | (1 << 4);
    }
    else if(ch2_mode==LONG_TYPE) {
        ldata.lval = *(long*)ch2_data;
        waveData[index++] = ldata.cval[0] >> 8;
        waveData[index++] = ldata.cval[0] & 0xFF;
        waveData[index++] = ldata.cval[1] >> 8;
        waveData[index++] = ldata.cval[1] & 0xFF;
        format = format | (2 << 4);
    }
    else if(ch2_mode==FLOAT_TYPE) {
        fdata.fval=*(float*)ch2_data;
        waveData[index++] = fdata.cval[0] >> 8;
        waveData[index++] = fdata.cval[0] & 0xFF;
        waveData[index++] = fdata.cval[1] >> 8;
        waveData[index++] = fdata.cval[1] & 0xFF;
        format = format | (3 << 4);
    }

    waveData[0] = format;
    SendWave(waveData, index);
}
#endif

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


