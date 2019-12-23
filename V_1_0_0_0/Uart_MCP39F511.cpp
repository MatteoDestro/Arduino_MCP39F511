/*********************************************************************
 *
 *       Uart routines to manage the communication with MCP39F511
 *
 *********************************************************************
 * FileName:        Uart_MCP39F511.c
 * Revision:        1.0.0 (First issue)
 * Date:            04/05/2019
 *
 * Dependencies:    Arduino.h
 *                  Cmd_MCP39F511.h
 *                  Io_MCP39F511.h
 *                  Isr_MCP39F511.h
 *                  Uart_MCP39F511.h
 *
 * Arduino Board:   Arduino Uno, Arduino Mega 2560, Fishino Uno, Fishino Mega 2560       
 *
 * Company:         Futura Group srl
 *                  www.Futurashop.it
 *                  www.open-electronics.org
 *
 * Developer:       Destro Matteo
 *
 * Support:         info@open-electronics.org
 * 
 * Software License Agreement
 *
 * Copyright (c) 2016, Futura Group srl 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 *  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************/

#include "Cmd_MCP39F511.h"
#include "Io_MCP39F511.h"
#include "Isr_MCP39F511.h"
#include "Uart_MCP39F511.h"

#if defined(SOFTWARE_UART1) || defined(SOFTWARE_UART2) || defined(SOFTWARE_UART3)
    #include <SoftwareSerial.h>
#endif

#ifdef __AVR__
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#endif

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef SOFTWARE_UART1
    SoftwareSerial Serial1_SW = SoftwareSerial(PIN_RX1_SW, PIN_TX1_SW);
#endif
#ifdef SOFTWARE_UART2
    SoftwareSerial Serial2_SW = SoftwareSerial(PIN_RX2_SW, PIN_TX2_SW);
#endif
#ifdef SOFTWARE_UART3
    SoftwareSerial Serial3_SW = SoftwareSerial(PIN_RX3_SW, PIN_TX3_SW);
#endif

/****************************************************************************
 * Function:        EnableDisableIdeMonitor
 *
 * Overview:        This routine is used to Enable/Disable the serial port for debug
 *                  with IDE monitor
 *
 * PreCondition:    None
 *
 * Input:           Set: If "true" enable the serial debug, if "false" disable debug
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
void Uart_MCP39F511::EnableDisableIdeMonitor(bool Set, uint8_t BaudRateSelector) {
    if (Set == true) {
        Serial.setTimeout(200);
        switch (BaudRateSelector)
        {
            case BAUD_9600:
                Serial.begin(9600);     //  Debug Serial
                break;
            case BAUD_19200:
                Serial.begin(19200);    //  Debug Serial
                break;
            case BAUD_38400:
                Serial.begin(38400);    //  Debug Serial
                break;
            case BAUD_57600:
                Serial.begin(57600);    //  Debug Serial
                break;
            case BAUD_115200:
                Serial.begin(115200);   //  Debug Serial
                break;  
            case BAUD_230400:
                Serial.begin(230400);   //  Debug Serial
                break;
            case BAUD_250000:
                Serial.begin(250000);   //  Debug Serial
                break;
            case BAUD_500000:
                Serial.begin(500000);   //  Debug Serial
                break;
            case BAUD_1000000:
                Serial.begin(1000000);  //  Debug Serial
                break;
            default:
                Serial.begin(57600);    //  Debug Serial
                break;          
        }
    } else {
        Serial.end();           
    }
}
/****************************************************************************/

/****************************************************************************
 * Function:        SetBaudRateUart3
 *
 * Overview:        This routine is used to Initialize the Serial port (Software/Hardware mode)
 *
 * PreCondition:    None
 *
 * Input:           BaudRateSelector: Bauderate code. Use defined directive
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
void Uart_MCP39F511::SetBaudRateUart(bool Set, uint8_t BaudRateSelector) {
    if (Set == true) {
        switch (BaudRateSelector)
        {
            case BAUD_2400:
                SetUartBegin((long)2400);
                break;
            case BAUD_4800:
                SetUartBegin((long)4800);
                break;
            case BAUD_9600:
                SetUartBegin((long)9600);
                break;
            case BAUD_19200:
                SetUartBegin((long)19600);
                break;
            case BAUD_38400:
                SetUartBegin((long)38400);
                break;
            case BAUD_57600:
                SetUartBegin((long)57600);
                break;
            case BAUD_115200:
                SetUartBegin((long)115200);
                break;  
            default:
                SetUartBegin((long)115200);
                break;
        }       
    } else {
        SetUartEnd();
    }
}

void Uart_MCP39F511::SetUartBegin(long BaudRate) {
#ifdef SOFTWARE_UART1
    Serial1_SW.begin(BaudRate);
#endif
#ifdef HARDWARE_UART1
    Serial1.begin(BaudRate);
#endif
#ifdef SOFTWARE_UART2
    Serial2_SW.begin(BaudRate);
#endif
#ifdef HARDWARE_UART2
    Serial2.begin(BaudRate);
#endif
#ifdef SOFTWARE_UART3
    Serial3_SW.begin(BaudRate);
#endif
#ifdef HARDWARE_UART3
    Serial3.begin(BaudRate);
#endif    
}
void Uart_MCP39F511::SetUartEnd(void) {
#ifdef SOFTWARE_UART1
    Serial1_SW.end();
#endif
#ifdef HARDWARE_UART1
    Serial1.end();
#endif
#ifdef SOFTWARE_UART2
    Serial2_SW.end();
#endif
#ifdef HARDWARE_UART2
    Serial2.end();
#endif
#ifdef SOFTWARE_UART3
    Serial3_SW.end();
#endif
#ifdef HARDWARE_UART3
    Serial3.end();
#endif    
}
/****************************************************************************/

/****************************************************************************
 * Function:        UartSendData
 *
 * Overview:        Uart Send Data Mode state
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
void Uart_MCP39F511::UartSendData(uint8_t DataLenght) {
    uint8_t * UartArrayPointer;
    uint8_t Counter;
        
    UartArrayPointer = &Uart.Uart_Array[0];
    Counter = 0;
    do {
#ifdef SOFTWARE_UART1
        Serial1_SW.write(*(uint8_t *)UartArrayPointer);
#endif
#ifdef SOFTWARE_UART2
        Serial2_SW.write(*(uint8_t *)UartArrayPointer);
#endif
#ifdef SOFTWARE_UART3
        Serial3_SW.write(*(uint8_t *)UartArrayPointer);
#endif


#ifdef HARDWARE_UART1
        Serial1.write(*(uint8_t *)UartArrayPointer);
#endif
#ifdef HARDWARE_UART2
        Serial2.write(*(uint8_t *)UartArrayPointer);
#endif
#ifdef HARDWARE_UART3
        Serial3.write(*(uint8_t *)UartArrayPointer);
#endif
        *UartArrayPointer++ = NULL_CHAR;
    } while (++Counter < DataLenght);  
}
/****************************************************************************/

/****************************************************************************
 * Function:        UartReceiveData
 *
 * Overview:        This function received data from serial port
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Command Note:    None
 *
 * Output:          Return the number of byte wrote on the serial port 
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Uart_MCP39F511::UartReceivedData(void) {
    uint8_t * UartArrayPointer;
    uint8_t Counter = 0;
    int     c = 0;

    UartArrayPointer = &Uart.Uart_Array[0];
#ifdef SOFTWARE_UART1
    Isr.Uart1_TimeOut = T_100MSEC;
    while (Serial1_SW.available() || (Isr.Uart1_TimeOut > 0)) {
        c = Serial1_SW.read();
        if (c != -1) {
            if (Counter < (sizeof(Uart.Uart_Array) - 1)) {
                *(uint8_t *)UartArrayPointer++ = c;
                *(uint8_t *)UartArrayPointer   = NULL_CHAR;
                Counter++;
            }   
            Isr.Uart1_TimeOut = T_100MSEC;
        }
    }
#endif
#ifdef SOFTWARE_UART2
    Isr.Uart2_TimeOut = T_100MSEC;
    while (Serial2_SW.available() || (Isr.Uart2_TimeOut > 0)) {
        c = Serial2_SW.read();
        if (c != -1) {
            if (Counter < (sizeof(Uart.Uart_Array) - 1)) {
                *(uint8_t *)UartArrayPointer++ = c;
                *(uint8_t *)UartArrayPointer   = NULL_CHAR;
                Counter++;
            }   
            Isr.Uart2_TimeOut = T_100MSEC;
        }
    }
#endif
#ifdef SOFTWARE_UART3
    Isr.Uart3_TimeOut = T_100MSEC;
    while (Serial3_SW.available() || (Isr.Uart3_TimeOut > 0)) {
        c = Serial3_SW.read();
        if (c != -1) {
            if (Counter < (sizeof(Uart.Uart_Array) - 1)) {
                *(uint8_t *)UartArrayPointer++ = c;
                *(uint8_t *)UartArrayPointer   = NULL_CHAR;
                Counter++;
            }   
            Isr.Uart3_TimeOut = T_100MSEC;
        }
    }
#endif


#ifdef HARDWARE_UART1
    Isr.Uart1_TimeOut = T_100MSEC;
    while (Serial1.available() || (Isr.Uart1_TimeOut > 0)) {
        c = Serial1.read();
        if (c != -1) {
            if (Counter < (sizeof(Uart.Uart_Array) - 1)) {
                *(uint8_t *)UartArrayPointer++ = c;
                *(uint8_t *)UartArrayPointer   = NULL_CHAR;
                Counter++;
            }
            Isr.Uart1_TimeOut = T_100MSEC;
        }
    }
#endif
#ifdef HARDWARE_UART2
    Isr.Uart2_TimeOut = T_100MSEC;
    while (Serial2.available() || (Isr.Uart2_TimeOut > 0)) {
        c = Serial2.read();
        if (c != -1) {
            if (Counter < (sizeof(Uart.Uart_Array) - 1)) {
                *(uint8_t *)UartArrayPointer++ = c;
                *(uint8_t *)UartArrayPointer   = NULL_CHAR;
                Counter++;
            }
            Isr.Uart2_TimeOut = T_100MSEC;
        }
    }
#endif
#ifdef HARDWARE_UART3
    Isr.Uart3_TimeOut = T_100MSEC;
    while (Serial3.available() || (Isr.Uart3_TimeOut > 0)) {
        c = Serial3.read();
        if (c != -1) {
            if (Counter < (sizeof(Uart.Uart_Array) - 1)) {
                *(uint8_t *)UartArrayPointer++ = c;
                *(uint8_t *)UartArrayPointer   = NULL_CHAR;
                Counter++;
            }
            Isr.Uart3_TimeOut = T_100MSEC;
        }
    }
#endif
    return(Counter);
}
/****************************************************************************/