/*********************************************************************
 *
 *       Uart routines to manage the communication with MCP39F511
 *
 *********************************************************************
 * FileName:        Uart_MCP39F511.h
 * Revision:        1.0.0 (First issue)
 * Date:            04/05/2019
 *
 * Dependencies:    Arduino.h
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

#ifndef _Uart_MCP39F511_H_INCLUDED__
#define _Uart_MCP39F511_H_INCLUDED__

#include "Arduino.h"
#include "Typedef_MCP39F511.h"

#define BAUD_2400       0x00
#define BAUD_4800       0x01
#define BAUD_9600       0x02
#define BAUD_19200      0x03
#define BAUD_38400      0x04
#define BAUD_57600      0x05
#define BAUD_115200     0x06
#define BAUD_230400     0x07
#define BAUD_250000     0x08
#define BAUD_500000     0x09
#define BAUD_1000000    0x0A

class Uart_MCP39F511 {
    
    public:                 
        uint8_t Uart_Array[64];         //  Array of byte used to send and receive data from device
        
        void  EnableDisableIdeMonitor(bool Set, uint8_t BaudRateSelector);    //  Enables/Disables and Sets BaudRate for serial interface used to debug code with IDE monitor
        void  SetBaudRateUart(bool Set, uint8_t BaudRateSelector);            //  Enables/Disables and Sets BaudRate for Software Seral interface
             
        void    UartSendData(uint8_t Lenght);                                 //  Send Data on the UART selected
        uint8_t UartReceivedData(void);                                       //  Data Read from UART
        
    private:
        void SetUartBegin(long BaudRate);
        void SetUartEnd(void);
};

extern Uart_MCP39F511 Uart;

#endif