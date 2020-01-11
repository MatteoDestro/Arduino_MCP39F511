/*********************************************************************
 *
 *       I/O Board configuration
 *
 *********************************************************************
 * FileName:        Io_MCP39F511.h
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

#ifndef _IO_MCP39F511_H_INCLUDED__
#define _IO_MCP39F511_H_INCLUDED__

#include "Arduino.h"
#include "Typedef_MCP39F511.h"

//======================================================================
//  Selects Arduino Board
//#define ARDUINO_UNO_REV3
#define ARDUINO_MEGA2560_REV3
//======================================================================

//======================================================================
//  Selects the Hardware or Software serial port
//#define SOFTWARE_UART1_39F511
//#define HARDWARE_UART1_39F511
//#define SOFTWARE_UART2_39F511
//#define HARDWARE_UART2_39F511
//#define SOFTWARE_UART3_39F511
#define HARDWARE_UART3_39F511
//======================================================================

//======================================================================
//  Select External Interrupt INT0 to INT7 to manage Zero Cross Detection
//#define ZCD_INT0
#define ZCD_INT1
//#define ZCD_INT2
//#define ZCD_INT3
//#define ZCD_INT4
//#define ZCD_INT5
//======================================================================

//======================================================================
//  Select External pin change interrupt to manage Zero Cross Detection
//#define ZCD_PCINT0
//#define ZCD_PCINT1
//#define ZCD_PCINT2
//#define ZCD_PCINT3
//#define ZCD_PCINT4
//#define ZCD_PCINT5
//#define ZCD_PCINT6
//#define ZCD_PCINT7
//#define ZCD_PCINT8
//#define ZCD_PCINT9
//#define ZCD_PCINT10
//#define ZCD_PCINT11
//#define ZCD_PCINT12
//#define ZCD_PCINT13
//#define ZCD_PCINT14
//#define ZCD_PCINT15
//#define ZCD_PCINT16
//#define ZCD_PCINT17
//#define ZCD_PCINT18
//#define ZCD_PCINT19
//#define ZCD_PCINT20
//#define ZCD_PCINT21
//#define ZCD_PCINT22
//#define ZCD_PCINT23
//======================================================================

//======================================================================
//  Output I/O
#ifdef ARDUINO_MEGA2560_REV3
    #define MCP39F511_RESET  45      //  MCP39F511 Reset line
    #define MCP39F511_CLEAR  46      //  MCP39F511 Clear line
    //#define MCP39F511_ZCD    A12     //  MCP39F511 ZCD (PCINT20)
    #define MCP39F511_ZCD    20     //  MCP39F511 ZCD (INT1)
#endif
#ifdef ARDUINO_UNO_REV3
    #define MCP39F511_RESET  6       //  MCP39F511 Reset line
    #define MCP39F511_CLEAR  5       //  MCP39F511 Clear line
    //#define MCP39F511_ZCD    4       //  MCP39F511 ZCD (PCINT20)
    #define MCP39F511_ZCD    3     //  MCP39F511 ZCD (INT1)    
#endif
//======================================================================

//======================================================================
//  Input I/O
#ifdef ARDUINO_MEGA2560_REV3
    #ifdef HARDWARE_UART1_39F511
        #define PIN_TX1_HW  18      //  UART1 HARDWARE TX
        #define PIN_RX1_HW  19      //  UART1 HARDWARE RX    
    #endif
    #ifdef HARDWARE_UART2_39F511
        #define PIN_TX2_HW  16      //  UART2 HARDWARE TX
        #define PIN_RX2_HW  17      //  UART2 HARDWARE RX    
    #endif    
    #ifdef HARDWARE_UART3_39F511
        #define PIN_TX3_HW  14      //  UART3 HARDWARE TX
        #define PIN_RX3_HW  15      //  UART3 HARDWARE RX    
    #endif

    #ifdef SOFTWARE_UART1_39F511
        #define PIN_TX1_SW  A8      //  UART1 SOFTWARE TX
        #define PIN_RX1_SW  A9      //  UART1 SOFTWARE RX
    #endif
    #ifdef SOFTWARE_UART2_39F511
        #define PIN_TX2_SW  A10     //  UART2 SOFTWARE TX
        #define PIN_RX2_SW  A11     //  UART2 SOFTWARE RX
    #endif    
    #ifdef SOFTWARE_UART3_39F511
        #define PIN_TX3_SW  14      //  UART3 SOFTWARE TX
        #define PIN_RX3_SW  15      //  UART3 SOFTWARE RX
    #endif
#endif
#ifdef ARDUINO_UNO_REV3
    #if defined(HARDWARE_UART2_39F511) || defined(HARDWARE_UART3_39F511)
        #error "On Arduino UNO R3 the hardware UART 2 and UART 3 are not available!"
    #endif
    #ifdef HARDWARE_UART1_39F511
        #warning "On the Arduino UNO R3 the hardware UART 1 colud be used for debug then the UART 1 could be busy!"
    #endif
    
    #ifdef SOFTWARE_UART1_39F511
        #define PIN_TX1_SW  A0      //  UART1 SOFTWARE TX
        #define PIN_RX1_SW  A1      //  UART1 SOFTWARE RX
    #endif
    #ifdef SOFTWARE_UART2_39F511
        #define PIN_TX2_SW  A2      //  UART2 SOFTWARE TX
        #define PIN_RX2_SW  A3      //  UART2 SOFTWARE RX
    #endif     
    #ifdef SOFTWARE_UART3_39F511
        #define PIN_TX3_SW  A4      //  UART3 SOFTWARE TX
        #define PIN_RX3_SW  A5      //  UART3 SOFTWARE RX
    #endif
#endif
//======================================================================

//======================================================================
//  NULL CHAR
#define NULL_CHAR               0x00    //  Dec -> 000 (Null)
#define ASCII_CARRIAGE_RET      0x0D    //  Dec -> 013 ('\r')   <CR>
//======================================================================

class Io_MCP39F511 {  
    public:
        void MCP39F511_SetControlLine(void);
        void MCP39F511_ResetLine(uint8_t LineState);
        void MCP39F511_MclrLine(uint8_t LineState);
        
    private:

};

extern Io_MCP39F511 Io_39F511;

#endif