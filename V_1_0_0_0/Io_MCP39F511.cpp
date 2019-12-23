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
#include "Isr_MCP39F511.h"
#include "Io_MCP39F511.h"
#include "Uart_MCP39F511.h"

#ifdef __AVR__
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#endif

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/****************************************************************************
 * Function:        MCP39F511_SetControlLine
 *
 * Overview:        This function is used to set MCP39F511 Control Line 
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
void Io_MCP39F511::MCP39F511_SetControlLine(void) {
    pinMode(MCP39F511_RESET, OUTPUT);
    pinMode(MCP39F511_CLEAR, OUTPUT);
    pinMode(MCP39F511_ZCD,   INPUT);
    
    digitalWrite(MCP39F511_RESET, LOW); //  Pin active LOW. Reset pin for Delta-Sigma ADCs
    digitalWrite(MCP39F511_CLEAR, LOW); //  Pin active LOW. Master Clear for device   
    
    Cmd.GenericFlags.ZCD_Error = 1;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ResetLine
 *
 * Overview:        This function is used to manage Reset pin for Delta-Sigma ADCs MCP39F511
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
void Io_MCP39F511::MCP39F511_ResetLine(uint8_t LineState) {
    if (LineState == 0) {
        digitalWrite(MCP39F511_RESET, LOW);
    } else {
        digitalWrite(MCP39F511_RESET, HIGH);
    }
}
/****************************************************************************/

 /****************************************************************************
 * Function:        MCP39F511_MclrLine
 *
 * Overview:        This function is used to manage Clear pin (Master Clear For Device)
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
void Io_MCP39F511::MCP39F511_MclrLine(uint8_t LineState) {
    if (LineState == 0) {
        digitalWrite(MCP39F511_CLEAR, LOW);
    } else {
        digitalWrite(MCP39F511_CLEAR, HIGH);
    }
}
/****************************************************************************/