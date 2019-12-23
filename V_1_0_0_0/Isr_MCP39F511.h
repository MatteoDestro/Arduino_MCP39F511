/*********************************************************************
 *
 *       Internet service routine
 *
 *********************************************************************
 * FileName:        Isr_MCP39F511.h
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

#ifndef _ISR_MCP39F511_H_INCLUDED__
#define _ISR_MCP39F511_H_INCLUDED__

#include "Arduino.h"
#include "Typedef_MCP39F511.h"

//=======================================================================
#define TIMEBASE 2 

#define T_6MSEC     (6/TIMEBASE)      //  Defines a constant for a timeout of 6 mSec
#define T_8MSEC     (8/TIMEBASE)      //  Defines a constant for a timeout of 8 mSec
#define T_10MSEC    (10/TIMEBASE)     //  Defines a constant for a timeout of 10 mSec
#define T_20MSEC    (20/TIMEBASE)     //  Defines a constant for a timeout of 20 mSec
#define T_50MSEC    (50/TIMEBASE)     //  Defines a constant for a timeout of 50 mSec
#define T_90MSEC    (90/TIMEBASE)     //  Defines a constant for a timeout of 90 mSec
#define T_100MSEC   (100/TIMEBASE)    //  Defines a constant for a timeout of 100 mSec
#define T_500MSEC   (500/TIMEBASE)    //  Defines a constant for a timeout of 500 mSec

#define T_1SEC      (1000/TIMEBASE)   //  Defines a constant for a timeout of 1,000  Sec
#define T_2SEC      (2000/TIMEBASE)   //  Defines a constant for a timeout of 2,000  Sec
#define T_3SEC      (3000/TIMEBASE)   //  Defines a constant for a timeout of 3,000  Sec
#define T_4SEC      (4000/TIMEBASE)   //  Defines a constant for a timeout of 4,000  Sec
#define T_5SEC      (5000/TIMEBASE)   //  Defines a constant for a timeout of 5,000  Sec
#define T_6SEC      (6000/TIMEBASE)   //  Defines a constant for a timeout of 5,000  Sec
#define T_7SEC      (7000/TIMEBASE)   //  Defines a constant for a timeout of 6,000  Sec
#define T_8SEC      (8000/TIMEBASE)   //  Defines a constant for a timeout of 7,000  Sec
#define T_9SEC      (9000/TIMEBASE)   //  Defines a constant for a timeout of 9,000  Sec
#define T_10SEC     (10000/TIMEBASE)  //  Defines a constant for a timeout of 10,000 Sec
#define T_15SEC     (15000/TIMEBASE)  //  Defines a constant for a timeout of 15,000 Sec
//=======================================================================

//=======================================================================
#define Fosc            (unsigned long)16000000                     //  Clock 16MHz
#define PRE             (unsigned long)256                          //  Prescaler 256
#define MSEC            2                                           //  Time Base: 2mSec
#define SLOWBASETIME_T1 (0xFFFF - ((Fosc * MSEC) / (PRE * 1000)))
//=======================================================================

class Isr_MCP39F511 {
    public:
#if defined(HARDWARE_UART1) || defined(SOFTWARE_UART1)
        uint16_t Uart1_TimeOut;             //  Serial TimeOut                      
#endif
#if defined(HARDWARE_UART2) || defined(SOFTWARE_UART2)
        uint16_t Uart2_TimeOut;             //  Serial TimeOut                      
#endif
#if defined(HARDWARE_UART3) || defined(SOFTWARE_UART3)
        uint16_t Uart3_TimeOut;             //  Serial TimeOut                      
#endif         
        uint8_t  ZcdTimeOut;                //  TimeOut ZCD
        
        void EnableLibInterrupt(void);      //  Enable Library Interrupt
        
    private:
        void EnableTimerInterrupt(void);    //  Enable and set TIMER1 interrupt
        void EnableZcdIntx(void);           //  Enable and set INTx interrupt
        void EnableZcdPortCx(void);         //  Enable and set PORTCx interrupt
};

extern Isr_MCP39F511 IsrMCP39F511;

#endif