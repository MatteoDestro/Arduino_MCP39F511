/*********************************************************************
 *
 *       Internet service routine
 *
 *********************************************************************
 * FileName:        Isr_MCP39F511.h
 * Revision:        1.0.0 (First issue)
 * Date:            04/05/2019
 *
 * Revision:        1.1.0
 *                  01/12/2019
 *                  - Added function "static volatile void Isr_MCP39F511::Timer1_Interrupt(void)" to manage interrupt TIMER 1
 *                    from another library
 *                  - Added "ENABLE_ISR_TIMER1" compiler directive to enable/disable ISR TIMER 1 code
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

#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/****************************************************************************
 * Function:        EnableLibInterrupt
 *
 * Overview:        This function enables Timer1 Interrupt and INT0 Interrupt (Or INT4 interrupt for Arduino Mega 2560)
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
void Isr_MCP39F511::EnableLibInterrupt(void) {
#ifdef ENABLE_ISR_TIMER1    
    Isr_39F511.EnableTimerInterrupt();
#endif
#if defined(ZCD_INT0) || defined(ZCD_INT1) || defined(ZCD_INT2)|| defined(ZCD_INT3)|| defined(ZCD_INT4)|| defined(ZCD_INT5)    
    Isr_39F511.EnableZcdIntx();
#endif    
#if defined(ZCD_PCINT0)  || defined(ZCD_PCINT1)  || defined(ZCD_PCINT2)  || defined(ZCD_PCINT3)  || defined(ZCD_PCINT4)  || defined(ZCD_PCINT5)  || defined(ZCD_PCINT6)  || defined(ZCD_PCINT7)  || \
    defined(ZCD_PCINT8)  || defined(ZCD_PCINT9)  || defined(ZCD_PCINT10) || defined(ZCD_PCINT11) || defined(ZCD_PCINT12) || defined(ZCD_PCINT13) || defined(ZCD_PCINT14) || defined(ZCD_PCINT15) || \
    defined(ZCD_PCINT16) || defined(ZCD_PCINT17) || defined(ZCD_PCINT18) || defined(ZCD_PCINT19) || defined(ZCD_PCINT20) || defined(ZCD_PCINT21) || defined(ZCD_PCINT22) || defined(ZCD_PCINT23)
    Isr_39F511.EnableZcdPortCx();
#endif        
}
/****************************************************************************/

/****************************************************************************
 * Function:        EnableTimerInterrupt
 *
 * Overview:        This function enable and sets the Timer1 interrupt
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
#ifdef ENABLE_ISR_TIMER1
    void Isr_MCP39F511::EnableTimerInterrupt(void) {
        cli();                  // disable all interrupts
    #ifdef ARDUINO_UNO_REV3
        TCCR1A = 0x00;
        TCCR1B = 0x00;
        TCNT1   = SLOWBASETIME_T1;
        TCCR1B |= 0x04;         // Prescaler 256
        TIMSK1 |= 0x01;         // enable oveflow timer interrupt
    #endif
    #ifdef ARDUINO_MEGA2560_REV3
        TCCR1A = 0x00;
        TCCR1B = 0x00;
        TCCR1C = 0x00;
        TCNT1  = SLOWBASETIME_T1;
        TCCR1B = 0x04;
        TIMSK1 = 0x01;          // enable oveflow timer interrupt
    #endif
        sei();                  // enable all interrupts
    }
#endif
/****************************************************************************/
    
/****************************************************************************
 * Function:        Enable External Intx Interrupt (Zero Cross Detection)
 *
 * Overview:        This function enable ZCD external interrupt on INTx for Arduino Uno R3 and INTx for ArduinoMega 2560 R3
 *
 * PreCondition:    None
 *
 * GSM cmd syntax:  None    
 *
 * Input:           None
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * GSM answer det:  None
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
void Isr_MCP39F511::EnableZcdIntx(void) {
    cli();                      // disable all interrupts
#ifdef ZCD_INT0
    EIMSK  = 0x00;
    EIMSK |= (1 << INT0);       // Enable external interrupt INT0
    EICRA  = 0x00;
    EICRA |= (1 << ISC00);
    EICRA |= (1 << ISC01);      // The Rising edge of INT0 generates an interrupt request
#endif
#ifdef ZCD_INT1
    EIMSK  = 0x00;
    EIMSK |= (1 << INT1);       // Enable external interrupt INT1
    EICRA  = 0x00;
    EICRA |= (1 << ISC10);
    EICRA |= (1 << ISC11);      // The Rising edge of INT1 generates an interrupt request
#endif
#ifdef ZCD_INT2
    EIMSK  = 0x00;
    EIMSK |= (1 << INT2);       // Enable external interrupt INT2
    EICRA  = 0x00;
    EICRA |= (1 << ISC20);
    EICRA |= (1 << ISC21);      // The Rising edge of INT2 generates an interrupt request
#endif
#ifdef ZCD_INT3
    EIMSK  = 0x00;
    EIMSK |= (1 << INT3);       // Enable external interrupt INT3
    EICRA  = 0x00;
    EICRA |= (1 << ISC30);
    EICRA |= (1 << ISC31);      // The Rising edge of INT3 generates an interrupt request
#endif
#ifdef ZCD_INT4
    EIMSK  = 0x00;
    EIMSK |= (1 << INT4);       // Enable external interrupt INT4
    EICRB  = 0x00;
    EICRB |= (1 << ISC40);
    EICRB |= (1 << ISC41);      // The Rising edge of INT4 generates an interrupt request
#endif
#ifdef ZCD_INT5
    EIMSK  = 0x00;
    EIMSK |= (1 << INT5);       // Enable external interrupt INT5
    EICRB  = 0x00;
    EICRB |= (1 << ISC50);
    EICRB |= (1 << ISC51);      // The Rising edge of INT5 generates an interrupt request
#endif
    sei();                      // enable all interrupts
}
/****************************************************************************/

/****************************************************************************
 * Function:        Enable PCINTx Interrupt (Zero Cross Detection)
 *
 * Overview:        This function enable ZCD interrupt on PCINTx for Arduino Uno R3 and PCINTx for ArduinoMega 2560 R3
 *
 * PreCondition:    None
 *
 * GSM cmd syntax:  None    
 *
 * Input:           None
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * GSM answer det:  None
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
void Isr_MCP39F511::EnableZcdPortCx(void) {
    cli();                      // disable all interrupts   

//-----------------------------------------------
//  Pin Change Interrupt Control Register PCIE0
#ifdef ZCD_PCINT0
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT0);
#endif
#ifdef ZCD_PCINT1
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT1);
#endif
#ifdef ZCD_PCINT2
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT2);
#endif
#ifdef ZCD_PCINT3
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT3);
#endif
#ifdef ZCD_PCINT4
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT4);
#endif
#ifdef ZCD_PCINT5
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT5);
#endif
#ifdef ZCD_PCINT6
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT6);
#endif
#ifdef ZCD_PCINT7
    PCICR  |= (1 << PCIE0);
    PCMSK0 |= (1 << PCINT7);
#endif
//-----------------------------------------------

//-----------------------------------------------
//  Pin Change Interrupt Control Register PCIE1
#ifdef ZCD_PCINT8
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT8);
#endif
#ifdef ZCD_PCINT9
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT9);
#endif
#ifdef ZCD_PCINT10
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT10);
#endif
#ifdef ZCD_PCINT11
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT11);
#endif
#ifdef ZCD_PCINT12
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT12);
#endif
#ifdef ZCD_PCINT13
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT13);
#endif
#ifdef ZCD_PCINT14
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT14);
#endif
#ifdef ZCD_PCINT15
    PCICR  |= (1 << PCIE1);
    PCMSK1 |= (1 << PCINT15);
#endif
//-----------------------------------------------
    
//-----------------------------------------------
//  Pin Change Interrupt Control Register PCIE2
#ifdef ZCD_PCINT16
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT16);
#endif
#ifdef ZCD_PCINT17
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT17);
#endif
#ifdef ZCD_PCINT18
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT18);
#endif
#ifdef ZCD_PCINT19
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT19);
#endif
#ifdef ZCD_PCINT20
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT20);
#endif
#ifdef ZCD_PCINT21
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT21);
#endif
#ifdef ZCD_PCINT22
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT22);
#endif
#ifdef ZCD_PCINT23
    PCICR  |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT23);
#endif
//-----------------------------------------------
    sei();                      // enable all interrupts
}
/****************************************************************************/
    
/****************************************************************************
 * Function:        ISR(TIMER1_OVF_vect)
 *
 * Overview:        TIMER1 interrupt vector. This timer is set to generate an interrupt every 2mSec
 *                  This feature is used to create a moltitude of new timers using only simples variables
 *                  with a resolution of 2 msec.
 *                  For example to create a new timer of 100mSec is enough define a new variable and load
 *                  a correct value into it. In this example the correct value to load into the variable is 50.
 *                  This value is decremented every time that the interrupt occur. When this variable reach the
 *                  zero value means that the timer is expired
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
 * Note:            This is a private function
 *****************************************************************************/
static volatile void Isr_MCP39F511::Timer1_Interrupt(void) {
    //===========================================
    //  Timeout 8 bit
    if (Isr_39F511.ZcdTimeOut > 0) { Isr_39F511.ZcdTimeOut--; }
    //===========================================

    //===========================================
    //  Timeout 16 bit
#if defined(HARDWARE_UART1_39F511) || defined(SOFTWARE_UART1_39F511)
     if (Isr_39F511.Uart1_TimeOut > 0) { Isr_39F511.Uart1_TimeOut--; }       //  TimeOut for hardware/software serial COM 1                          
#endif
#if defined(HARDWARE_UART2_39F511) || defined(SOFTWARE_UART2_39F511)
     if (Isr_39F511.Uart2_TimeOut > 0) { Isr_39F511.Uart2_TimeOut--; }       //  TimeOut for hardware/software serial COM 2
#endif
#if defined(HARDWARE_UART3_39F511) || defined(SOFTWARE_UART3_39F511)
     if (Isr_39F511.Uart3_TimeOut > 0) { Isr_39F511.Uart3_TimeOut--; }       //  TimeOut for hardware/software serial COM 3
#endif
    //===========================================
}

#ifdef ENABLE_ISR_TIMER1
    ISR(TIMER1_OVF_vect) {
        TCNT1 = SLOWBASETIME_T1;    // preload timer
      
        //===========================================
        //  Timeout 8 bit
        if (Isr_39F511.ZcdTimeOut > 0) { Isr_39F511.ZcdTimeOut--; }
        //===========================================

        //===========================================
        //  Timeout 16 bit
    #if defined(HARDWARE_UART1_39F511) || defined(SOFTWARE_UART1_39F511)
         if (Isr_39F511.Uart1_TimeOut > 0) { Isr_39F511.Uart1_TimeOut--; }       //  TimeOut for hardware/software serial COM 1                          
    #endif
    #if defined(HARDWARE_UART2_39F511) || defined(SOFTWARE_UART2_39F511)
         if (Isr_39F511.Uart2_TimeOut > 0) { Isr_39F511.Uart2_TimeOut--; }       //  TimeOut for hardware/software serial COM 2
    #endif
    #if defined(HARDWARE_UART3_39F511) || defined(SOFTWARE_UART3_39F511)
         if (Isr_39F511.Uart3_TimeOut > 0) { Isr_39F511.Uart3_TimeOut--; }       //  TimeOut for hardware/software serial COM 3
    #endif
        //===========================================
    }
#endif     
/****************************************************************************/

/****************************************************************************
 * Function:        ISR(INT0_vect to INT5_Vect)
 *
 * Overview:        INT0 to INT5 interrupt vector. This interrupt is used to
 *                  manage Zero Cross Detection
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
 * Note:            This is a private function
 *****************************************************************************/
#ifdef ZCD_INT0
ISR(INT0_vect) {
#endif
#ifdef ZCD_INT1
ISR(INT1_vect) {
#endif
#ifdef ZCD_INT2
ISR(INT2_vect) {
#endif
#ifdef ZCD_INT3
ISR(INT3_vect) {
#endif
#ifdef ZCD_INT4
ISR(INT4_vect) {
#endif
#ifdef ZCD_INT5
ISR(INT5_vect) {
#endif
#if defined(ZCD_INT0) || defined(ZCD_INT1) || defined(ZCD_INT2)|| defined(ZCD_INT3)|| defined(ZCD_INT4)|| defined(ZCD_INT5)
    if (Isr_39F511.ZcdTimeOut > 0) {
        Isr_39F511.ZcdTimeOut = T_50MSEC;
        Cmd_39F511.GenericFlags.ZCD_Error = 0; //  Zero Cross Detection
    } else {
        Isr_39F511.ZcdTimeOut = T_50MSEC;
        Cmd_39F511.GenericFlags.ZCD_Error = 1; //  Zero Cross Detection Error
    }
}
#endif
/****************************************************************************/

/****************************************************************************
 * Function:        ISR(PCINTx_Vect)
 *
 * Overview:        PCINTx interrupt vector. This interrupt is used to
 *                  manage Zero Cross Detection
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
 * Note:            This is a private function
 *****************************************************************************/
#if defined(ZCD_PCINT0) || defined(ZCD_PCINT1) || defined(ZCD_PCINT2) || defined(ZCD_PCINT3) || defined(ZCD_PCINT4) || defined(ZCD_PCINT5) || defined(ZCD_PCINT6) || defined(ZCD_PCINT7)
ISR(PCINT0_vect) {
#endif
#if defined(ZCD_PCINT8) || defined(ZCD_PCINT9) || defined(ZCD_PCINT10) || defined(ZCD_PCINT11) || defined(ZCD_PCINT12) || defined(ZCD_PCINT13) || defined(ZCD_PCINT14) || defined(ZCD_PCINT15)
ISR(PCINT1_vect) {
#endif    
#if defined(ZCD_PCINT16) || defined(ZCD_PCINT17) || defined(ZCD_PCINT18) || defined(ZCD_PCINT19) || defined(ZCD_PCINT20) || defined(ZCD_PCINT21) || defined(ZCD_PCINT22) || defined(ZCD_PCINT23)
ISR(PCINT2_vect) {
#endif    
#if defined(ZCD_PCINT0)  || defined(ZCD_PCINT1)  || defined(ZCD_PCINT2)  || defined(ZCD_PCINT3)  || defined(ZCD_PCINT4)  || defined(ZCD_PCINT5)  || defined(ZCD_PCINT6)  || defined(ZCD_PCINT7)  || \
    defined(ZCD_PCINT8)  || defined(ZCD_PCINT9)  || defined(ZCD_PCINT10) || defined(ZCD_PCINT11) || defined(ZCD_PCINT12) || defined(ZCD_PCINT13) || defined(ZCD_PCINT14) || defined(ZCD_PCINT15) || \
    defined(ZCD_PCINT16) || defined(ZCD_PCINT17) || defined(ZCD_PCINT18) || defined(ZCD_PCINT19) || defined(ZCD_PCINT20) || defined(ZCD_PCINT21) || defined(ZCD_PCINT22) || defined(ZCD_PCINT23)

    if (Isr_39F511.ZcdTimeOut == 0) {
        Isr_39F511.ZcdTimeOut = T_20MSEC;
        Cmd_39F511.GenericFlags.ZCD_Error = 1; //  Zero Cross Detection Error        
    } else {
        if ((T_20MSEC - Isr_39F511.ZcdTimeOut) <= T_8MSEC) {
            Isr_39F511.ZcdTimeOut = T_20MSEC;
            Cmd_39F511.GenericFlags.ZCD_Error = 1; //  Zero Cross Detection Error                    
        } else {
            Isr_39F511.ZcdTimeOut = T_20MSEC;
            Cmd_39F511.GenericFlags.ZCD_Error = 0; //  Zero Cross Detection            
        }
    }
}
#endif
/****************************************************************************/