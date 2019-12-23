/*********************************************************************
 *
 *       Generic command for MCP39F511
 *
 *********************************************************************
 * FileName:        Cmd_MCP39F511.cpp
 * Revision:        1.0.0
 * Date:            04/05/2019
 *
 * Dependencies:    Cmd_MCP39F511.h
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
#include <avr/wdt.h>
#endif

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

Isr_MCP39F511     Isr;
Io_MCP39F511      Io;
Uart_MCP39F511    Uart;

/****************************************************************************
 * Function:        MCP39F511_FreezeFlash
 *
 * Overview:        This function is used to freeze data sent to FLASH
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
void Cmd_MCP39F511::MCP39F511_FreezeFlash(void) {
    Uart.Uart_Array[0] = HEADER_BYTE;
    Uart.Uart_Array[1] = CMD_FREEZE_FLASH_DATA_LENGHT;
    Uart.Uart_Array[2] = SAVE_REGISTER_TO_FLASH;
    Uart.Uart_Array[3] = CalcChecksum(3);
    Uart.Uart_Array[4] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteData_Byte
 *
 * Overview:        This routine is used to write a byte into MCP39F511
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
void Cmd_MCP39F511::MCP39F511_WriteData_Byte(uint16_t StartAdd, uint8_t Data) {
    Uart.Uart_Array[0] = HEADER_BYTE;
    Uart.Uart_Array[1] = CMD_WRITE_BYTE_DATA_LENGHT;
    Uart.Uart_Array[2] = SET_ADDRESS_POINTER;
    Uart.Uart_Array[3] = (uint8_t)((StartAdd & 0xFF00) >> 8);   //  MSB Address
    Uart.Uart_Array[4] = (uint8_t)(StartAdd  & 0x00FF);         //  LSB Address   
    Uart.Uart_Array[5] = REGISTER_WRITE_N;
    Uart.Uart_Array[6] = 1;   
    Uart.Uart_Array[7] = Data;
    Uart.Uart_Array[8] = CalcChecksum(8);   
    Uart.Uart_Array[9] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteData_Word
 *
 * Overview:        This routine is used to write a word into MCP39F511
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
void Cmd_MCP39F511::MCP39F511_WriteData_Word(uint16_t StartAdd, uint16_t Data) {
    Uart.Uart_Array[0]  = HEADER_BYTE;
    Uart.Uart_Array[1]  = CMD_WRITE_WORD_DATA_LENGHT;
    Uart.Uart_Array[2]  = SET_ADDRESS_POINTER;
    Uart.Uart_Array[3]  = (uint8_t)((StartAdd & 0xFF00) >> 8);      //  MSB Address
    Uart.Uart_Array[4]  = (uint8_t)(StartAdd  & 0x00FF);            //  LSB Address   
    Uart.Uart_Array[5]  = REGISTER_WRITE_N;
    Uart.Uart_Array[6]  = 2;   
    Uart.Uart_Array[7]  = (uint8_t)(Data  & 0x000000FF);            //  LSB Data First
    Uart.Uart_Array[8]  = (uint8_t)((Data & 0x0000FF00) >> 8);      //  MSB Data
    Uart.Uart_Array[9]  = CalcChecksum(9);   
    Uart.Uart_Array[10] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteData_DoubleWord
 *
 * Overview:        This routine is used to write a word into MCP39F511
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
void Cmd_MCP39F511::MCP39F511_WriteData_DoubleWord(uint16_t StartAdd, uint32_t Data) {
    Uart.Uart_Array[0]  = HEADER_BYTE;
    Uart.Uart_Array[1]  = CMD_WRITE_DWORD_DATA_LENGHT;
    Uart.Uart_Array[2]  = SET_ADDRESS_POINTER;
    Uart.Uart_Array[3]  = (uint8_t)((StartAdd & 0xFF00) >> 8);      //  MSB Address
    Uart.Uart_Array[4]  = (uint8_t)(StartAdd  & 0x00FF);            //  LSB Address   
    Uart.Uart_Array[5]  = REGISTER_WRITE_N;
    Uart.Uart_Array[6]  = 4;   
    Uart.Uart_Array[7]  = (uint8_t)(Data  & 0x000000FF);            //  LSB Data First
    Uart.Uart_Array[8]  = (uint8_t)((Data & 0x0000FF00) >> 8);      //
    Uart.Uart_Array[9]  = (uint8_t)((Data & 0x00FF0000) >> 16);     //     
    Uart.Uart_Array[10] = (uint8_t)((Data & 0xFF000000) >> 24);     //  MSB Data
    Uart.Uart_Array[11] = CalcChecksum(11);
    Uart.Uart_Array[12] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadData
 *
 * Overview:        This routine is used to read data from MCP39F511
 *
 * PreCondition:    None
 *
 * Input:           StartAdd:    Starting addres from which to read data 
 *                  nByteToRead: Numbers of byte to read from device 
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            This is a private function
 *****************************************************************************/
void Cmd_MCP39F511::MCP39F511_ReadData(uint16_t StartAdd, uint8_t nByteToRead) {
    Uart.Uart_Array[0] = HEADER_BYTE;
    Uart.Uart_Array[1] = CMD_READ_DATA_LENGHT;
    Uart.Uart_Array[2] = SET_ADDRESS_POINTER;
    Uart.Uart_Array[3] = (uint8_t)((StartAdd & 0xFF00) >> 8);   //  MSB Address
    Uart.Uart_Array[4] = (uint8_t)(StartAdd  & 0x00FF);         //  LSB Address
    Uart.Uart_Array[5] = REGISTER_READ_N;
    Uart.Uart_Array[6] = nByteToRead;
    Uart.Uart_Array[7] = CalcChecksum(7);
    Uart.Uart_Array[8] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteEeprom
 *
 * Overview:        This routine is used to Write Eeprom Page
 *
 * PreCondition:    None
 *
 * Input:           EepromPage:  Eeprom page number
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            This is a private function
 *****************************************************************************/
void Cmd_MCP39F511::MCP39F511_WriteEeprom(uint8_t EepromPage) {
    uint8_t Count;
    
    Uart.Uart_Array[0] = HEADER_BYTE;
    Uart.Uart_Array[1] = CMD_WRITE_PAGE_EEPROM_LENGHT;
    Uart.Uart_Array[2] = PAGE_WRITE_EEPROM;
    Uart.Uart_Array[3] = EepromPage;
    do {
        Uart.Uart_Array[Count + 4] = Cmd.EepromPageArray[Count];
    } while (++Count < sizeof(Cmd.EepromPageArray));
    Uart.Uart_Array[20] = CalcChecksum(20);
    Uart.Uart_Array[21] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadEeprom
 *
 * Overview:        This routine is used to read Eeprom Page
 *
 * PreCondition:    None
 *
 * Input:           EepromPage:  Eeprom page number
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            This is a private function
 *****************************************************************************/
void Cmd_MCP39F511::MCP39F511_ReadEeprom(uint8_t EepromPage) {
    Uart.Uart_Array[0] = HEADER_BYTE;
    Uart.Uart_Array[1] = CMD_READ_PAGE_EEPROM_LENGHT;
    Uart.Uart_Array[2] = PAGE_READ_EEPROM;
    Uart.Uart_Array[3] = EepromPage;
    Uart.Uart_Array[4] = CalcChecksum(4);
    Uart.Uart_Array[5] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_EraseEeprom
 *
 * Overview:        This routine is used to Erase Eeprom Page
 *
 * PreCondition:    None
 *
 * Input:           EepromPage:  Eeprom page number
 *
 * Command Note:    None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            This is a private function
 *****************************************************************************/
void Cmd_MCP39F511::MCP39F511_EraseEeprom(void) {
    Uart.Uart_Array[0] = HEADER_BYTE;
    Uart.Uart_Array[1] = CMD_ERASE_BULK_EEPROM_LENGHT;
    Uart.Uart_Array[2] = TYPE_ERASE_BULK_EEPROM;
    Uart.Uart_Array[3] = CalcChecksum(3);
    Uart.Uart_Array[4] = NULL_CHAR;
}
/****************************************************************************/

/****************************************************************************
 * Function:        CalcChecksum
 *
 * Overview:        This routine is used to calculate the checksum
 *
 * PreCondition:    None
 *
 * Input:           nByte: Number of bytes to calc checksum
 *
 * Command Note:    None
 *
 * Output:          Checksum
 *
 * Side Effects:    None
 *
 * Note:            This is a private function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::CalcChecksum(uint8_t nByte) {
    uint16_t TempChk = 0;
    uint8_t  Count   = 0;
    
    do {
        TempChk += Uart.Uart_Array[Count++];
    } while (Count < nByte);
    TempChk %= 0x0100;
    
    return(TempChk);
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_DecodeDataReceived
 *
 * Overview:        This function is used to decode received data
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
uint8_t Cmd_MCP39F511::MCP39F511_DecodeDataReceived(uint8_t nByteReceived, uint8_t TypeOfCmd) {
    uint8_t Count;
    
    switch(TypeOfCmd)
    {
        case TYPE_READ_EEPROM_PAGE_0:
        case TYPE_READ_EEPROM_PAGE_1:
        case TYPE_READ_EEPROM_PAGE_2:
        case TYPE_READ_EEPROM_PAGE_3:
        case TYPE_READ_EEPROM_PAGE_4:
        case TYPE_READ_EEPROM_PAGE_5:
        case TYPE_READ_EEPROM_PAGE_6: 
        case TYPE_READ_EEPROM_PAGE_7: 
        case TYPE_READ_EEPROM_PAGE_8: 
        case TYPE_READ_EEPROM_PAGE_9: 
        case TYPE_READ_EEPROM_PAGE_10:
        case TYPE_READ_EEPROM_PAGE_11:
        case TYPE_READ_EEPROM_PAGE_12:
        case TYPE_READ_EEPROM_PAGE_13:
        case TYPE_READ_EEPROM_PAGE_14:
        case TYPE_READ_EEPROM_PAGE_15:
        case TYPE_READ_EEPROM_PAGE_16:
        case TYPE_READ_EEPROM_PAGE_17:
        case TYPE_READ_EEPROM_PAGE_18:
        case TYPE_READ_EEPROM_PAGE_19:
        case TYPE_READ_EEPROM_PAGE_20:
        case TYPE_READ_EEPROM_PAGE_21:
        case TYPE_READ_EEPROM_PAGE_22:
        case TYPE_READ_EEPROM_PAGE_23:
        case TYPE_READ_EEPROM_PAGE_24:
        case TYPE_READ_EEPROM_PAGE_25:
        case TYPE_READ_EEPROM_PAGE_26:
        case TYPE_READ_EEPROM_PAGE_27:
        case TYPE_READ_EEPROM_PAGE_28:
        case TYPE_READ_EEPROM_PAGE_29:
        case TYPE_READ_EEPROM_PAGE_30:
        case TYPE_READ_EEPROM_PAGE_31:
            if (nByteReceived > (LENGHT_READ_PAGE_EEPROM_ANSWER - 1)) {
                if (Uart.Uart_Array[0] == RX_ACK) {                    
                    if (Uart.Uart_Array[LENGHT_READ_PAGE_EEPROM_ANSWER - 1] == CalcChecksum(LENGHT_READ_PAGE_EEPROM_ANSWER - 1)) {
                        Count = 0;
                        do {
                            EepromPageArray[Count] = Uart.Uart_Array[Count + 1];
                        } while (++Count < sizeof(EepromPageArray));
                        switch(TypeOfCmd)
                        {
                            case TYPE_READ_EEPROM_PAGE_0:
                                if (Cmd.GenericFlags.RestoreParametersFT1346M == 1) {
                                    GainCurrentRms.Byte0    = Uart.Uart_Array[2];
                                    GainCurrentRms.Byte1    = Uart.Uart_Array[3];
                                    GainVoltageRms.Byte0    = Uart.Uart_Array[4];
                                    GainVoltageRms.Byte1    = Uart.Uart_Array[5];
                                    GainActivePower.Byte0   = Uart.Uart_Array[6];
                                    GainActivePower.Byte1   = Uart.Uart_Array[7];
                                    GainReactivePower.Byte0 = Uart.Uart_Array[8];
                                    GainReactivePower.Byte1 = Uart.Uart_Array[9];
                                    OffsetCurrentRms.Byte0  = Uart.Uart_Array[10];
                                    OffsetCurrentRms.Byte1  = Uart.Uart_Array[11];
                                    OffsetCurrentRms.Byte2  = Uart.Uart_Array[12];
                                    OffsetCurrentRms.Byte3  = Uart.Uart_Array[13];
                                    OffsetActivePower.Byte0 = Uart.Uart_Array[14];
                                    OffsetActivePower.Byte1 = Uart.Uart_Array[15];
                                    OffsetActivePower.Byte2 = Uart.Uart_Array[16];
                                    OffsetActivePower.Byte3 = Uart.Uart_Array[17];                                    
                                } else {
                                    LoadEepromArray();
                                }
                                break;
                            case TYPE_READ_EEPROM_PAGE_1: 
                                if (Cmd.GenericFlags.RestoreParametersFT1346M == 1) {
                                    OffsetReactivePower.Byte0  = Uart.Uart_Array[2];
                                    OffsetReactivePower.Byte1  = Uart.Uart_Array[3];
                                    OffsetReactivePower.Byte2  = Uart.Uart_Array[4];
                                    OffsetReactivePower.Byte3  = Uart.Uart_Array[5];
                                    DcOffsetCurrent.Byte0      = Uart.Uart_Array[6];
                                    DcOffsetCurrent.Byte1      = Uart.Uart_Array[7];
                                    PhaseCompensation.Byte0    = Uart.Uart_Array[8];
                                    PhaseCompensation.Byte1    = Uart.Uart_Array[9];
                                    ApparentPowerDivisor.Byte0 = Uart.Uart_Array[10];
                                    ApparentPowerDivisor.Byte1 = Uart.Uart_Array[11];
                                    SystemConfiguration.Byte0  = Uart.Uart_Array[12];
                                    SystemConfiguration.Byte1  = Uart.Uart_Array[13];
                                    SystemConfiguration.Byte2  = Uart.Uart_Array[14];
                                    SystemConfiguration.Byte3  = Uart.Uart_Array[15];                 
                                } else {
                                    LoadEepromArray();                                    
                                }
                                break;
                            case TYPE_READ_EEPROM_PAGE_2:
                                if (Cmd.GenericFlags.RestoreParametersFT1346M == 1) {
                                    EventConfiguration.Byte0 = Uart.Uart_Array[2];
                                    EventConfiguration.Byte1 = Uart.Uart_Array[3];
                                    EventConfiguration.Byte2 = Uart.Uart_Array[4];
                                    EventConfiguration.Byte3 = Uart.Uart_Array[5];
                                    Range.Byte0              = Uart.Uart_Array[6];
                                    Range.Byte1              = Uart.Uart_Array[7];
                                    Range.Byte2              = Uart.Uart_Array[8];
                                    Range.Byte3              = Uart.Uart_Array[9];
                                    CalibrationCurrent.Byte0 = Uart.Uart_Array[10];
                                    CalibrationCurrent.Byte1 = Uart.Uart_Array[11];
                                    CalibrationCurrent.Byte2 = Uart.Uart_Array[12];
                                    CalibrationCurrent.Byte3 = Uart.Uart_Array[13];
                                    CalibrationVoltage.Byte0 = Uart.Uart_Array[14];
                                    CalibrationVoltage.Byte1 = Uart.Uart_Array[15];
                                } else {
                                    LoadEepromArray();
                                }
                                break;
                            case TYPE_READ_EEPROM_PAGE_3:
                                if (Cmd.GenericFlags.RestoreParametersFT1346M == 1) {
                                    CalibrationActivePower.Byte0        = Uart.Uart_Array[2];
                                    CalibrationActivePower.Byte1        = Uart.Uart_Array[3];
                                    CalibrationActivePower.Byte2        = Uart.Uart_Array[4];
                                    CalibrationActivePower.Byte3        = Uart.Uart_Array[5];
                                    CalibrationReactivePower.Byte0      = Uart.Uart_Array[6];
                                    CalibrationReactivePower.Byte1      = Uart.Uart_Array[7];
                                    CalibrationReactivePower.Byte2      = Uart.Uart_Array[8];
                                    CalibrationReactivePower.Byte3      = Uart.Uart_Array[9];
                                    LineFrequencyReference.Byte0        = Uart.Uart_Array[10];
                                    LineFrequencyReference.Byte1        = Uart.Uart_Array[11];
                                    AccumulatorIntervallParameter.Byte0 = Uart.Uart_Array[12];
                                    AccumulatorIntervallParameter.Byte1 = Uart.Uart_Array[13];
                                    VoltageSagLimit.Byte0               = Uart.Uart_Array[14];
                                    VoltageSagLimit.Byte1               = Uart.Uart_Array[15];
                                    VoltageSurgeLimit.Byte0             = Uart.Uart_Array[16];
                                    VoltageSurgeLimit.Byte1             = Uart.Uart_Array[17];
                                } else {
                                    LoadEepromArray();
                                }
                                break;
                            case TYPE_READ_EEPROM_PAGE_4:
                                if (Cmd.GenericFlags.RestoreParametersFT1346M == 1) {
                                    OverCurrentLimit.Byte0          = Uart.Uart_Array[2];
                                    OverCurrentLimit.Byte1          = Uart.Uart_Array[3];
                                    OverCurrentLimit.Byte2          = Uart.Uart_Array[4];
                                    OverCurrentLimit.Byte3          = Uart.Uart_Array[5];
                                    OverPowerLimit.Byte0            = Uart.Uart_Array[6];
                                    OverPowerLimit.Byte1            = Uart.Uart_Array[7];
                                    OverPowerLimit.Byte2            = Uart.Uart_Array[8];
                                    OverPowerLimit.Byte3            = Uart.Uart_Array[9];
                                    TempCompensationFrequency.Byte0 = Uart.Uart_Array[10];
                                    TempCompensationFrequency.Byte1 = Uart.Uart_Array[11];
                                    TempCompensationCurrent.Byte0   = Uart.Uart_Array[12];
                                    TempCompensationCurrent.Byte1   = Uart.Uart_Array[13];
                                    TempCompensationPower.Byte0     = Uart.Uart_Array[14];
                                    TempCompensationPower.Byte1     = Uart.Uart_Array[15];
                                    AmbientTempReference.Byte0      = Uart.Uart_Array[16];
                                    AmbientTempReference.Byte1      = Uart.Uart_Array[17];
                                } else {
                                    LoadEepromArray();
                                }
                                break;
                            case TYPE_READ_EEPROM_PAGE_5:
                                if (Cmd.GenericFlags.RestoreParametersFT1346M == 1) {
                                    PwmPeriod.Byte0            = Uart.Uart_Array[2];
                                    PwmPeriod.Byte1            = Uart.Uart_Array[3];
                                    PwmDutyCycle.Byte0         = Uart.Uart_Array[4];
                                    PwmDutyCycle.Byte1         = Uart.Uart_Array[5];
                                    MinMaxPointer_1.Byte0      = Uart.Uart_Array[6];
                                    MinMaxPointer_1.Byte1      = Uart.Uart_Array[7];
                                    MinMaxPointer_2.Byte0      = Uart.Uart_Array[8];
                                    MinMaxPointer_2.Byte1      = Uart.Uart_Array[9];
                                    OverTemperatureLimit.Byte0 = Uart.Uart_Array[10];
                                    OverTemperatureLimit.Byte1 = Uart.Uart_Array[11];
                                    EnergyControl.Byte0        = Uart.Uart_Array[12];
                                    EnergyControl.Byte1        = Uart.Uart_Array[13];
                                    PwmControl.Byte0           = Uart.Uart_Array[14];
                                    PwmControl.Byte1           = Uart.Uart_Array[15];
                                    NoLoadThreshold.Byte0      = Uart.Uart_Array[16];
                                    NoLoadThreshold.Byte1      = Uart.Uart_Array[17];
                                } else {
                                    LoadEepromArray();
                                }
                                break;                                
                            case TYPE_READ_EEPROM_PAGE_6: 
                            case TYPE_READ_EEPROM_PAGE_7: 
                            case TYPE_READ_EEPROM_PAGE_8: 
                            case TYPE_READ_EEPROM_PAGE_9: 
                            case TYPE_READ_EEPROM_PAGE_10:
                            case TYPE_READ_EEPROM_PAGE_11:
                            case TYPE_READ_EEPROM_PAGE_12:
                            case TYPE_READ_EEPROM_PAGE_13:
                            case TYPE_READ_EEPROM_PAGE_14:
                            case TYPE_READ_EEPROM_PAGE_15:
                            case TYPE_READ_EEPROM_PAGE_16:
                            case TYPE_READ_EEPROM_PAGE_17:
                            case TYPE_READ_EEPROM_PAGE_18:
                            case TYPE_READ_EEPROM_PAGE_19:
                            case TYPE_READ_EEPROM_PAGE_20:
                            case TYPE_READ_EEPROM_PAGE_21:
                            case TYPE_READ_EEPROM_PAGE_22:
                            case TYPE_READ_EEPROM_PAGE_23:
                            case TYPE_READ_EEPROM_PAGE_24:
                            case TYPE_READ_EEPROM_PAGE_25:
                            case TYPE_READ_EEPROM_PAGE_26:
                            case TYPE_READ_EEPROM_PAGE_27:
                            case TYPE_READ_EEPROM_PAGE_28:
                            case TYPE_READ_EEPROM_PAGE_29:
                            case TYPE_READ_EEPROM_PAGE_30:
                            case TYPE_READ_EEPROM_PAGE_31:
                                LoadEepromArray();
                                break;
                            default:
                                break;
                        }
                        return(DECODE_DATA_OK);
                    } else {
                        //  Checksum error
                        return(DECODE_CHECKSUM_ERROR);                        
                    }
                } else {
                    //  Packet error. NO ACK
                    return(DECODE_NO_ACKNOWLEDGE);                    
                }
            } else {
                //  Packet error. Received less data of those expected
                return(DECODE_RECEIVED_LESS_DATA);                
            }
            break;
        case TYPE_STATUS_VERSION:
        case TYPE_READ_V_RMS:
        case TYPE_READ_LINE_FREQ:
        case TYPE_READ_IN_ANAL_V:
        case TYPE_READ_P_FACTOR:
        case TYPE_READ_I_RMS:
        case TYPE_READ_P_ACTIVE:
        case TYPE_READ_P_REACTIVE:
        case TYPE_READ_P_APPARENT:
        case TYPE_READ_E_ACTIVE_IMP:
        case TYPE_READ_E_ACTIVE_EXP:
        case TYPE_READ_E_REACTIVE_IMP:
        case TYPE_READ_E_REACTIVE_EXP:
        case TYPE_READ_SYSTEM_CONFIGURATION:
        case TYPE_READ_EVENT_CONFIGURATION:
        case TYPE_READ_MINIMUM_RECORD_1:
        case TYPE_READ_MINIMUM_RECORD_2:
        case TYPE_READ_MAXIMUM_RECORD_1:
        case TYPE_READ_MAXIMUM_RECORD_2:
            if ((nByteReceived > 0) && (nByteReceived >= Uart.Uart_Array[1])) {
                if (Uart.Uart_Array[0] == RX_ACK) {
                    if (Uart.Uart_Array[Uart.Uart_Array[1] - 1] == CalcChecksum(Uart.Uart_Array[1] - 1)) {
                        switch(TypeOfCmd)
                        {
                            case TYPE_STATUS_VERSION:
                                Cmd.SystemStatus.Byte0  = Uart.Uart_Array[2];
                                Cmd.SystemStatus.Byte1  = Uart.Uart_Array[3];
                                Cmd.SystemVersion.Day   = (Uart.Uart_Array[4] & 0x0F) + ((Uart.Uart_Array[4] >> 4) * 10);   //  Decimal Format
                                Cmd.SystemVersion.Year  = (Uart.Uart_Array[5] >> 4);                                        //  Hex Format
                                Cmd.SystemVersion.Month = (Uart.Uart_Array[5] & 0x0F);                                      //  Hex Format
                                break;                                
                            case TYPE_READ_V_RMS:
                                VoltageRmsRaw.Byte0 = Uart.Uart_Array[2];
                                VoltageRmsRaw.Byte1 = Uart.Uart_Array[3];
                                break;
                            case TYPE_READ_LINE_FREQ:
                                LineFrequencyRaw.Byte0 = Uart.Uart_Array[2];
                                LineFrequencyRaw.Byte1 = Uart.Uart_Array[3];
                                break;
                            case TYPE_READ_IN_ANAL_V:   
                                AnalogInputVoltageRaw.Byte0 = Uart.Uart_Array[2];
                                AnalogInputVoltageRaw.Byte1 = Uart.Uart_Array[3];
                                break;
                            case TYPE_READ_P_FACTOR:
                                PowerFactorRaw.Byte0 = Uart.Uart_Array[2];
                                PowerFactorRaw.Byte1 = Uart.Uart_Array[3];
                                break;
                            case TYPE_READ_I_RMS:
                                CurrentRmsRaw.Byte0 = Uart.Uart_Array[2];
                                CurrentRmsRaw.Byte1 = Uart.Uart_Array[3];
                                CurrentRmsRaw.Byte2 = Uart.Uart_Array[4];
                                CurrentRmsRaw.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_P_ACTIVE:
                                ActivePowerRaw.Byte0 = Uart.Uart_Array[2];
                                ActivePowerRaw.Byte1 = Uart.Uart_Array[3];
                                ActivePowerRaw.Byte2 = Uart.Uart_Array[4];
                                ActivePowerRaw.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_P_REACTIVE:
                                ReactivePowerRaw.Byte0 = Uart.Uart_Array[2];
                                ReactivePowerRaw.Byte1 = Uart.Uart_Array[3];
                                ReactivePowerRaw.Byte2 = Uart.Uart_Array[4];
                                ReactivePowerRaw.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_P_APPARENT:
                                ApparentPowerRaw.Byte0 = Uart.Uart_Array[2];
                                ApparentPowerRaw.Byte1 = Uart.Uart_Array[3];
                                ApparentPowerRaw.Byte2 = Uart.Uart_Array[4];
                                ApparentPowerRaw.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_E_ACTIVE_IMP:
                                ImportActiveEnergyCount.Byte0 = Uart.Uart_Array[2];
                                ImportActiveEnergyCount.Byte1 = Uart.Uart_Array[3];
                                ImportActiveEnergyCount.Byte2 = Uart.Uart_Array[4];
                                ImportActiveEnergyCount.Byte3 = Uart.Uart_Array[5];
                                ImportActiveEnergyCount.Byte4 = Uart.Uart_Array[6];
                                ImportActiveEnergyCount.Byte5 = Uart.Uart_Array[7];
                                ImportActiveEnergyCount.Byte6 = Uart.Uart_Array[8];
                                ImportActiveEnergyCount.Byte7 = Uart.Uart_Array[9];
                                break;
                            case TYPE_READ_E_ACTIVE_EXP:
                                ExportActiveEnergyCount.Byte0 = Uart.Uart_Array[2];
                                ExportActiveEnergyCount.Byte1 = Uart.Uart_Array[3];
                                ExportActiveEnergyCount.Byte2 = Uart.Uart_Array[4];
                                ExportActiveEnergyCount.Byte3 = Uart.Uart_Array[5];
                                ExportActiveEnergyCount.Byte4 = Uart.Uart_Array[6];
                                ExportActiveEnergyCount.Byte5 = Uart.Uart_Array[7];
                                ExportActiveEnergyCount.Byte6 = Uart.Uart_Array[8];
                                ExportActiveEnergyCount.Byte7 = Uart.Uart_Array[9];
                                break;
                            case TYPE_READ_E_REACTIVE_IMP:
                                ImportReactiveEnergyCount.Byte0 = Uart.Uart_Array[2];
                                ImportReactiveEnergyCount.Byte1 = Uart.Uart_Array[3];
                                ImportReactiveEnergyCount.Byte2 = Uart.Uart_Array[4];
                                ImportReactiveEnergyCount.Byte3 = Uart.Uart_Array[5];
                                ImportReactiveEnergyCount.Byte4 = Uart.Uart_Array[6];
                                ImportReactiveEnergyCount.Byte5 = Uart.Uart_Array[7];
                                ImportReactiveEnergyCount.Byte6 = Uart.Uart_Array[8];
                                ImportReactiveEnergyCount.Byte7 = Uart.Uart_Array[9];
                                break;
                            case TYPE_READ_E_REACTIVE_EXP:
                                ExportReactiveEnergyCount.Byte0 = Uart.Uart_Array[2];
                                ExportReactiveEnergyCount.Byte1 = Uart.Uart_Array[3];
                                ExportReactiveEnergyCount.Byte2 = Uart.Uart_Array[4];
                                ExportReactiveEnergyCount.Byte3 = Uart.Uart_Array[5];
                                ExportReactiveEnergyCount.Byte4 = Uart.Uart_Array[6];
                                ExportReactiveEnergyCount.Byte5 = Uart.Uart_Array[7];
                                ExportReactiveEnergyCount.Byte6 = Uart.Uart_Array[8];
                                ExportReactiveEnergyCount.Byte7 = Uart.Uart_Array[9];
                                break;
                            case TYPE_READ_SYSTEM_CONFIGURATION:
                                SystemConfiguration.Byte0 = Uart.Uart_Array[2];
                                SystemConfiguration.Byte1 = Uart.Uart_Array[3];
                                SystemConfiguration.Byte2 = Uart.Uart_Array[4];
                                SystemConfiguration.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_EVENT_CONFIGURATION:
                                EventConfiguration.Byte0 = Uart.Uart_Array[2];
                                EventConfiguration.Byte1 = Uart.Uart_Array[3];
                                EventConfiguration.Byte2 = Uart.Uart_Array[4];
                                EventConfiguration.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_MINIMUM_RECORD_1:
                                MinimumRecord_1.Byte0 = Uart.Uart_Array[2];
                                MinimumRecord_1.Byte1 = Uart.Uart_Array[3];
                                MinimumRecord_1.Byte2 = Uart.Uart_Array[4];
                                MinimumRecord_1.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_MINIMUM_RECORD_2:
                                MinimumRecord_2.Byte0 = Uart.Uart_Array[2];
                                MinimumRecord_2.Byte1 = Uart.Uart_Array[3];
                                MinimumRecord_2.Byte2 = Uart.Uart_Array[4];
                                MinimumRecord_2.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_MAXIMUM_RECORD_1:
                                MaximumRecord_1.Byte0 = Uart.Uart_Array[2];
                                MaximumRecord_1.Byte1 = Uart.Uart_Array[3];
                                MaximumRecord_1.Byte2 = Uart.Uart_Array[4];
                                MaximumRecord_1.Byte3 = Uart.Uart_Array[5];
                                break;
                            case TYPE_READ_MAXIMUM_RECORD_2:
                                MaximumRecord_2.Byte0 = Uart.Uart_Array[2];
                                MaximumRecord_2.Byte1 = Uart.Uart_Array[3];
                                MaximumRecord_2.Byte2 = Uart.Uart_Array[4];
                                MaximumRecord_2.Byte3 = Uart.Uart_Array[5];
                                break;
                             default:
                                break;
                        }
                        return(DECODE_DATA_OK);
                    } else {
                        //  Checksum error
                        return(DECODE_CHECKSUM_ERROR);
                    }
                } else {
                    //  Packet error. NO ACK
                    return(DECODE_NO_ACKNOWLEDGE);
                }
            } else {
                //  Packet error. Received less data of those expected
                return(DECODE_RECEIVED_LESS_DATA);
            }
            break;
        case TYPE_WRITE_EEPROM_PAGE:
        case TYPE_ERASE_BULK_EEPROM:
        case TYPE_WRITE_RETRIEVE_FACTORY_DEFAULT:
        case TYPE_WRITE_GAIN_CURRENT_RMS:
        case TYPE_WRITE_GAIN_VOLTAGE_RMS:
        case TYPE_WRITE_GAIN_ACTIVE_POWER:
        case TYPE_WRITE_GAIN_REACTIVE_POWER:
        case TYPE_WRITE_OFFSET_CURRENT_RMS:
        case TYPE_WRITE_OFFSET_ACTIVE_POWER:
        case TYPE_WRITE_OFFSET_REACTIVE_POWER:
        case TYPE_WRITE_DC_OFFSET_CURRENT:
        case TYPE_WRITE_PHASE_COMPENSATION:
        case TYPE_WRITE_APPARENT_POWER_DIVISOR:
        case TYPE_WRITE_SYSTEM_CONFIGURATION:
        case TYPE_WRITE_EVENT_CONFIGURATION:
        case TYPE_WRITE_RANGE:
        case TYPE_WRITE_CALIBRATION_CURRENT:
        case TYPE_WRITE_CALIBRATION_VOLTAGE:
        case TYPE_WRITE_CALIBRATION_ACTIVE_POWER:
        case TYPE_WRITE_CALIBRATION_REACTIVE_POWER:
        case TYPE_WRITE_ACCUMULATOR_INTERVAL_PARAMETER:
        case TYPE_WRITE_VOLTAGE_SAG_LIMIT:
        case TYPE_WRITE_VOLTAGE_SURGE_LIMIT:
        case TYPE_WRITE_OVER_CURRENT_LIMIT:
        case TYPE_WRITE_OVER_POWER_LIMIT:
        case TYPE_WRITE_TEMP_COMPENSATION_FOR_FREQUENCY:
        case TYPE_WRITE_TEMP_COMPENSATION_FOR_CURRENT:
        case TYPE_WRITE_TEMP_COMPENSATION_FOR_POWER:
        case TYPE_WRITE_AMBIENT_TEMP_REFERENCE:
        case TYPE_WRITE_PWM_PERIOD:
        case TYPE_WRITE_PWM_DUTY_CYCLE:
        case TYPE_WRITE_MINMAX_POINTER_1:
        case TYPE_WRITE_MINMAX_POINTER_2:
        case TYPE_WRITE_OVERTEMPERATURE_LIMIT:
        case TYPE_WRITE_ENERGY_CONTROL:
        case TYPE_WRITE_PWM_CONTROL:
        case TYPE_WRITE_NO_LOAD_THRESHOLD:
        case TYPE_FREEZE_FLASH:
            if (nByteReceived > 0) {
                if (Uart.Uart_Array[0] == RX_ACK) {
                    return(DECODE_DATA_OK);
                } else {
                    //  Packet error. NO ACK
                    return(DECODE_NO_ACKNOWLEDGE);
                }
            } else {
                //  Packet error. Received less data of those expected
                return(DECODE_RECEIVED_LESS_DATA);
            }
            break;
        default:
            break;
    }  
}

void Cmd_MCP39F511::LoadEepromArray(void) {
    uint8_t Count = 0;
    do {
        Cmd.EepromPageArray[Count] = Uart.Uart_Array[Count + 2];
    } while (++Count < sizeof(Cmd.EepromPageArray));    
}
/****************************************************************************/

//============================================================================================================
//  LIST OF FUNCTIONS USED TO RETREIVE FACTORY PARAMETERS

/****************************************************************************
 * Function:        MCP39F511_RetrieveMicrochipFactoryValues
 *
 * Overview:        This function is used to write the code 0xA5A5 to Retrieve Microchip Factory Default
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_RetrieveMicrochipFactoryValues(void) {    
    MCP39F511_WriteData_Word(CALIBRATION_REGISTER_DELIMITER_ADD, RETRIEVE_FACTORY_DEFAULT_CODE);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_RETRIEVE_FACTORY_DEFAULT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_RetrieveFT1346M_FactoryParameters
 *
 * Overview:        This function is used to retrieve FT1346M Factory Parameters
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_RetrieveFT1346M_FactoryParameters(void) {    
    uint8_t ErrorCode;

    ErrorCode = MCP39F511_ReadEepromPage(EEPROM_PAGE_00);
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteGainCurrentRms();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteGainVoltageRms();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteGainActivePower();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteGainReactivePower();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteOffsetCurrentRms();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteOffsetActivePower();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    
    
    ErrorCode = MCP39F511_ReadEepromPage(EEPROM_PAGE_01);
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteOffsetReactivePower();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteDcOffsetCurrent();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WritePhaseCompensation();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteApparentPowerDivisor();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteSystemConfiguration();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    
    
    ErrorCode = MCP39F511_ReadEepromPage(EEPROM_PAGE_02);
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteEventConfiguration();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteRange();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteCalibrationCurrent();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteCalibrationVoltage();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    
    
    ErrorCode = MCP39F511_ReadEepromPage(EEPROM_PAGE_03);
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteCalibrationActivePower();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteCalibrationReactivePower();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteLineFrequencyReference();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteAccumulatorItervalParameter();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteVoltageSagLimit();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_WriteVoltageSurgeLimit();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    
    
    ErrorCode = MCP39F511_ReadEepromPage(EEPROM_PAGE_04);
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteOverCurrentLimit();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteOverPowerLimit();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteTempCompensationFrequency();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteTempCompensationCurrent();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteTempCompensationPower();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteAmbientTempReference();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    
    
    ErrorCode = MCP39F511_ReadEepromPage(EEPROM_PAGE_05);
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WritePwmPeriod();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WritePwmDutyCycle();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteMinMaxPointer_1();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteMinMaxPointer_2();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteOverTemperatureLimit();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteEnergyControl();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WritePwmControl();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_WriteNoLoadThreshold();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    
    return(DECODE_DATA_OK);
}
/****************************************************************************/
//============================================================================================================

//============================================================================================================
/****************************************************************************
 * Function:        CalcPowerFactor
 *
 * Overview:        This routine is used to calculate the checksum
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Command Note:    None
 *
 * Output:          Power Factor
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
unsigned long Cmd_MCP39F511::CalcPowerFactor(void) {
    unsigned long TempData;
    
    Cmd.GenericFlags.PowerFactorSign = 0;
    TempData = Cmd.PowerFactorRaw.uWord;
    if ((Cmd.PowerFactorRaw.uWord & 0x8000) > 0) {
        //  Negative Power Factor
        Cmd.GenericFlags.PowerFactorSign = 1;
        TempData = (Cmd.PowerFactorRaw.uWord ^ 0xFFFF) + 0x0001;
    }
    //  The equation to convert the PowerFactor value in a decimal value is:
    //       3051 * X
    //  Y = -----------
    //        100000
    TempData *= 3051;
    TempData /= 100000; 
    
    return(TempData);
}
/****************************************************************************/
//============================================================================================================

//============================================================================================================
//  LIST OF FUNCTIONS USED TO READ AND WRITE EEPROM PAGE

/****************************************************************************
 * Function:        MCP39F511_ReadEepromPage
 *
 * Overview:        This function is used to read Page Eeprom
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadEepromPage(uint8_t EepromPage) {
    MCP39F511_ReadEeprom(EepromPage);
    Uart.UartSendData(CMD_READ_PAGE_EEPROM_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), (TYPE_READ_EEPROM_PAGE_0 + EepromPage)));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteEepromPage
 *
 * Overview:        This function is used to read Page Eeprom
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteEepromPage(uint8_t EepromPage) {
    MCP39F511_WriteEeprom(EepromPage);
    Uart.UartSendData(CMD_WRITE_PAGE_EEPROM_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_EEPROM_PAGE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_BulkEraseEeprom
 *
 * Overview:        This function is used to Erase Eeprom
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_BulkEraseEeprom(void) {
    MCP39F511_EraseEeprom();
    Uart.UartSendData(CMD_ERASE_BULK_EEPROM_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_ERASE_BULK_EEPROM));
}
/****************************************************************************/
//============================================================================================================

//============================================================================================================
//  LIST OF FUNCTIONS USED TO READ DATA FROM MCP39F511 DEVICE

/****************************************************************************
 * Function:        MCP39F511_ReadStatusAndVersion
 *
 * Overview:        This function is used to read Status and Versione of device
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadStatusAndVersion(void) {
    MCP39F511_ReadData(SYSTEM_STATUS_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_STATUS_VERSION));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadSystemConfiguration
 *
 * Overview:        This function is used to read System Configuration
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadSystemConfiguration(void) {
    MCP39F511_ReadData(SYSTEM_CONFIGURATION_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_SYSTEM_CONFIGURATION));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadEventConfiguration
 *
 * Overview:        This function is used to read System Configuration
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadEventStatus(void) {
    MCP39F511_ReadData(EVENT_CONFIGURATION_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_EVENT_CONFIGURATION));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadRmsVoltageRaw
 *
 * Overview:        This function is used to read RMS Voltage Raw data
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadRmsVoltageRaw(void) {
    MCP39F511_ReadData(VOLTAGE_RMS_ADD, 2);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_V_RMS));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadLineFrequencyRaw
 *
 * Overview:        This function is used to read line frequency
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadLineFrequencyRaw(void) {
    MCP39F511_ReadData(LINE_FREQUENCY_ADD, 2);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_LINE_FREQ));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadSarAdc
 *
 * Overview:        This function is used to read internal SAR ADC
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadSarAdc(void) {
    MCP39F511_ReadData(ANALOG_INPUT_VOLTAGE_ADD, 2);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_IN_ANAL_V));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadPowerFactorRaw
 *
 * Overview:        This function is used to read Power Factor Raw Data
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadPowerFactorRaw(void) {
    MCP39F511_ReadData(POWER_FACTOR_ADD, 2);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_P_FACTOR));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadRmsCurrentRaw
 *
 * Overview:        This function is used to read RMS current Raw Data
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadRmsCurrentRaw(void) {
    MCP39F511_ReadData(CURRENT_RMS_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_I_RMS));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadActivePowerRaw
 *
 * Overview:        This function is used to read Active Power Raw Data
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadActivePowerRaw(void) {
    MCP39F511_ReadData(ACTIVE_POWER_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_P_ACTIVE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadReactivePowerRaw
 *
 * Overview:        This function is used to read Reactive Power Raw Data
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadReactivePowerRaw(void) {
    MCP39F511_ReadData(REACTIVE_POWER_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_P_REACTIVE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadApparentPowerRaw
 *
 * Overview:        This function is used to read Apparent Power Raw Data
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadApparentPowerRaw(void) {
    MCP39F511_ReadData(APPARENT_POWER_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_P_APPARENT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadImportedActiveEnergy
 *
 * Overview:        This function is used to read Imported Active Energy
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadImportedActiveEnergy(void) {
    MCP39F511_ReadData(IMPORT_ACTIVE_ENERGY_COUNT_ADD, 8);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_E_ACTIVE_IMP));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadExportedActiveEnergy
 *
 * Overview:        This function is used to read Exported Active Energy
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadExportedActiveEnergy(void) {
    MCP39F511_ReadData(EXPORT_ACTIVE_ENERGY_COUNT_ADD, 8);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_E_ACTIVE_EXP));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadImportedReactiveEnergy
 *
 * Overview:        This function is used to read Imported Reactive Energy
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadImportedReactiveEnergy(void) {
    MCP39F511_ReadData(IMPORT_REACTIVE_ENERGY_COUNT_ADD, 8);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_E_REACTIVE_IMP));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadExportedReactiveEnergy
 *
 * Overview:        This function is used to read Exported Reactive Energy
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadExportedReactiveEnergy(void) {
    MCP39F511_ReadData(EXPORT_REACTIVE_ENERGY_COUNT_ADD, 8);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_E_REACTIVE_EXP));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadMinimumRecord_1
 *
 * Overview:        This function is used to read Minimum Record 1
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadMinimumRecord_1(void) {
    MCP39F511_ReadData(MINIMUM_RECORD_1_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_MINIMUM_RECORD_1));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadMinimumRecord_2
 *
 * Overview:        This function is used to read Minimum Record 2
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadMinimumRecord_2(void) {
    MCP39F511_ReadData(MINIMUM_RECORD_2_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_MINIMUM_RECORD_2));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadMaximumRecord_1
 *
 * Overview:        This function is used to read Maximum Record 1
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadMaximumRecord_1(void) {
    MCP39F511_ReadData(MAXIMUM_RECORD_1_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_MAXIMUM_RECORD_1));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadMaximumRecord_2
 *
 * Overview:        This function is used to read Maximum Record 2
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadMaximumRecord_2(void) {
    MCP39F511_ReadData(MAXIMUM_RECORD_2_ADD, 4);
    Uart.UartSendData(CMD_READ_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_READ_MAXIMUM_RECORD_2));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadAllRawData
 *
 * Overview:        This function is used to read:
 *                  RMS Voltage Raw Data
 *                  Line Frequency Raw Data
 *                  Power Factor Raw Data
 *                  RMS current Raw Data
 *                  Active Power Raw Data
 *                  Reactive Power Raw Data
 *                  Apparent Power Raw Data
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadAllRawData(void) {
    uint8_t ErrorCode;
    
    ErrorCode = MCP39F511_ReadRmsVoltageRaw();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_ReadLineFrequencyRaw();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_ReadPowerFactorRaw();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }   
    ErrorCode = MCP39F511_ReadRmsCurrentRaw();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }       
    ErrorCode = MCP39F511_ReadActivePowerRaw();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); } 
    ErrorCode = MCP39F511_ReadReactivePowerRaw();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); } 
    return(MCP39F511_ReadApparentPowerRaw());
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ReadAllEnergy
 *
 * Overview:        This function is used to read:
 *                  Imported Active Energy
 *                  Exported Active Energy
 *                  Imported Reactive Energy
 *                  Exported Reactive Energy
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ReadAllEnergy(void) {
    uint8_t ErrorCode;
    
    ErrorCode = MCP39F511_ReadImportedActiveEnergy();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }
    ErrorCode = MCP39F511_ReadExportedActiveEnergy();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); }    
    ErrorCode = MCP39F511_ReadImportedReactiveEnergy();
    if (ErrorCode != DECODE_DATA_OK) { return (ErrorCode); } 
    return(MCP39F511_ReadExportedReactiveEnergy());
}
/****************************************************************************/
//============================================================================================================

//============================================================================================================
//  LIST OF FUNCTIONS USED TO write DATA INTO MCP39F511 DEVICE

/****************************************************************************
 * Function:        MCP39F511_FreezeFlashData
 *
 * Overview:        This function is used to Freeze data in FLASH
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_FreezeFlashData(void) {
    MCP39F511_FreezeFlash();
    Uart.UartSendData(CMD_FREEZE_FLASH_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_FREEZE_FLASH));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteMinMaxPointer_1
 *
 * Overview:        This function is used to write address pointer for Min/Max 1 Outputs
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteMinMaxPointer_1(void) {
    MCP39F511_WriteData_DoubleWord(MINMAX_POINTER_1_ADD, MinMaxPointer_1.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_MINMAX_POINTER_1));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ResetMinMaxPointer_1
 *
 * Overview:        This function is used to reset address pointer for Min/Max 1 Outputs
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ResetMinMaxPointer_1(void) {
    MCP39F511_WriteData_DoubleWord(MINMAX_POINTER_1_ADD, 0x0000);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_MINMAX_POINTER_1));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteMinMaxPointer_2
 *
 * Overview:        This function is used to write address pointer for Min/Max 2 Outputs
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteMinMaxPointer_2(void) {
    MCP39F511_WriteData_DoubleWord(MINMAX_POINTER_2_ADD, MinMaxPointer_2.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_MINMAX_POINTER_2));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_ResetMinMaxPointer_2
 *
 * Overview:        This function is used to reset address pointer for Min/Max 2 Outputs
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_ResetMinMaxPointer_2(void) {
    MCP39F511_WriteData_DoubleWord(MINMAX_POINTER_1_ADD, 0x0000);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_MINMAX_POINTER_1));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteGainCurrentRms
 *
 * Overview:        This function is used to set Gain current RMS
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteGainCurrentRms(void) {   
    MCP39F511_WriteData_Word(GAIN_CURRENT_RMS_ADD, GainCurrentRms.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_GAIN_CURRENT_RMS));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteGainVoltageRms
 *
 * Overview:        This function is used to set Gain Voltage RMS
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteGainVoltageRms(void) {   
    MCP39F511_WriteData_Word(GAIN_VOLTAGE_RMS_ADD, GainVoltageRms.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_GAIN_VOLTAGE_RMS));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteGainActivePower
 *
 * Overview:        This function is used to set Gain Active Power
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteGainActivePower(void) {   
    MCP39F511_WriteData_Word(GAIN_ACTIVE_POWER_ADD, GainActivePower.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_GAIN_ACTIVE_POWER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteGainReactivePower
 *
 * Overview:        This function is used to set Gain Reactive Power
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteGainReactivePower(void) {   
    MCP39F511_WriteData_Word(GAIN_REACTIVE_POWER_ADD, GainReactivePower.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_GAIN_REACTIVE_POWER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteOffsetCurrentRms\
 *
 * Overview:        This function is used to set Offset Current RMS
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteOffsetCurrentRms(void) {   
    MCP39F511_WriteData_DoubleWord(OFFSET_CURRENT_RMS_ADD, OffsetCurrentRms.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_OFFSET_CURRENT_RMS));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteOffsetActivePower
 *
 * Overview:        This function is used to set Offset Active Power
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteOffsetActivePower(void) {   
    MCP39F511_WriteData_DoubleWord(OFFSET_ACTIVE_POWER_ADD, OffsetActivePower.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_OFFSET_ACTIVE_POWER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteOffsetReactivePower
 *
 * Overview:        This function is used to set Offset Active Power
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteOffsetReactivePower(void) {   
    MCP39F511_WriteData_DoubleWord(OFFSET_REACTIVE_POWER_ADD, OffsetReactivePower.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_OFFSET_REACTIVE_POWER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteDcOffsetCurrent
 *
 * Overview:        This function is used to set DC Offset Current
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteDcOffsetCurrent(void) {   
    MCP39F511_WriteData_Word(DC_OFFSET_CURRENT_ADD, DcOffsetCurrent.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_DC_OFFSET_CURRENT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WritePhaseCompensation
 *
 * Overview:        This function is used to set Phase Compensation
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WritePhaseCompensation(void) {   
    MCP39F511_WriteData_Word(PHASE_COMPENSATION_ADD, PhaseCompensation.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_PHASE_COMPENSATION));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteApparentPowerDivisor
 *
 * Overview:        This function is used to set Apparent Power Divisor
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteApparentPowerDivisor(void) {   
    MCP39F511_WriteData_Word(APPARENT_POWER_DIVISOR_ADD, ApparentPowerDivisor.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_APPARENT_POWER_DIVISOR));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteSystemConfiguration
 *
 * Overview:        This function is used to set System configuration Register
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteSystemConfiguration(void) {   
    MCP39F511_WriteData_Word(SYSTEM_CONFIGURATION_ADD, SystemConfiguration.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_SYSTEM_CONFIGURATION));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteEventConfiguration
 *
 * Overview:        This function is used to set Event Configuration
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteEventConfiguration(void) {   
    MCP39F511_WriteData_Word(EVENT_CONFIGURATION_ADD, EventConfiguration.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_EVENT_CONFIGURATION));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteRange
 *
 * Overview:        This function is used to set Range
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteRange(void) {   
    MCP39F511_WriteData_Word(RANGE_ADD, Range.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_RANGE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteCalibrationCurrent
 *
 * Overview:        This function is used to set Calibration Current
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteCalibrationCurrent(void) {   
    MCP39F511_WriteData_Word(CALIBRATION_CURRENT_ADD, CalibrationCurrent.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_CALIBRATION_CURRENT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteCalibrationVoltage
 *
 * Overview:        This function is used to set Calibration Voltage
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteCalibrationVoltage(void) {   
    MCP39F511_WriteData_Word(CALIBRATION_VOLTAGE_ADD, CalibrationVoltage.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_CALIBRATION_VOLTAGE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteCalibrationActivePower
 *
 * Overview:        This function is used to set Calibration Active Power
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteCalibrationActivePower(void) {   
    MCP39F511_WriteData_Word(CALIBRATION_ACTIVE_POWER_ADD, CalibrationActivePower.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_CALIBRATION_ACTIVE_POWER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteCalibrationReactivePower
 *
 * Overview:        This function is used to set Calibration Reactive Power
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteCalibrationReactivePower(void) {   
    MCP39F511_WriteData_Word(CALIBRATION_REACTIVE_POWER_ADD, CalibrationReactivePower.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_CALIBRATION_REACTIVE_POWER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteLineFrequencyReference
 *
 * Overview:        This function is used to set Line Frequency Reference
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteLineFrequencyReference(void) {   
    MCP39F511_WriteData_Word(LINE_FREQUENCY_REFERENCE_ADD, LineFrequencyReference.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_LINE_FREQUENCY_REFERENCE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteAccumulatorItervalParameter
 *
 * Overview:        This function is used to set Accumulator Interval Parameter
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteAccumulatorItervalParameter(void) {   
    MCP39F511_WriteData_Word(ACCUMULATOR_INTERVAL_PARAMETER_ADD, AccumulatorIntervallParameter.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_ACCUMULATOR_INTERVAL_PARAMETER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteVoltageSagLimit
 *
 * Overview:        This function is used to set Voltage Sag Limit
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteVoltageSagLimit(void) {   
    MCP39F511_WriteData_Word(VOLTAGE_SAG_LIMIT_ADD, VoltageSagLimit.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_VOLTAGE_SAG_LIMIT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteVoltageSurgeLimit
 *
 * Overview:        This function is used to set Voltage Surge Limit
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteVoltageSurgeLimit(void) {   
    MCP39F511_WriteData_Word(VOLTAGE_SURGE_LIMIT_ADD, VoltageSurgeLimit.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_VOLTAGE_SURGE_LIMIT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteOverCurrentLimit
 *
 * Overview:        This function is used to set Over Current Limit
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteOverCurrentLimit(void) {   
    MCP39F511_WriteData_Word(OVER_CURRENT_LIMIT_ADD, OverCurrentLimit.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_OVER_CURRENT_LIMIT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteOverPowerLimit
 *
 * Overview:        This function is used to set Over Power Limit
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteOverPowerLimit(void) {   
    MCP39F511_WriteData_Word(OVER_POWER_LIMIT_ADD, OverPowerLimit.uDWord);
    Uart.UartSendData(CMD_WRITE_DWORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_OVER_POWER_LIMIT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteTempCompensationFrequency
 *
 * Overview:        This function is used to set Temperature Compensation For Frequency
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteTempCompensationFrequency(void) {   
    MCP39F511_WriteData_Word(TEMP_COMPENSATION_FOR_FREQUENCY_ADD, TempCompensationFrequency.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_TEMP_COMPENSATION_FOR_FREQUENCY));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteTempCompensationCurrent
 *
 * Overview:        This function is used to set Temperature Compensation For Current
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteTempCompensationCurrent(void) {   
    MCP39F511_WriteData_Word(TEMP_COMPENSATION_FOR_CURRENT_ADD, TempCompensationCurrent.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_TEMP_COMPENSATION_FOR_CURRENT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteTempCompensationPower
 *
 * Overview:        This function is used to set Temperature Compensation For Power
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteTempCompensationPower(void) {   
    MCP39F511_WriteData_Word(TEMP_COMPENSATION_FOR_POWER_ADD, TempCompensationPower.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_TEMP_COMPENSATION_FOR_POWER));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteAmbientTempReference
 *
 * Overview:        This function is used to set Ambient Temperature Reference
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteAmbientTempReference(void) {   
    MCP39F511_WriteData_Word(AMBIENT_TEMP_REFERENCE_ADD, AmbientTempReference.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_AMBIENT_TEMP_REFERENCE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WritePwmPeriod
 *
 * Overview:        This function is used to set PWM Period
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WritePwmPeriod(void) {   
    MCP39F511_WriteData_Word(PWM_PERIOD_ADD, PwmPeriod.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_PWM_PERIOD));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WritePwmDutyCycle
 *
 * Overview:        This function is used to set PWM Duty cycle
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WritePwmDutyCycle(void) {   
    MCP39F511_WriteData_Word(PWM_DUTY_CYCLE_ADD, PwmDutyCycle.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_PWM_DUTY_CYCLE));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteOverTemperatureLimit
 *
 * Overview:        This function is used to set Over Temperature Limit
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteOverTemperatureLimit(void) {   
    MCP39F511_WriteData_Word(OVERTEMPERATURE_LIMIT_ADD, OverTemperatureLimit.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_OVERTEMPERATURE_LIMIT));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteEnergyControl
 *
 * Overview:        This function is used to set Energy Control
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteEnergyControl(void) {   
    MCP39F511_WriteData_Word(ENERGY_CONTROL_ADD, EnergyControl.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_ENERGY_CONTROL));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WritePwmControl
 *
 * Overview:        This function is used to set PWM Control
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WritePwmControl(void) {   
    MCP39F511_WriteData_Word(PWM_CONTROL_ADD, PwmControl.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_PWM_CONTROL));
}
/****************************************************************************/

/****************************************************************************
 * Function:        MCP39F511_WriteNoLoadThreshold
 *
 * Overview:        This function is used to set No Load Threshold
 *
 * PreCondition:    None
 *
 * Input:           None 
 *
 * Command Note:    None
 *
 * Output:          Error Code:
 *                      0x00 No error
 *                      0x01 Received less data
 *                      0x02 Checksum error
 *                      0x03 No Acknowledge
 *
 * Side Effects:    None
 *
 * Note:            This is a public function
 *****************************************************************************/
uint8_t Cmd_MCP39F511::MCP39F511_WriteNoLoadThreshold(void) {   
    MCP39F511_WriteData_Word(NO_LOAD_THRESHOLD_ADD, NoLoadThreshold.uWord);
    Uart.UartSendData(CMD_WRITE_WORD_DATA_LENGHT);
    return(MCP39F511_DecodeDataReceived(Uart.UartReceivedData(), TYPE_WRITE_NO_LOAD_THRESHOLD));
}
/****************************************************************************/
//============================================================================================================