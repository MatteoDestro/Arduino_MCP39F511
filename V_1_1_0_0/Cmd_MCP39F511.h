/*********************************************************************
 *
 *       Generic command for MCP39F511
 *
 *********************************************************************
 * FileName:        Cmd_MCP39F511.h
 * Revision:        1.0.0 (First issue)
 * Date:            04/05/2019
 *
 * Revision:        1.1.0
 *                  28/12/2019
 *                  - Added function "uint16_t Cmd_MCP39F511::MCP39F511_RmsVoltageRawAverage(void)"
 *                  - Added function "uint16_t Cmd_MCP39F511::MCP39F511_LineFrequencyRawAverage(void)"
 *                  - Added function "uint16_t Cmd_MCP39F511::MCP39F511_PowerFactorRawAverage(void)"
 *                  - Added function "unsigned long Cmd_MCP39F511::CalcAveragePowerFactor(void)"
 *                  - Added function "uint32_t Cmd_MCP39F511::MCP39F511_RmsCurrentRawAverage(void)"
 *                  - Added function "uint32_t Cmd_MCP39F511::MCP39F511_ActivePowerRawAverage(void)"
 *                  - Added function "uint32_t Cmd_MCP39F511::MCP39F511_ReactivePowerRawAverage(void)"
 *                  - Added function "uint32_t Cmd_MCP39F511::MCP39F511_ApparentPowerRawAverage(void)"
 *                  - Added arrays and variables used to calc average value
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

#ifndef _MCP39F511_H_INCLUDED__
#define _MCP39F511_H_INCLUDED__

#include "Arduino.h"
#include "Typedef_MCP39F511.h"

//=========================================================================================================================
/*
 ___________________________________________________________________________________________
|                 |                                                                         |
|  MCP39F511 INT  |                                                                         |
|   EEPROM ADD    | DESCRIPTION                                                             |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 0                                            |
|-------------------------------------------------------------------------------------------|
|     0x0000      | (u16) -> Gain Current RMS                                               |
|     0x0002      | (u16) -> Gain Voltage RMS                                               |
|     0x0004      | (u16) -> Gain Active Power                                              |
|     0x0006      | (u16) -> Gain Reactive Power                                            |
|     0x0008      | (s32) -> Offset Current RMS                                             |
|     0x000C      | (s32) -> Offset Active Power                                            |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 1                                            |
|-------------------------------------------------------------------------------------------|
|     0x0010      | (s32) -> Offset Reactive Power                                          |
|     0x0014      | (s16) -> DC Offset Current                                              |
|     0x0016      | (s16) -> Phase Compensation                                             |
|     0x0018      | (u16) -> Apparent Power Divisor                                         |
|     0x001A      | (b32) -> System Configuration                                           |
|     0x001E      | (u16) -> Free (Not Used)                                                |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 2                                            |
|-------------------------------------------------------------------------------------------|
|     0x0020      | (b32) -> Event Configuration                                            |
|     0x0024      | (b32) -> Range                                                          |
|     0x0028      | (u32) -> Calibration Current                                            |
|     0x002C      | (u16) -> Calibration Voltage                                            |
|     0x002E      | (u16) -> Free (Not Used)                                                |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 3                                            |
|-------------------------------------------------------------------------------------------|
|     0x0030      | (u32) -> Calibration Active Power                                       |
|     0x0034      | (u32) -> Calibration Reactive Power                                     |
|     0x0038      | (u16) -> Line Frequency Reference                                       |
|     0x003A      | (u16) -> Accumulator Interval Parameter                                 |
|     0x003C      | (u16) -> Voltage Sag Limit                                              |
|     0x003E      | (u16) -> Voltage Surge Limit                                            |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 4                                            |
|-------------------------------------------------------------------------------------------|
|     0x0040      | (u32) -> OverCurrent Limit                                              |
|     0x0044      | (u32) -> OverPower Limit                                                |
|     0x0048      | (u16) -> Temperature Compensation for Frequency                         |
|     0x004A      | (u16) -> Temperature Compensation for Current                           |
|     0x004C      | (u16) -> Temperature Compensation for Power                             |
|     0x004E      | (u16) -> Ambient Temperature Reference Voltage                          |
|-------------------------------------------------------------------------------------------|
|                                         PAGE 5                                            |
|-------------------------------------------------------------------------------------------|
|     0x0050      | (u16) -> PWM Period                                                     |
|     0x0052      | (u16) -> PWM Duty Cycle                                                 |
|     0x0054      | (u16) -> MinMax Pointer 1                                               |
|     0x0056      | (u16) -> MinMax Pointer 2                                               |
|     0x0058      | (u16) -> OverTemperature Limit                                          |
|     0x005A      | (u16) -> Energy Control                                                 |
|     0x005C      | (u16) -> PWM Control                                                    |
|     0x005E      | (u16) -> No Load Threshold                                              |
|-------------------------------------------------------------------------------------------|
|                                   PAGE 6 - Page 31                                        |
|-------------------------------------------------------------------------------------------|
| 0x0060 - 0x01FF | Reserved (Free Space)                                                   |
|_________________|_________________________________________________________________________|
*/
//=========================================================================================================================

//=========================================================================================================================
//  MCP39F511 FACTORY CALIBRATION PARAMETERS (FT1346M). [MCP39F511 EEPROM ADDRESS]
#define GAIN_CURRENT_RMS_EEADD                0x0000
#define GAIN_VOLTAGE_RMS_EEADD                0x0002
#define GAIN_ACTIVE_POWER_EEADD               0x0004
#define GAIN_REACTIVE_POWER_EEADD             0x0006
#define OFFSET_CURRENT_RMS_EEADD              0x0008
#define OFFSET_ACTIVE_POWER_EEADD             0x000C
#define OFFSET_REACTIVE_POWER_EEADD           0x0010
#define DC_OFFSET_CURRENT_EEADD               0x0014
#define PHASE_COMPENSATION_EEADD              0x0016
#define APPARENT_POWER_DIVISOR_EEADD          0x0018

#define SYSTEM_CONFIGURATION_EEADD            0x001A
#define EVENT_CONFIGURATION_EEADD             0x0020
#define RANGE_EEADD                           0x0024
#define CALIBRATION_CURRENT_EEADD             0x0028
#define CALIBRATION_VOLTAGE_EEADD             0x002C
#define CALIBRATION_ACTIVE_POWER_EEADD        0x0030
#define CALIBRATION_REACTIVE_POWER_EEADD      0x0034
#define LINE_FREQUENCY_REFERENCE_EEADD        0x0038
#define ACCUMULATOR_INTERVAL_PARAMETER_EEADD  0x003A
#define VOLTAGE_SAG_LIMIT_EEADD               0x003C
#define VOLTAGE_SURGE_LIMIT_EEADD             0x003E
#define OVER_CURRENT_LIMIT_EEADD              0x0040
#define OVER_POWER_LIMIT_EEADD                0x0044
#define TEMP_COMPENSATION_FOR_FREQUENCY_EEADD 0x0048
#define TEMP_COMPENSATION_FOR_CURRENT_EEADD   0x004A
#define TEMP_COMPENSATION_FOR_POWER_EEADD     0x004C
#define AMBIENT_TEMP_REFERENCE_EEADD          0x004E
#define PWM_PERIOD_EEADD                      0x0050
#define PWM_DUTY_CYCLE_EEADD                  0x0052
#define MINMAX_POINTER_1_EEADD                0x0054
#define MINMAX_POINTER_2_EEADD                0x0056
#define OVERTEMPERATURE_LIMIT_EEADD           0x0058
#define ENERGY_CONTROL_EEADD                  0x005A
#define PWM_CONTROL_EEADD                     0x005C
#define NO_LOAD_THRESHOLD_EEADD               0x005E
//=========================================================================================================================

//=========================================================================================================================
//  PAGE NUMBER. [MCP39F511 EEPROM ADDRESS]
#define EEPROM_PAGE_00   0
#define EEPROM_PAGE_01   1
#define EEPROM_PAGE_02   2
#define EEPROM_PAGE_03   3
#define EEPROM_PAGE_04   4
#define EEPROM_PAGE_05   5
#define EEPROM_PAGE_06   6
#define EEPROM_PAGE_07   7
#define EEPROM_PAGE_08   8
#define EEPROM_PAGE_09   9
#define EEPROM_PAGE_10   10
#define EEPROM_PAGE_11   11
#define EEPROM_PAGE_12   12
#define EEPROM_PAGE_13   13
#define EEPROM_PAGE_14   14
#define EEPROM_PAGE_15   15
#define EEPROM_PAGE_16   16
#define EEPROM_PAGE_17   17
#define EEPROM_PAGE_18   18
#define EEPROM_PAGE_19   19
#define EEPROM_PAGE_20   20
#define EEPROM_PAGE_21   21
#define EEPROM_PAGE_22   22
#define EEPROM_PAGE_23   23
#define EEPROM_PAGE_24   24
#define EEPROM_PAGE_25   25
#define EEPROM_PAGE_26   26
#define EEPROM_PAGE_27   27
#define EEPROM_PAGE_28   28
#define EEPROM_PAGE_29   29
#define EEPROM_PAGE_30   30
#define EEPROM_PAGE_31   31
//=========================================================================================================================

//======================================================================
//	MCP39F511 COMMANDS CODE
#define REGISTER_READ_N                 0x4E
#define REGISTER_WRITE_N                0x4D
#define SET_ADDRESS_POINTER             0x41
#define SAVE_REGISTER_TO_FLASH          0x53
#define PAGE_READ_EEPROM                0x42
#define PAGE_WRITE_EEPROM               0x50
#define BULK_ERASE_EEPROM               0x4F
#define AUTO_CALIBRATE_GAIN             0x5A
#define AUTO_CALIBRATE_REACTIVE_GAIN    0x7A
#define AUTO_CALIBRATE_FREQUENCY        0x76

#define RX_ACK                          0x06    //  Acknowledge
#define RX_NACK                         0x15    //  NO Acknowledge
#define RX_CHK_FAIL                     0x51    //  Checksum fail

#define HEADER_BYTE		                0xA5

#define RETRIEVE_FACTORY_DEFAULT_CODE   0xA5A5
//======================================================================

//======================================================================
//	COMMANDS TYPE USED TO DECODE THE ANSWER RECEIVED FROM MCP39F511
#define TYPE_STATUS_VERSION                           0x00
#define TYPE_READ_V_RMS                               0x01
#define TYPE_READ_LINE_FREQ                           0x02
#define TYPE_READ_IN_ANAL_V                           0x03
#define TYPE_READ_P_FACTOR                            0x04
#define TYPE_READ_I_RMS                               0x05
#define TYPE_READ_P_ACTIVE                            0x06
#define TYPE_READ_P_REACTIVE                          0x07
#define TYPE_READ_P_APPARENT                          0x08
#define TYPE_READ_E_ACTIVE_IMP                        0x09
#define TYPE_READ_E_ACTIVE_EXP                        0x0A
#define TYPE_READ_E_REACTIVE_IMP                      0x0B
#define TYPE_READ_E_REACTIVE_EXP                      0x0C
#define TYPE_READ_SYSTEM_CONFIGURATION                0x0D
#define TYPE_READ_EVENT_CONFIGURATION                 0x0E
#define TYPE_READ_MINIMUM_RECORD_1                    0x0F
#define TYPE_READ_MINIMUM_RECORD_2                    0x10
#define TYPE_READ_MAXIMUM_RECORD_1                    0x11
#define TYPE_READ_MAXIMUM_RECORD_2                    0x12

#define TYPE_WRITE_RETRIEVE_FACTORY_DEFAULT           0x40
#define TYPE_WRITE_GAIN_CURRENT_RMS                   0x41
#define TYPE_WRITE_GAIN_VOLTAGE_RMS                   0x42 
#define TYPE_WRITE_GAIN_ACTIVE_POWER                  0x43
#define TYPE_WRITE_GAIN_REACTIVE_POWER                0x44
#define TYPE_WRITE_OFFSET_CURRENT_RMS                 0x45
#define TYPE_WRITE_OFFSET_ACTIVE_POWER                0x46
#define TYPE_WRITE_OFFSET_REACTIVE_POWER              0x47
#define TYPE_WRITE_DC_OFFSET_CURRENT                  0x48
#define TYPE_WRITE_PHASE_COMPENSATION                 0x49
#define TYPE_WRITE_APPARENT_POWER_DIVISOR             0x4A
#define TYPE_WRITE_SYSTEM_CONFIGURATION               0x4B
#define TYPE_WRITE_EVENT_CONFIGURATION                0x4C
#define TYPE_WRITE_RANGE                              0x4D
#define TYPE_WRITE_CALIBRATION_CURRENT                0x4E
#define TYPE_WRITE_CALIBRATION_VOLTAGE                0x4F
#define TYPE_WRITE_CALIBRATION_ACTIVE_POWER           0x50
#define TYPE_WRITE_CALIBRATION_REACTIVE_POWER         0x51
#define TYPE_WRITE_LINE_FREQUENCY_REFERENCE           0x52
#define TYPE_WRITE_ACCUMULATOR_INTERVAL_PARAMETER     0x53
#define TYPE_WRITE_VOLTAGE_SAG_LIMIT                  0x54
#define TYPE_WRITE_VOLTAGE_SURGE_LIMIT                0x55
#define TYPE_WRITE_OVER_CURRENT_LIMIT                 0x56
#define TYPE_WRITE_OVER_POWER_LIMIT                   0x57
#define TYPE_WRITE_TEMP_COMPENSATION_FOR_FREQUENCY    0x58
#define TYPE_WRITE_TEMP_COMPENSATION_FOR_CURRENT      0x59
#define TYPE_WRITE_TEMP_COMPENSATION_FOR_POWER        0x5A
#define TYPE_WRITE_AMBIENT_TEMP_REFERENCE             0x5B
#define TYPE_WRITE_PWM_PERIOD                         0x5C
#define TYPE_WRITE_PWM_DUTY_CYCLE                     0x5D
#define TYPE_WRITE_MINMAX_POINTER_1                   0x5E
#define TYPE_WRITE_MINMAX_POINTER_2                   0x5F
#define TYPE_WRITE_OVERTEMPERATURE_LIMIT              0x60
#define TYPE_WRITE_ENERGY_CONTROL                     0x61
#define TYPE_WRITE_PWM_CONTROL                        0x62
#define TYPE_WRITE_NO_LOAD_THRESHOLD                  0x63

#define TYPE_READ_EEPROM_PAGE_0                       0x80
#define TYPE_READ_EEPROM_PAGE_1                       0x81
#define TYPE_READ_EEPROM_PAGE_2                       0x82
#define TYPE_READ_EEPROM_PAGE_3                       0x83
#define TYPE_READ_EEPROM_PAGE_4                       0x84
#define TYPE_READ_EEPROM_PAGE_5                       0x85
#define TYPE_READ_EEPROM_PAGE_6                       0x86
#define TYPE_READ_EEPROM_PAGE_7                       0x87
#define TYPE_READ_EEPROM_PAGE_8                       0x88
#define TYPE_READ_EEPROM_PAGE_9                       0x89
#define TYPE_READ_EEPROM_PAGE_10                      0x8A
#define TYPE_READ_EEPROM_PAGE_11                      0x8B
#define TYPE_READ_EEPROM_PAGE_12                      0x8C
#define TYPE_READ_EEPROM_PAGE_13                      0x8D
#define TYPE_READ_EEPROM_PAGE_14                      0x8E
#define TYPE_READ_EEPROM_PAGE_15                      0x8F
#define TYPE_READ_EEPROM_PAGE_16                      0x90
#define TYPE_READ_EEPROM_PAGE_17                      0x91
#define TYPE_READ_EEPROM_PAGE_18                      0x92
#define TYPE_READ_EEPROM_PAGE_19                      0x93
#define TYPE_READ_EEPROM_PAGE_20                      0x94
#define TYPE_READ_EEPROM_PAGE_21                      0x95
#define TYPE_READ_EEPROM_PAGE_22                      0x96
#define TYPE_READ_EEPROM_PAGE_23                      0x97
#define TYPE_READ_EEPROM_PAGE_24                      0x98
#define TYPE_READ_EEPROM_PAGE_25                      0x99
#define TYPE_READ_EEPROM_PAGE_26                      0x9A
#define TYPE_READ_EEPROM_PAGE_27                      0x9B
#define TYPE_READ_EEPROM_PAGE_28                      0x9C
#define TYPE_READ_EEPROM_PAGE_29                      0x9D
#define TYPE_READ_EEPROM_PAGE_30                      0x9E
#define TYPE_READ_EEPROM_PAGE_31                      0x9F

#define TYPE_WRITE_EEPROM_PAGE                        0xA1
#define TYPE_ERASE_BULK_EEPROM                        0xA2

#define TYPE_FREEZE_FLASH                             0xFF    //  Save register into FLASH
//======================================================================

//======================================================================
//	MCP39F511 DATA DECODE RESULT
#define DECODE_DATA_OK                                0x00
#define DECODE_RECEIVED_LESS_DATA                     0x01
#define DECODE_CHECKSUM_ERROR                         0x02
#define DECODE_NO_ACKNOWLEDGE                         0x03
//======================================================================

//======================================================================
//  MCP39F511 DATA PACKET LENGHT
#define CMD_READ_DATA_LENGHT            8
#define CMD_READ_PAGE_EEPROM_LENGHT     5

#define CMD_ERASE_BULK_EEPROM_LENGHT    4
#define CMD_WRITE_PAGE_EEPROM_LENGHT    21
#define CMD_WRITE_BYTE_DATA_LENGHT      9
#define CMD_WRITE_WORD_DATA_LENGHT      10
#define CMD_WRITE_DWORD_DATA_LENGHT     12
#define CMD_FREEZE_FLASH_DATA_LENGHT    4

#define LENGHT_READ_PAGE_EEPROM_ANSWER  19
//======================================================================

//======================================================================
#define VOLTAGE_DEC         2   //  Voltage decimals
#define CURRENT_DEC         3   //  Current decimals
#define FREQUENCY_DEC       3   //  Frequency decimals
#define POWER_FACTOR_DEC    3   //  Power Factor decimals
#define APPARENT_POWER_DEC  2   //  Apparent Power decimals
#define ACTIVE_POWER_DEC    2   //  Active Power decimals
#define REACTIVE_POWER_DEC  2   //  Reactive Power decimals
//======================================================================

//======================================================================
#define ENABLE_AVERAGE_CALC      // ENABLE/DISABLE Average Calc Functions. To Disable functions to calc average value, comments this line
#define AVERAGE_ARRAY_SIZE  16   // AVERAGE ARRAY SIZE (Value accepted are: 4; 8; 16; 32)
//======================================================================

//======================================================================
//	MCP39F511 COMPLETE REGISTER MAP (ADDRESS))
#define INSTRUCTION_POINTER_ADD             0x0000  //  (u16) Address pointer for read or write commands
#define SYSTEM_STATUS_ADD                   0x0002  //  (b16) System status register
#define SYSTEM_VERSION_ADD                  0x0004  //  (u16) System versione date code information for MCP39F511
#define VOLTAGE_RMS_ADD                     0x0006  //  (u16) RMS Voltage Output
#define LINE_FREQUENCY_ADD                  0x0008  //  (u16) Line frequency output
#define ANALOG_INPUT_VOLTAGE_ADD            0x000A  //  (u16) Output of the 10Bit SAR ADC
#define POWER_FACTOR_ADD                    0x000C  //  (s16) Power Factor Output
#define CURRENT_RMS_ADD                     0x000E  //  (u32) RMS Current Output
#define ACTIVE_POWER_ADD                    0x0012  //  (u32) Active Power Output
#define REACTIVE_POWER_ADD                  0x0016  //  (u32) Reactive Power Output
#define APPARENT_POWER_ADD                  0x001A  //  (u32) Apparent Power Output
#define IMPORT_ACTIVE_ENERGY_COUNT_ADD      0x001E  //  (u64) Accumulator for Active Energy, Import
#define EXPORT_ACTIVE_ENERGY_COUNT_ADD      0x0026  //  (u64) Accumulator for Active Energy, Export
#define IMPORT_REACTIVE_ENERGY_COUNT_ADD    0x002E  //  (u64) Accumulator for Reactive Energy, Import
#define EXPORT_REACTIVE_ENERGY_COUNT_ADD    0x0036  //  (u64) Accumulator for Reactive Energy, Export
#define MINIMUM_RECORD_1_ADD                0x003E  //  (u32) Minimum value of the Output Quantity Address in Min/Max Pointer 1 Register
#define MINIMUM_RECORD_2_ADD                0x0042  //  (u32) Minimum value of the Output Quantity Address in Min/Max Pointer 2 Register
#define RESERVED_1_ADD                      0x0046  //  (u32) Reserved
#define RESERVED_2_ADD                      0x004A  //  (u32) Reserved
#define MAXIMUM_RECORD_1_ADD                0x004E  //  (u32) Maximum value of the Output Quantity Address in Min/Max Pointer 1 Register
#define MAXIMUM_RECORD_2_ADD                0x0052  //  (u32) Maximum value of the Output Quantity Address in Min/Max Pointer 2 Register
#define RESERVED_3_ADD                      0x0056  //  (u32) Reserved
#define RESERVED_4_ADD                      0x005A  //  (u32) Reserved

#define CALIBRATION_REGISTER_DELIMITER_ADD  0x005E  //  (u16) May be used to initiate loading of the default calibration coefficients at start-up
#define GAIN_CURRENT_RMS_ADD                0x0060  //  (u16) Gain calibration factor for RMS current
#define GAIN_VOLTAGE_RMS_ADD                0x0062  //  (u16) Gain calibration factor for RMS Voltage
#define GAIN_ACTIVE_POWER_ADD               0x0064  //  (u16) Gain calibration factor for Active Power
#define GAIN_REACTIVE_POWER_ADD             0x0066  //  (u16) Gain calibration factor for Reactive Power
#define OFFSET_CURRENT_RMS_ADD              0x0068  //  (s32) Offset calibration factor for RMS current
#define OFFSET_ACTIVE_POWER_ADD             0x006C  //  (s32) Offset calibration factor for Active Power
#define OFFSET_REACTIVE_POWER_ADD           0x0070  //  (s32) Offset calibration factor for Reactive Power
#define DC_OFFSET_CURRENT_ADD               0x0074  //  (s16) Offset calibration for DC current
#define PHASE_COMPENSATION_ADD              0x0076  //  (s16) Phase compensation
#define APPARENT_POWER_DIVISOR_ADD          0x0078  //  (u16) Number of digits for apparent power divisor to match Irms and Vrms resolution

#define SYSTEM_CONFIGURATION_ADD            0x007A  //  (b32) Control for device configuration, including ADC configuration
#define EVENT_CONFIGURATION_ADD             0x007E  //  (b32) Settings for the Event pins including Relay Control
#define RANGE_ADD                           0x0082  //  (b32) Scaling factor for output
#define CALIBRATION_CURRENT_ADD             0x0086  //  (u32) Target Current to be used during single-point calibration
#define CALIBRATION_VOLTAGE_ADD             0x008A  //  (u16) Target Voltage to be used during single-point calibration
#define CALIBRATION_ACTIVE_POWER_ADD        0x008C  //  (u32) Target Active Power to be used during single-point calibration
#define CALIBRATION_REACTIVE_POWER_ADD      0x0090  //  (u32) Target Reactive Power to be used during single-point calibration
#define LINE_FREQUENCY_REFERENCE_ADD        0x0094  //  (u16) Reference value for the nominal line frequency
#define RESERVED_5_ADD                      0x0096  //  (u16) Reserved
#define RESERVED_6_ADD                      0x009A  //  (u32) Reserved
#define ACCUMULATOR_INTERVAL_PARAMETER_ADD  0x009E  //  (u16) N for 2^N number of line cycles to be used during a single computation cycle 
#define VOLTAGE_SAG_LIMIT_ADD               0x00A0  //  (u16) RMS Voltage threshold at witch an event flag is recorded
#define VOLTAGE_SURGE_LIMIT_ADD             0x00A2  //  (u16) RMS Voltage threshold at witch an event flag is recorded
#define OVER_CURRENT_LIMIT_ADD              0x00A4  //  (u32) RMS Current threshold at witch an event flag is recorded
#define OVER_POWER_LIMIT_ADD                0x00A8  //  (u32) RMS Active Power threshold at witch an event flag is recorded
#define RESERVED_7_ADD                      0x00AC  //  (u16) Reserved
#define RESERVED_8_ADD                      0x00AE  //  (u16) Reserved
#define RESERVED_9_ADD                      0x00B0  //  (u16) Reserved
#define RESERVED_10_ADD                     0x00B2  //  (u16) Reserved
#define RESERVED_11_ADD                     0x00B4  //  (u16) Reserved
#define RESERVED_12_ADD                     0x00B6  //  (u16) Reserved
#define RESERVED_13_ADD                     0x00B8  //  (u16) Reserved
#define RESERVED_14_ADD                     0x00BA  //  (u16) Reserved
#define RESERVED_15_ADD                     0x00BC  //  (u16) Reserved
#define RESERVED_16_ADD                     0x00BE  //  (u16) Reserved
#define RESERVED_17_ADD                     0x00C0  //  (u16) Reserved
#define RESERVED_18_ADD                     0x00C2  //  (u16) Reserved
#define RESERVED_19_ADD                     0x00C4  //  (u16) Reserved
#define TEMP_COMPENSATION_FOR_FREQUENCY_ADD 0x00C6  //  (u16) Correction factor for compensating the line frequency indication overtemperature 
#define TEMP_COMPENSATION_FOR_CURRENT_ADD   0x00C8  //  (u16) Correction factor for compensating the current RMS indication overtemperature 
#define TEMP_COMPENSATION_FOR_POWER_ADD     0x00CA  //  (u16) Correction factor for compensating the active power indication overtemperature 
#define AMBIENT_TEMP_REFERENCE_ADD          0x00CC  //  (u16) Register for storing the reference temperature during calibration
#define PWM_PERIOD_ADD                      0x00CE  //  (u16) Input register controlling PWM period
#define PWM_DUTY_CYCLE_ADD                  0x00D0  //  (u16) Input register controlling PWM duty cycle
#define RESERVED_20_ADD                     0x00D2  //  (u16) Reserved
#define MINMAX_POINTER_1_ADD                0x00D4  //  (u16) Address Pointer for Min/Max 1 Outputs
#define MINMAX_POINTER_2_ADD                0x00D6  //  (u16) Address Pointer for Min/Max 1 Outputs
#define OVERTEMPERATURE_LIMIT_ADD           0x00D8  //  (u16) Limit at which an Overtemperature event flag is recorded
#define RESERVED_21_ADD                     0x00DA  //  (u16) Reserved
#define ENERGY_CONTROL_ADD                  0x00DC  //  (u16) Input register for Reset/Start of energy accumulation
#define PWM_CONTROL_ADD                     0x00DE  //  (u16) Input register for PWM On/Off and other PWM controls
#define NO_LOAD_THRESHOLD_ADD               0x00E0  //  (u16) No load threshold for energy Counting
//======================================================================

//======================================================================
//	MCP39F511 CONFIGURATION REGISTER
#define SYSTEM_CONFIGURATION_REG    0x00420000      //  0b00000000010000100000000000000000
                                                    //    ||||||||||||||||||||||||||||||||
                                                    //    ||||||||||||||||||||||||||||||++--------> Unimplemented
                                                    //    |||||||||||||||||||||||||||||+----------> VREFEXT: Internal Voltage Reference Shutdown Control
                                                    //    |||||||||||||||||||||||||||++-----------> SHUTDOWN <1:0>: Shutdown mode setting for ADCs
                                                    //    |||||||||||||||||||||||||++-------------> RESET <1:0>: Reset mode setting for ADCs
                                                    //    ||||||||||||||||||||||||+---------------> TEMPCOMP: Temperature-Compensation Enable bit
                                                    //    |||||||||||||||||||||||+----------------> SINGLE_WIRE: Single-Wire Enable bit
                                                    //    ||||||||||||||||||||||+-----------------> Unimplemented
                                                    //    |||||||||||||||||||||+------------------> ZCD_OUTPUT_DIS: Disable the Zero Crossing output pin
                                                    //    ||||||||||||||||||||+-------------------> ZCD_PULS: Zero Crossing Detection Pulse mode
                                                    //    |||||||||||||||||||+--------------------> ZCD_INV: Zero Crossing Detection Output Inverse
                                                    //    ||||||||||||||||+++---------------------> UART<2:0>: UART Baud Rate bits "000" -> 115200 Default
                                                    //    ||||||||++++++++------------------------> VREFCAL<n>: Internal voltage reference temperature coefficient register value
                                                    //    |||||+++--------------------------------> PGA_CH0 <2:0>: PGA Setting for the voltage channel
                                                    //    ||+++-----------------------------------> PGA_CH1 <2:0>: PGA Setting for the voltage channel
                                                    //    ++--------------------------------------> Unimplemented

#define EVENT_CONFIGURATION_REG     0x00280000      //  0b00000000001010000000000000000000
                                                    //    ||||||||||||||||||||||||||||||||
                                                    //    |||||||||||||||||||||||||||||||+--------> OVERCUR_TST: Test control of the Overcurrent event
                                                    //    ||||||||||||||||||||||||||||||+---------> OVERPOW_TST: Test control of the Overpower event
                                                    //    |||||||||||||||||||||||||||||+----------> VSAG_TST: Test control of the Voltage Sag event
                                                    //    ||||||||||||||||||||||||||||+-----------> VSUR_TST: Test control of the Voltage Surge event
                                                    //    |||||||||||||||||||||||||||+------------> OVERCUR_LA: Latching control of the Overcurrent event
                                                    //    ||||||||||||||||||||||||||+-------------> OVERPOW_LA: Latching control of the Overpower event
                                                    //    |||||||||||||||||||||||||+--------------> VSAG_LA: Latching control of the Voltage Sag event
                                                    //    ||||||||||||||||||||||||+---------------> VSUR_LA: Latching control of the Voltage Surge event
                                                    //    |||||||||||||||||||||||+----------------> VSAG_CL: Reset or clear bit for the Voltage Sag even
                                                    //    ||||||||||||||||||||||+-----------------> VSUR_CL: Reset or clear bit for the Voltage Surge even
                                                    //    |||||||||||||||||||||+------------------> OVERPOW_CL: Reset or clear bit for the Overpower event
                                                    //    ||||||||||||||||||||+-------------------> OVERCUR_CL: Reset or clear bit for the Overcurrent event
                                                    //    ||||||||||||||||++++--------------------> Unimplemented
                                                    //    |||||||||||||||+------------------------> VSAG_PIN1: Event pin 1 operation for the Voltage Sag event
                                                    //    ||||||||||||||+-------------------------> VSURGE_PIN1: Event pin 1 operation for the Voltage Surge event
                                                    //    |||||||||||||+--------------------------> OVERCUR_PIN1: Event pin 1 operation for the Overcurrent event
                                                    //    ||||||||||||+---------------------------> OVERPOW_PIN1: Event pin 1 operation for the Overpower event
                                                    //    |||||||||||+----------------------------> VSAG_PIN2: Event pin 2 operation for the Voltage Sag event
                                                    //    ||||||||||+-----------------------------> VSURGE_PIN2: Event pin 2 operation for the Voltage Surge event
                                                    //    |||||||||+------------------------------> OVERCUR_PIN2: Event pin 2 operation for the Overcurrent event
                                                    //    ||||||||+-------------------------------> OVERPOW_PIN2: Event pin 2 operation for the Overpower event
                                                    //    |||||||+--------------------------------> OVERTEMP_TST: Test control of the Overtemperature event 
                                                    //    ||||||+---------------------------------> OVERTEMP_LA: Latching control of the Overtemperature event
                                                    //    |||||+----------------------------------> OVERTEMP_CL: Reset or clear bit for the Overtemperature even
                                                    //    ||||+-----------------------------------> OVERTEMP_PIN1: Event pin 1 operation for the Overtemperature even
                                                    //    |||+------------------------------------> OVERTEMP_PIN2: Event pin 2 operation for the Overtemperature event
                                                    //    +++-------------------------------------> Unimplemented

#define ENERGY_ACCUMULATION_CONTROL_REG 0x0001      //  0b0000000000000001
                                                    //    ||||||||||||||||
                                                    //    |||||||||||||||+--------> ENRG_CNTRL: Energy Accumulation Control Bit. "1" Energy is ON and all registers are accumulating
                                                    //    +++++++++++++++---------> Unimplemented

#define NO_LOAD_THRESHOLD_REG   0x03E8              //  10W Threshold (Res 0.01W). Any power that is below 10w will not be accumulated into any of the energy registers


#define ACCUMULATOR_REG         0x0005              //  N for 2^N number of line cycles to be used during a single computation cycle.
                                                    //  With N = 5 the number of line cycles is 32 
//======================================================================

//======================================================================
typedef	union {
	uint16_t	uWord;
	struct {
		uint8_t	Byte0;
		uint8_t	Byte1;
	};
	struct
    {
		uint8_t Vsag		:1; //  State of Voltage Sag Detection algorithm. This bit is latched and must be cleared
		uint8_t Vsurge		:1; //  State of Voltage Surge Detection algorithm. This bit is latched and must be cleared
		uint8_t OverCur		:1; //  State of the Overcurrent detection algorithm. An Overcurrent event has occurred in the system
        uint8_t OverPow		:1; //  State of Overpower detection algorithm. An Overpower event has occurred in the system
        uint8_t Sign_PA		:1; //  Sign of Active Power (import/export sign of active power)
        uint8_t Sign_PR		:1; //  Sign of Reactive Power
        uint8_t OverTemp	:1; //  State of the Overtemperature Detection Algorithm
        uint8_t Free1		:3;
        uint8_t Event1		:1; //  State of Event1 Detection algorithm. This bit is latched and must be cleared
        uint8_t Event2		:1; //  State of Event2 Detection algorithm. This bit is latched and must be cleared
		uint8_t Free2		:4;
	};
} SystemStatusFlag;

typedef	union {
	uint16_t	uWord;
	struct {
		uint8_t	Byte0;
		uint8_t	Byte1;
	};
	struct
    {
		uint8_t Year		:4;
		uint8_t Month 		:4;
		uint8_t Day   		:8;
	};
} SystemVersionFlag;

typedef union {
	long 	 l;
	uint32_t uDWord;
	uint8_t	 Byte0;
	uint8_t	 Byte1;
	uint8_t	 Byte2;
	uint8_t	 Byte3;
	struct {
		uint16_t Word0;
		uint16_t Word1;
	};
	struct {
		uint8_t Free_1         :2; //  Unimplemented
        uint8_t VrefExt        :1; //  VREFEXT:        Internal Voltage Reference Shutdown Control
        uint8_t ShutDown       :2; //  SHUTDOWN <1:0>: Shutdown mode setting for ADCs
        uint8_t Reset          :2; //  RESET <1:0>:    Reset mode setting for ADCs
        uint8_t TempComp       :1; //  TEMPCOMP:       Temperature-Compensation Enable bit
        uint8_t SingleWire     :1; //  SINGLE_WIRE:    Single-Wire Enable bit
        uint8_t Free_2         :2; //  Unimplemented
        uint8_t ZcdOutput_Dis  :1; //  ZCD_OUTPUT_DIS: Disable the Zero Crossing output pin
        uint8_t ZcdPuls        :1; //  ZCD_PULS:       Zero Crossing Detection Pulse mode
        uint8_t ZcdInv         :1; //  ZCD_INV:        Zero Crossing Detection Output Inverse
        uint8_t UartBaud       :3; //  UART<2:0>:      UART Baud Rate bits "000" -> 115200 Default
        uint8_t VrefCal        :8; //  VREFCAL<n>:     Internal voltage reference temperature coefficient register value
        uint8_t PgaCh0         :3; //  PGA_CH0 <2:0>:  PGA Setting for the voltage channel
        uint8_t PgaCh1         :3; //  PGA_CH1 <2:0>:  PGA Setting for the voltage channel
        uint8_t Free_3         :2; //  Unimplemented
	};
} SystemConfigurationFlag;

typedef union {
	long	 l;
	uint32_t uDWord;
	uint8_t	 Byte0;
	uint8_t	 Byte1;
	uint8_t	 Byte2;
	uint8_t	 Byte3;
	struct {
		uint16_t Word0;
		uint16_t Word1;
	};
	struct {
		uint8_t OverCurTst    :1; //  OVERCUR_TST:   Test control of the Overcurrent event
        uint8_t OverPowTst    :1; //  OVERPOW_TST:   Test control of the Overpower event
        uint8_t VsagTst       :1; //  VSAG_TST:      Test control of the Voltage Sag event
        uint8_t VsurTst       :1; //  VSUR_TST:      Test control of the Voltage Surge event
        uint8_t OverCurLa     :1; //  OVERCUR_LA:    Latching control of the Overcurrent event
        uint8_t OverPowLa     :1; //  OVERPOW_LA:    Latching control of the Overpower event
        uint8_t VsagLa        :1; //  VSAG_LA:       Latching control of the Voltage Sag event
        uint8_t VsurLa        :1; //  VSUR_LA:       Latching control of the Voltage Surge event
        uint8_t VsagCl        :1; //  VSAG_CL:       Reset or clear bit for the Voltage Sag even
        uint8_t VsurCl        :1; //  VSUR_CL:       Reset or clear bit for the Voltage Surge even
        uint8_t OverPowCl     :1; //  OVERPOW_CL:    Reset or clear bit for the Overpower event
        uint8_t OverCurCl     :1; //  OVERCUR_CL:    Reset or clear bit for the Overcurrent event
        uint8_t Free_1        :4; //  Unimplemented
        uint8_t VsagPin1      :1; //  VSAG_PIN1:     Event pin 1 operation for the Voltage Sag event
        uint8_t VsurgePin1    :1; //  VSURGE_PIN1:   Event pin 1 operation for the Voltage Surge event
        uint8_t OverCurPin1   :1; //  OVERCUR_PIN1:  Event pin 1 operation for the Overcurrent event
        uint8_t OverPowPin1   :1; //  OVERPOW_PIN1:  Event pin 1 operation for the Overpower event
        uint8_t VsagPin2      :1; //  VSAG_PIN2:     Event pin 2 operation for the Voltage Sag event
        uint8_t VsurgePin2    :1; //  VSURGE_PIN2:   Event pin 2 operation for the Voltage Surge event
        uint8_t OverCurPin2   :1; //  OVERCUR_PIN2:  Event pin 2 operation for the Overcurrent event
        uint8_t OverPowPin2   :1; //  OVERPOW_PIN2:  Event pin 2 operation for the Overpower event        
        uint8_t OverTempTest  :1; //  OVERTEMP_TST:  Test control of the Overtemperature event 
        uint8_t OverTempLa    :1; //  OVERTEMP_LA:   Latching control of the Overtemperature event
        uint8_t OverTempCl    :1; //  OVERTEMP_CL:   Reset or clear bit for the Overtemperature even
        uint8_t OverTempPin1  :1; //  OVERTEMP_PIN1: Event pin 1 operation for the Overtemperature event
        uint8_t OverTempPin2  :1; //  OVERTEMP_PIN2: Event pin 2 operation for the Overtemperature event
        uint8_t Free_2        :3; //  Unimplemented
	};
} EventConfigurationFlag;

typedef	union {
	uint16_t	uWord;
	struct {
		uint8_t	Byte0;
		uint8_t	Byte1;
	};
	struct
    {
		uint8_t PowerFactorSign             :1;  //  Sign of Power Factor "1" negative sign; "0" positive sign
        uint8_t PowerFactorAverageSign      :1;  //  Sign of Power Factor Average "1" negative sign; "0" positive sign
        uint8_t ReadDataInProgress          :1;  //  If "1" reads data in progress
		uint8_t RestoreParametersFT1346M 	:1;  //  If "1" Restore FT1346 Factory Parameters
        uint8_t RestoreParametersMCP39F511  :1;  //  If "1" Restore Microchip MCP39F511 Factory Parameters
		uint8_t Free 		                :2;
        uint8_t ZCD_Error                   :1;  //  If "1" Zero Cross Detection failed
	};
} GenericFlag;

#ifdef ENABLE_AVERAGE_CALC  
    typedef	union {
        uint8_t	Byte;
    #if AVERAGE_ARRAY_SIZE == 4
        struct
        {
            uint8_t Count  :2;
            uint8_t Free   :6;
        };
    #endif
    #if AVERAGE_ARRAY_SIZE == 8
        struct
        {
            uint8_t Count  :3;
            uint8_t Free   :5;
        };
    #endif
    #if AVERAGE_ARRAY_SIZE == 16
        struct
        {
            uint8_t Count  :4;
            uint8_t Free   :4;
        };
    #endif
    #if AVERAGE_ARRAY_SIZE == 32
        struct
        {
            uint8_t Count  :5;
            uint8_t Free   :3;
        };
    #endif
    } AverageCounter;
#endif
//======================================================================

class Cmd_MCP39F511 {
    public:
        //==========================================
        //  Functions used to manage Eeprom 
        uint8_t MCP39F511_ReadEepromPage(uint8_t EepromPage);
        uint8_t MCP39F511_WriteEepromPage(uint8_t EepromPage);
        uint8_t MCP39F511_BulkEraseEeprom(void);
        //==========================================
        
        //==========================================
        //  Functions used to read data from MCP39F511   
        uint8_t MCP39F511_ReadStatusAndVersion(void);
        uint8_t MCP39F511_ReadSystemConfiguration(void);
        uint8_t MCP39F511_ReadEventStatus(void);
        uint8_t MCP39F511_ReadRmsVoltageRaw(void);
        uint8_t MCP39F511_ReadLineFrequencyRaw(void);
        uint8_t MCP39F511_ReadPowerFactorRaw(void);
        uint8_t MCP39F511_ReadRmsCurrentRaw(void);
        uint8_t MCP39F511_ReadActivePowerRaw(void);
        uint8_t MCP39F511_ReadReactivePowerRaw(void);
        uint8_t MCP39F511_ReadApparentPowerRaw(void);
        uint8_t MCP39F511_ReadImportedActiveEnergy(void);
        uint8_t MCP39F511_ReadExportedActiveEnergy(void);
        uint8_t MCP39F511_ReadImportedReactiveEnergy(void);
        uint8_t MCP39F511_ReadExportedReactiveEnergy(void);
        uint8_t MCP39F511_ReadSarAdc(void);
        uint8_t MCP39F511_ReadMinimumRecord_1(void);
        uint8_t MCP39F511_ReadMinimumRecord_2(void);
        uint8_t MCP39F511_ReadMaximumRecord_1(void);
        uint8_t MCP39F511_ReadMaximumRecord_2(void);
        
        uint8_t MCP39F511_ReadAllRawData(void);
        uint8_t MCP39F511_ReadAllEnergy(void);
        //==========================================

        //==========================================
#ifdef ENABLE_AVERAGE_CALC
        void MCP39F511_UpdateRmsVoltageRawArray(void);
        void MCP39F511_UpdateLineFrequencyRawArray(void);
        void MCP39F511_UpdatePowerFactorRawArray(void);
        void MCP39F511_UpdateRmsCurrentRawArray(void);
        void MCP39F511_UpdateActivePowerRawArray(void);
        void MCP39F511_UpdateReactivePowerRawArray(void);
        void MCP39F511_UpdateApparentPowerRawArray(void);
        
        uint16_t MCP39F511_RmsVoltageRawAverage(void);
        uint16_t MCP39F511_LineFrequencyRawAverage(void);
        uint16_t MCP39F511_PowerFactorRawAverage(void);
        uint32_t MCP39F511_RmsCurrentRawAverage(void);
        uint32_t MCP39F511_ActivePowerRawAverage(void);
        uint32_t MCP39F511_ReactivePowerRawAverage(void);
        uint32_t MCP39F511_ApparentPowerRawAverage(void);
#endif        
        //==========================================
        
        //==========================================
        uint8_t MCP39F511_RetrieveMicrochipFactoryValues(void);
        uint8_t MCP39F511_RetrieveFT1346M_FactoryParameters(void);
        //==========================================
        
        //==========================================
        //  Functions used to write data into MCP39F511
        uint8_t MCP39F511_WriteGainCurrentRms(void);
        uint8_t MCP39F511_WriteGainVoltageRms(void);
        uint8_t MCP39F511_WriteGainActivePower(void);
        uint8_t MCP39F511_WriteGainReactivePower(void);
        uint8_t MCP39F511_WriteOffsetCurrentRms(void);
        uint8_t MCP39F511_WriteOffsetActivePower(void);
        uint8_t MCP39F511_WriteOffsetReactivePower(void);
        uint8_t MCP39F511_WriteDcOffsetCurrent(void);
        uint8_t MCP39F511_WritePhaseCompensation(void);
        uint8_t MCP39F511_WriteApparentPowerDivisor(void);
        uint8_t MCP39F511_WriteSystemConfiguration(void);
        uint8_t MCP39F511_WriteEventConfiguration(void);
        uint8_t MCP39F511_WriteRange(void);
        uint8_t MCP39F511_WriteCalibrationCurrent(void);
        uint8_t MCP39F511_WriteCalibrationVoltage(void);
        uint8_t MCP39F511_WriteCalibrationActivePower(void);
        uint8_t MCP39F511_WriteCalibrationReactivePower(void);
        uint8_t MCP39F511_WriteLineFrequencyReference(void);
        uint8_t MCP39F511_WriteAccumulatorItervalParameter(void);
        uint8_t MCP39F511_WriteVoltageSagLimit(void);
        uint8_t MCP39F511_WriteVoltageSurgeLimit(void);
        uint8_t MCP39F511_WriteOverCurrentLimit(void);
        uint8_t MCP39F511_WriteOverPowerLimit(void);
        uint8_t MCP39F511_WriteTempCompensationFrequency(void);
        uint8_t MCP39F511_WriteTempCompensationCurrent(void);
        uint8_t MCP39F511_WriteTempCompensationPower(void);
        uint8_t MCP39F511_WriteAmbientTempReference(void);
        uint8_t MCP39F511_WritePwmPeriod(void);
        uint8_t MCP39F511_WritePwmDutyCycle(void);
        uint8_t MCP39F511_WriteMinMaxPointer_1(void);
        uint8_t MCP39F511_WriteMinMaxPointer_2(void);
        uint8_t MCP39F511_ResetMinMaxPointer_1(void);
        uint8_t MCP39F511_ResetMinMaxPointer_2(void);
        uint8_t MCP39F511_WriteOverTemperatureLimit(void);
        uint8_t MCP39F511_WriteEnergyControl(void);
        uint8_t MCP39F511_WritePwmControl(void);
        uint8_t MCP39F511_WriteNoLoadThreshold(void);
        uint8_t MCP39F511_FreezeFlashData(void);

        unsigned long CalcPowerFactor(void);
#ifdef ENABLE_AVERAGE_CALC  
        unsigned long CalcAveragePowerFactor(void);
#endif        
        //==========================================
        
        GenericFlag GenericFlags;
        
        //================================================================================
        //  OUTPUT VARIABLES
        SystemStatusFlag  SystemStatus;                 //  (u16) System Status Register
        SystemVersionFlag SystemVersion;                //  (b16) System Version Register
        
        t_SHORT     VoltageRmsRaw;                      //  (u16) Voltage RMS Register Raw Data
        t_SHORT     LineFrequencyRaw;                   //  (u16) Line Frequency Register Raw Data
        t_SHORT     AnalogInputVoltageRaw;              //  (u16) Analog Input Voltage Register Raw Data
        t_SHORT     PowerFactorRaw;                     //  (u16) Power Factor Register Raw Data
        t_LONG      CurrentRmsRaw;                      //  (u32) Current RMS Register Raw Data
        t_LONG      ActivePowerRaw;                     //  (u32) Active Power Register Raw Data
        t_LONG      ReactivePowerRaw;                   //  (u32) Reactive Power Register Raw Data
        t_LONG      ApparentPowerRaw;                   //  (u32) Apparent Power Register Raw Data
                
        t_LONG_LONG ImportActiveEnergyCount;            //  (u64) Import Active Energy Counter Register
        t_LONG_LONG ExportActiveEnergyCount;            //  (u64) Export Active Energy Counter Register
        t_LONG_LONG ImportReactiveEnergyCount;          //  (u64) Import Reactive Energy Counter Register
        t_LONG_LONG ExportReactiveEnergyCount;          //  (u64) Export Reactive Energy Counter Register
        
        t_LONG      MinimumRecord_1;                    //  (u32) Minimum Value of the Output Quantity Address in Min/Max Pointer 1 Register
        t_LONG      MinimumRecord_2;                    //  (u32) Minimum Value of the Output Quantity Address in Min/Max Pointer 2 Register
        t_LONG      MaximumRecord_1;                    //  (u32) Maximum Value of the Output Quantity Address in Min/Max Pointer 1 Register
        t_LONG      MaximumRecord_2;                    //  (u32) Maximum Value of the Output Quantity Address in Min/Max Pointer 2 Register
        //================================================================================
        
        //================================================================================
        //  Variables used to calc average value
#ifdef ENABLE_AVERAGE_CALC          
        AverageCounter V_AveIndex;
        AverageCounter F_AveIndex;
        AverageCounter PF_AveIndex;
        AverageCounter I_AveIndex;
        AverageCounter P_AveIndex;
        AverageCounter S_AveIndex;
        AverageCounter Q_AveIndex;
        
        uint16_t VoltageRmsRaw_Array[AVERAGE_ARRAY_SIZE];       //  (u16) Temp Voltage RMS used to calc average
        uint16_t LineFrequencyRaw_Array[AVERAGE_ARRAY_SIZE];    //  (u32) Temp Line Frequency used to calc average 
        uint16_t PowerFactorRaw_Array[AVERAGE_ARRAY_SIZE];      //  (u32) Temp Power Factory used to calc average
        uint32_t CurrentRmsRaw_Array[AVERAGE_ARRAY_SIZE];       //  (u32) Temp Current RMS used to calc average 
        uint32_t ActivePowerRaw_Array[AVERAGE_ARRAY_SIZE];      //  (u32) Temp Active Power used to calc average 
        uint32_t ReactivePowerRaw_Array[AVERAGE_ARRAY_SIZE];    //  (u32) Temp Reactive Power used to calc average 
        uint32_t ApparentPowerRaw_Array[AVERAGE_ARRAY_SIZE];    //  (u32) Temp Apparent Power used to calc average
#endif        
        //================================================================================

        //================================================================================
        //  CALIBRATION VARIABLE
        t_SHORT     GainCurrentRms;                     //  (u16) Gain Current RMS
        t_SHORT     GainVoltageRms;                     //  (u16) Gain Voltage RMS
        t_SHORT     GainActivePower;                    //  (u16) Gain Active Power
        t_SHORT     GainReactivePower;                  //  (u16) Gain Reactive Power
        t_LONG      OffsetCurrentRms;                   //  (s32) Offset Current RMS 
        t_LONG      OffsetActivePower;                  //  (s32) Offset Active Power
        t_LONG      OffsetReactivePower;                //  (s32) Offset Reactive Power
        t_SHORT     DcOffsetCurrent;                    //  (s16) DC Offset Current
        t_SHORT     PhaseCompensation;                  //  (s16) Phase Compensation
        t_SHORT     ApparentPowerDivisor;               //  (u16) Apparent Power Divisior
        //================================================================================
        
        //================================================================================
        //  DESIGN CONFIGURATION VARIABLES
        SystemConfigurationFlag SystemConfiguration;    //  (b32)
        EventConfigurationFlag  EventConfiguration;     //  (b32)
        
        t_LONG      Range;                              //  (b32) Selecting factor for Outputs
        t_LONG      CalibrationCurrent;                 //  (u32) Calibration Current
        t_SHORT     CalibrationVoltage;                 //  (u16) Calibration Voltage
        t_LONG      CalibrationActivePower;             //  (u32) Calibration Power Active
        t_LONG      CalibrationReactivePower;           //  (u32) Calibration Power Reactive
        t_SHORT     LineFrequencyReference;             //  (u16) Line Frequency Reference
        t_SHORT     AccumulatorIntervallParameter;      //  (u16) Accumulation Interval Parameter
        t_SHORT     VoltageSagLimit;                    //  (u16) Voltage SAG Limit 
        t_SHORT     VoltageSurgeLimit;                  //  (u16) Voltage Surge Limit
        t_LONG      OverCurrentLimit;                   //  (u32) OverCurrent Limit
        t_LONG      OverPowerLimit;                     //  (u32) OverPower Limit
        t_SHORT     TempCompensationFrequency;          //  (u16) Temperature Compensation for Frequency
        t_SHORT     TempCompensationCurrent;            //  (u16) Temperature Compensation for Current
        t_SHORT     TempCompensationPower;              //  (u16) Temperature Compensation for Power
        t_SHORT     AmbientTempReference;               //  (u16) Ambiente Temperature Reference Voltage
        t_SHORT     PwmPeriod;                          //  (u16) PWM Period
        t_SHORT     PwmDutyCycle;                       //  (u16) PWM Duty Cycle
        t_SHORT     MinMaxPointer_1;                    //  (u16) Min/Max Pointer 1
        t_SHORT     MinMaxPointer_2;                    //  (u16) Min/Max Pointer 2
        t_SHORT     OverTemperatureLimit;               //  (u16) Overtemperature Limit
        t_SHORT     EnergyControl;                      //  (u16) Energy Control
        t_SHORT     PwmControl;                         //  (u16) PWM Control
        t_SHORT     NoLoadThreshold;                    //  (u16) No Load Threshold
        //================================================================================
           
        //================================================================================
        //  EEPROM VARIABLES
        uint8_t     EepromPageArray[16];
        //================================================================================
        
    private:
        void MCP39F511_FreezeFlash(void);
        void MCP39F511_ReadData(uint16_t StartAdd, uint8_t Data);
        void MCP39F511_WriteData_Byte(uint16_t StartAdd, uint8_t Data);
        void MCP39F511_WriteData_Word(uint16_t StartAdd, uint16_t Data);
        void MCP39F511_WriteData_DoubleWord(uint16_t StartAdd, uint32_t Data);    
        
        void MCP39F511_ReadEeprom(uint8_t EepromPage);
        void MCP39F511_WriteEeprom(uint8_t EepromPage);
        void MCP39F511_EraseEeprom(void);
        
        uint8_t CalcChecksum(uint8_t nByte);
        uint8_t MCP39F511_DecodeDataReceived(uint8_t nByteReceived, uint8_t TypeOfCmd);
        void    LoadEepromArray(void);
};
extern Cmd_MCP39F511 Cmd_39F511;

#endif