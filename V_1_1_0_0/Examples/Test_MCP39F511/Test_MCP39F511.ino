/*********************************************************************
 *
 *       Sketch MCP39F511 (How to use the library)
 *
 *********************************************************************
 * FileName:        Test_MCP79511.ino
 *                  DigitalInput.ino
 *                  DigitalOutput.ino
 *                  TimersInt.ino
 *                  
 * Revision:        1.0.0
 * Date:            05/05/2019
 * Dependencies:    Cmd_MCP39F511.h
 *                  Io_MCP39F511.h
 *                  Isr_MCP39F511.h
 *                  Uart_MCP39F511.h
 *
 * Revision:        1.1.0
 * Date:            27/12/2019
 *                  - Added code to use new library functions used to calcl average value of the electrical measures
 * 
 * Dependencies:    Cmd_MCP39F511.h
 *                  Io_MCP39F511.h
 *                  Isr_MCP39F511.h
 *                  Uart_MCP39F511.h
 *
 * Arduino Board:   Arduino Mega 2560 or Fishino Mega 2560
 *                  See file Io_MCP39F511.h to define hardware and Arduino Board to use
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
 * ====================================================================
 * NOTES
 * ====================================================================
 * The code for this demo is splitted into six files:
 *
 *  - Test_MCP39F511   -> Main file project
 *  - DigitalInput     -> Contains the code to manage the digital inputs and the respective functions to manage them
 *  - DigitalOutput    -> Contains the code to manage the digital outputs and the respective functions to manage them
 *  - Measures         -> Contains the code to send commands to the MCP39F511 and manage the answers (Decode data and prints them on the IDE Serial Monitor)
 *  - TimerInt         -> Contains the code to manage Timer5 of ATMEGA2560. Timer5 generates an interrupt every 2 mSec.
 *                        With this resolution (2 mSec) it is possible to manage the timer variables used in this demo.
 *                        For example the TimeOut used to filter the unwanted variations on the digital inputs (debouncing)
 *  - Uart_to_Uart     -> Contains the code to manage the communication between EnergyMeter Software R.1.0.4.0 (Driver R.1.0.2.0) and the MCP39F511device
 *
 **********************************************************************/

#include <avr/eeprom.h>

#include "Cmd_MCP39F511.h"
#include "Io_MCP39F511.h"
#include "Isr_MCP39F511.h"
#include "Uart_MCP39F511.h"

Cmd_MCP39F511   Cmd_39F511;

#define TRUE  0
#define FALSE 1

#define SKETCH_MCP39F511_REV                 "1.1"

//#define ONLY_SOFTWARE_ENERGY_METER       //  Uncomment this line to enable uart-to-uart function only (Software EnergyMeter R.1.0.4.0 or higher)

//======================================================================
//  MCP39F511  HARDWARE DEFINE (INPUT AND OUTPUT)

//--------------------------------------------
//  Directives used to manage Output
#define PIN_RELAY_1               22      //  Relay 1      mapped on the I/O 22 (Pin 75)
#define PIN_RELAY_2               23      //  Relay 2      mapped on the I/O 23 (Pin 76)
#define PIN_BUZZER                44      //  Buzzer       mapped on the I/O 44 (Pin 53)
#define PIN_LED_1                 24      //  Red Led 1    mapped on the I/O 24 (Pin 73)
#define PIN_LED_2                 25      //  Led Yellow 2 mapped on the I/O 25 (Pin 74)
#define PIN_LED_3                 26      //  Led Green 3  mapped on the I/O 26 (Pin 71)
#define PIN_LED_4                 27      //  Led Green 4  mapped on the I/O 27 (Pin 72)

#define OUTPUT_STATE_OFF           0
#define OUTPUT_STATE_ON            1
#define OUTPUT_STATE_TOGGLE        2
#define OUTPUT_STATE_TOGGLE_WAIT   3
#define OUTPUT_STATE_TOGGLE_ONLY   4
#define OUTPUT_STATE_NOTHING       255

#define SELECT_RELAY_1             0
#define SELECT_RELAY_2             1
#define SELECT_BUZZER              2
#define SELECT_LED_1               3
#define SELECT_LED_2               4
#define SELECT_LED_3               5
#define SELECT_LED_4               6

#define OUTPUT_OFF                 0
#define OUTPUT_ON                  1

#define POWER_THR_OFFSET           100    //  Power Threshold offset. This parameter is used to manage leds visualization. If power measure is less then
                                          //  threshold minus offset the led green is on. Then if the power measure is more then threshold 
                                          //  minus offset the led yellow is on. Finally if the power measure is more then threshold the led red is on and buzzer is active
#define POWER_THR_DEFAULT          500    //  Power Threshold 500W
//--------------------------------------------

//--------------------------------------------
#define IN_P1                     42    //  P1 Input mapped on the I/O 42 (Pin 55)
#define IN_P2                     43    //  P2 Input mapped on the I/O 43 (Pin 56)

#define LEVEL_HIGH_INPUT          0
#define LEVEL_LOW_INPUT           1
#define TOGGLE_INPUT              2

#define INPUT_STATE_IDLE          0          
#define INPUT_STATE_WAIT          1
//--------------------------------------------
//======================================================================

//======================================================================
//  CONST STRING
const char STR_SKETCH_REV[]               PROGMEM = "MCP39F511 Sketch Rev: ";
const char STR_REV[]                      PROGMEM = "MCP39F511 Device Rev: ";
const char STR_NEGATIVE_ACTIVE_POWER[]    PROGMEM = "Active power is negative (export) and is in quadrants 2,3\n";
const char STR_POSITIVE_ACTIVE_POWER[]    PROGMEM = "Active power is positive (import) and is in quadrants 1,4\n";
const char STR_NEGATIVE_REACTIVE_POWER[]  PROGMEM = "Reactive power is negative (capacitive load) and is in quadrants 3,4\n";
const char STR_POSITIVE_REACTIVE_POWER[]  PROGMEM = "Reactive power is positive and is in quadrants 1,2\n";
const char STR_ZCD[]                      PROGMEM = "Zero Cross Detected!\n";
const char STR_ZCD_ERROR[]                PROGMEM = "Zero Cross Failure!\n";
const char STR_VOLTAGE[]                  PROGMEM = "Instant Value Vrms: ";
const char STR_CURRENT[]                  PROGMEM = "Instant Value Irms: ";
const char STR_FREQUENCY[]                PROGMEM = "Instant Value Freq: ";
const char STR_POWER_FACTOR[]             PROGMEM = "Instant Value PF:   ";
const char STR_APPARENT_POWER[]           PROGMEM = "Instant Value S:    ";
const char STR_ACTIVE_POWER[]             PROGMEM = "Instant Value P:    ";
const char STR_REACTIVE_POWER[]           PROGMEM = "Instant Value Q:    ";

#ifdef ENABLE_AVERAGE_CALC 
  const char STR_AVERAGE_VOLTAGE[]          PROGMEM = "Average Value Vrms: ";
  const char STR_AVERAGE_CURRENT[]          PROGMEM = "Average Value Irms: ";
  const char STR_AVERAGE_FREQUENCY[]        PROGMEM = "Average Value Freq: ";
  const char STR_AVERAGE_POWER_FACTOR[]     PROGMEM = "Average Value PF:   ";
  const char STR_AVERAGE_APPARENT_POWER[]   PROGMEM = "Average Value S:    ";
  const char STR_AVERAGE_ACTIVE_POWER[]     PROGMEM = "Average Value P:    ";
  const char STR_AVERAGE_REACTIVE_POWER[]   PROGMEM = "Average Value Q:    ";
#endif
const char STR_THRESHOLD[]                PROGMEM = "Power Threshold selected: ";

const char STR_FT1346M_FACTORY_PAR[]      PROGMEM = "Restore FT1346M Factory Parameters\n";
const char STR_MCP39F511_FACTORY_PAR[]    PROGMEM = "Restore MCP39F511 Microchip Factory Parameters\n";
//======================================================================

//======================================================================
typedef void State;
typedef State (*Pstate)();

Pstate UartToUart;              //  State Machine used to manage the data trasmission from PC uart to MCP39F511 uart
Pstate ReadDataMCP39F511;       //  State Machine used to read data from MCP39F511
Pstate InputP1_Management;      //  State Machine used to manage P1 Button
Pstate InputP2_Management;      //  State Machine used to manage P2 Button
Pstate ActivePower_Management;  //  State Machine used to check if active power exceeds the active power Threshold
//======================================================================

//======================================================================
//  Variables
uint16_t PowerThreshold;

union Flags {
  struct {
    uint8_t  PowerThreshold     :3;  // 000 (0) -> 1kW Threshold
                                     // 001 (1) -> 2kW Threshold
                                     // 010 (2) -> 3kW Threshold
                                     // 011 (3) -> 4kW Threshold
                                     // 100 (4) -> 5kW Threshold
                                     // 101 (5) -> 6kW Threshold
    uint8_t  PcUartRequest;     :1;  // If "1" communication request from PC UART
    uint8_t  Free               :3;
    uint8_t  SettingInProgress  :1;
  } Bit;
} Flags;
//======================================================================

//======================================================================
//  Timer 5 variables
uint16_t  DebouncingInput[2];
uint16_t  TimeOutInputPin[2];

uint16_t  TimeOutCheckActivePower;
uint16_t  TimeOutPrintSignActivePower;
uint16_t  TimeOutPrintSignReactivePower;
uint16_t  TimeOutBeepAlarm;
uint16_t  TimeOutReadData;
uint16_t  TimeOutPrintData;
//======================================================================

//======================================================================
//
uint8_t OutputPin[7];
uint8_t LastStateOutputPin[7];
uint8_t InputPin[2];

union DigitalInput {
  struct {
    uint8_t  InputStatus     :1;
    uint8_t  InputReaded     :1;
    uint8_t  InputVar        :1;
    uint8_t  LastInputStatus :1;
    uint8_t  Free            :4;
  } Bit;
} DigitalInput[2];
//======================================================================

//======================================================================
//  MCP39F511 Sketch
void setup() {
  SetupTimer5();
  SetInputPin();
  SetOutputPin();
  PowerOnBuzzerLedTest();
  
  Uart_39F511.EnableDisableIdeMonitor(true, BAUD_115200);    // Enables and configure UART for debug 
  Uart_39F511.SetBaudRateUart(true, BAUD_115200);            // Enables and configure MCP39F511 UART 
  Isr_39F511.EnableLibInterrupt();
  Io_39F511.MCP39F511_SetControlLine();
  Io_39F511.MCP39F511_MclrLine(1);
  delay(500);
  Io_39F511.MCP39F511_ResetLine(1);
  
  //----------------------------------------------- 
  //  Sets State Machine
  UartToUart             = WaitDataFromPc;
  ReadDataMCP39F511      = ReadDataIdle;
  InputP1_Management     = InputP1_Idle;
  InputP2_Management     = InputP2_Idle;
  ActivePower_Management = TestActivePowerIdle;
  //----------------------------------------------- 

  //----------------------------------------------- 
  PowerThreshold   = POWER_THR_DEFAULT;        //  Power Threshold 500 W
  
  TimeOutReadData  = T_2SEC;
  TimeOutPrintData = T_10SEC;

  DigitalInput[0].Bit.InputStatus = 1;
  DigitalInput[1].Bit.InputStatus = 1;  
  //----------------------------------------------- 

  //-----------------------------------------------
  //  Prints sketch revision
  ReadStringCmd_FLASH((uint8_t *)STR_SKETCH_REV, strlen(STR_SKETCH_REV), FALSE, FALSE);
  Serial.println(SKETCH_MCP39F511_REV);
  Serial.println("");
  //----------------------------------------------- 
}
//======================================================================

#ifdef ONLY_SOFTWARE_ENERGY_METER
  //======================================================================
  //  Main Loop MCP39F511 Uart-To-Uart only
  void loop() {
    UartToUart();                   //  State machine used to manage serial monitor commands or EnergyMeter software communication
  }
  //======================================================================
#else
  //======================================================================
  //  Main Loop MCP39F511
  void loop() {
    DebouncingInputPin();           //  Reads and debounce pulse P1 and P2
    InputP1_Management();           //  State machine used to manage P1 button
    InputP2_Management();           //  State machine used to manage P2 button
    ReadDataMCP39F511();            //  State machine used to read measures from MCP39F511
    if (Cmd_39F511.GenericFlags.ReadDataInProgress == 0) {
      UartToUart();                 //  State machine used to manage serial monitor commands or EnergyMeter software communication  
    }
        
    if (Flags.Bit.SettingInProgress == 0) {
      ActivePower_Management();       //  State machine used to check active power read  
    }
    if (Flags.Bit.PcUartRequest != 1) {
      if ((TimeOutReadData == 0) && (Cmd_39F511.GenericFlags.ReadDataInProgress == 0)) {
        TimeOutReadData = T_2SEC;
        ReadDataMCP39F511 = ReadStatus;   //  Set state machine to start to read data from MCP39F511
      }
      
      if ((TimeOutPrintData == 0) && (Cmd_39F511.GenericFlags.ReadDataInProgress == 0)) {
        TimeOutPrintData = T_5SEC; // T_10SEC;
        PrintDataOnSerialMonitor();
      }
    
      if ((Cmd_39F511.GenericFlags.RestoreParametersFT1346M == 1) && (Cmd_39F511.GenericFlags.ReadDataInProgress == 0)) {
        OnOff_Output(SELECT_LED_3, OUTPUT_ON);
        Cmd_39F511.GenericFlags.ReadDataInProgress = 1;
        Cmd_39F511.MCP39F511_RetrieveFT1346M_FactoryParameters();
        Cmd_39F511.MCP39F511_FreezeFlashData();
        Cmd_39F511.GenericFlags.ReadDataInProgress = 0;
        Cmd_39F511.GenericFlags.RestoreParametersFT1346M = 0;
        OnOff_Output(SELECT_LED_3, OUTPUT_OFF);
      }
      
      if ((Cmd_39F511.GenericFlags.RestoreParametersMCP39F511 == 1) && (Cmd_39F511.GenericFlags.ReadDataInProgress == 0)) {
        OnOff_Output(SELECT_LED_4, OUTPUT_ON);
        Cmd_39F511.GenericFlags.ReadDataInProgress = 1;
        Cmd_39F511.MCP39F511_RetrieveMicrochipFactoryValues();
        Cmd_39F511.MCP39F511_FreezeFlashData();
        Cmd_39F511.GenericFlags.ReadDataInProgress = 0;
        Cmd_39F511.GenericFlags.RestoreParametersMCP39F511 = 0;
        OnOff_Output(SELECT_LED_4, OUTPUT_OFF);
      }    
    }
  }
  //======================================================================
#endif
