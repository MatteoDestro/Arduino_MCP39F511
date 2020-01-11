//======================================================================
//  Sets digital inputs
void SetInputPin(void) {
  InputPin[0] = IN_P1;
  InputPin[1] = IN_P2;
  
  pinMode(InputPin[0], INPUT);          // set pin to input
  pinMode(InputPin[1], INPUT);          // set pin to input

  digitalWrite(InputPin[0], HIGH);      // turn on pullup resistors
  digitalWrite(InputPin[1], HIGH);      // turn on pullup resistors
}
//======================================================================

//======================================================================
void DebouncingInputPin(void) {
  uint8_t n = 0;

  do {
    DigitalInput[n].Bit.InputReaded = digitalRead(InputPin[n]);
    
    if (DigitalInput[n].Bit.InputReaded != DigitalInput[n].Bit.InputStatus) {    
      if (DigitalInput[n].Bit.InputReaded != DigitalInput[n].Bit.InputVar) {
        DigitalInput[n].Bit.InputVar = DigitalInput[n].Bit.InputReaded;
        DebouncingInput[n] = T_50MSEC;
      } else {
        if (DebouncingInput[n] == 0) {
          DigitalInput[n].Bit.InputVar    = DigitalInput[n].Bit.InputReaded;
          DigitalInput[n].Bit.InputStatus = DigitalInput[n].Bit.InputReaded;
        }
      }
    } else {
        DigitalInput[n].Bit.InputVar = DigitalInput[n].Bit.InputStatus;
    }    
  } while (++n < 2);
}
//======================================================================

//======================================================================
//  P1 Button function - Short pression
//
//  Toggle relay 1 output
//
//  ------------------------------------------------------------------------------------------
//  P1 Button function - Long pression
//
//  If P1 pressed for 1 second sets 1kW threshold - Led 1 ON  - Led 2 OFF - Led3 OFF - Buzzer 1 Beep to confirm 1kW setting
//  If P1 pressed for 2 second sets 2kW threshold - Led 1 OFF - Led 2 ON  - Led3 OFF - Buzzer 2 Beep to confirm 2kW setting
//  If P1 pressed for 3 second sets 3kW threshold - Led 1 OFF - Led 2 OFF - Led3 ON  - Buzzer 3 Beep to confirm 3kW setting
//  If P1 pressed for 4 second sets 4kW threshold - Led 1 ON  - Led 2 ON  - Led3 OFF - Buzzer 4 Beep to confirm 4kW setting
//  If P1 pressed for 5 second sets 5kW threshold - Led 1 ON  - Led 2 OFF - Led3 ON  - Buzzer 5 Beep to confirm 5kW setting
//  If P1 pressed for 6 second sets 6kW threshold - Led 1 OFF - Led 2 ON  - Led3 ON  - Buzzer 6 Beep to confirm 6kW setting
//  ------------------------------------------------------------------------------------------
//
//  P2 Button function - Short pression
//
//  Toggle relay 2 output
//
//  ------------------------------------------------------------------------------------------
//  P2 Button function - Long pression
//
//  If P2 pressed for 3 second FT1346M factory parameters restored             - Led 1 ON  - Led 2 OFF - Led3 OFF
//  If P2 pressed for 6 second MCP39F511 Microchip factory parameters restored - Led 1 OFF - Led 2 ON  - Led3 OFF
//  ------------------------------------------------------------------------------------------
//
//======================================================================

//======================================================================
void InputP1_Idle(void) {
  if (DigitalInput[0].Bit.InputStatus != 1) {
    TimeOutInputPin[0] = T_7SEC;
    InputP1_Management =  InputP1_Wait;   
    Flags.Bit.SettingInProgress = 1;
    return;
  }
}

void InputP1_Wait(void) {
  if ((TimeOutInputPin[0] > T_6SEC) && (DigitalInput[0].Bit.InputStatus != 0)) {
    //  Button P1 pressed for a time lower than 1 second and subsequently released
    //  Toggle Digital Output Relay 1
    InputP1_Management = InputP1_Released;    //  Button released
    Toggle_Output(SELECT_RELAY_1);
  } else if ((TimeOutInputPin[0] > T_5SEC) && (TimeOutInputPin[0] < T_6SEC) && (DigitalInput[0].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_ON);
     OnOff_Output(SELECT_LED_2, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_3, OUTPUT_OFF);
  } else if ((TimeOutInputPin[0] > T_4SEC) && (TimeOutInputPin[0] < T_5SEC) && (DigitalInput[0].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_2, OUTPUT_ON);
     OnOff_Output(SELECT_LED_3, OUTPUT_OFF);
  } else if ((TimeOutInputPin[0] > T_3SEC) && (TimeOutInputPin[0] < T_4SEC) && (DigitalInput[0].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_2, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_3, OUTPUT_ON);
  } else if ((TimeOutInputPin[0] > T_2SEC) && (TimeOutInputPin[0] < T_3SEC) && (DigitalInput[0].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_ON);
     OnOff_Output(SELECT_LED_2, OUTPUT_ON);
     OnOff_Output(SELECT_LED_3, OUTPUT_OFF);
  } else if ((TimeOutInputPin[0] > T_1SEC) && (TimeOutInputPin[0] < T_2SEC) && (DigitalInput[0].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_ON);
     OnOff_Output(SELECT_LED_2, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_3, OUTPUT_ON);    
  } else if ((TimeOutInputPin[0] >= 0) && (TimeOutInputPin[0] < T_1SEC) && (DigitalInput[0].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_2, OUTPUT_ON);
     OnOff_Output(SELECT_LED_3, OUTPUT_ON);
  } else if ((TimeOutInputPin[0] > T_5SEC) && (DigitalInput[0].Bit.InputStatus != 0)) {
    //  Button P1 pressed for a time lower than 2 second and subsequently 
    //  Set 1kW threshold
    InputP1_Management = InputP1_Released;    //  Button released
    PowerThreshold = 1000;
    ReadStringCmd_FLASH((uint8_t *)STR_THRESHOLD, strlen(STR_THRESHOLD), FALSE, FALSE);
    Serial.println(PowerThreshold);
    ThresholdBeep(70, 1);
  } else if ((TimeOutInputPin[0] > T_4SEC) && (DigitalInput[0].Bit.InputStatus != 0)) {
    //  Button P1 pressed for a time lower than 3 second and subsequently 
    //  Set 2kW threshold
    InputP1_Management = InputP1_Released;    //  Button released
    PowerThreshold = 2000;
    ReadStringCmd_FLASH((uint8_t *)STR_THRESHOLD, strlen(STR_THRESHOLD), FALSE, FALSE);
    Serial.println(PowerThreshold);
    ThresholdBeep(70, 2);    
  } else if ((TimeOutInputPin[0] > T_3SEC) && (DigitalInput[0].Bit.InputStatus != 0)) {
    //  Button P1 pressed for a time lower than 4 second and subsequently 
    //  Set 3kW threshold
    InputP1_Management = InputP1_Released;    //  Button released
    PowerThreshold = 3000;
    ReadStringCmd_FLASH((uint8_t *)STR_THRESHOLD, strlen(STR_THRESHOLD), FALSE, FALSE);
    Serial.println(PowerThreshold);    
    ThresholdBeep(70, 3);
  } else if ((TimeOutInputPin[0] > T_2SEC) && (DigitalInput[0].Bit.InputStatus != 0)) {
    //  Button P1 pressed for a time lower than 5 second and subsequently 
    //  Set 4kW threshold
    InputP1_Management = InputP1_Released;    //  Button released
    PowerThreshold = 4000;
    ReadStringCmd_FLASH((uint8_t *)STR_THRESHOLD, strlen(STR_THRESHOLD), FALSE, FALSE);
    Serial.println(PowerThreshold);    
    ThresholdBeep(70, 4);
  } else if ((TimeOutInputPin[0] > T_1SEC) && (DigitalInput[0].Bit.InputStatus != 0)) {
    //  Button P1 pressed for a time lower than 6 second and subsequently 
    //  Set 5kW threshold
    InputP1_Management = InputP1_Released;    //  Button released
    PowerThreshold = 5000;
    ReadStringCmd_FLASH((uint8_t *)STR_THRESHOLD, strlen(STR_THRESHOLD), FALSE, FALSE);
    Serial.println(PowerThreshold);        
    ThresholdBeep(70, 5);
  } else if ((TimeOutInputPin[0] >= 0) && (DigitalInput[0].Bit.InputStatus != 0)) {
    //  Button P1 pressed for a time lower than 7 second and subsequently 
    //  Set 6kW threshold
    InputP1_Management = InputP1_Released;    //  Button released
    PowerThreshold = 6000;
    ReadStringCmd_FLASH((uint8_t *)STR_THRESHOLD, strlen(STR_THRESHOLD), FALSE, FALSE);
    Serial.println(PowerThreshold);    
    ThresholdBeep(70, 6);    
  }
}

void InputP1_Released(void) {
  if (DigitalInput[0].Bit.InputStatus != 0) {
    OnOff_Output(SELECT_LED_1, OUTPUT_OFF);
    OnOff_Output(SELECT_LED_2, OUTPUT_OFF);
    OnOff_Output(SELECT_LED_3, OUTPUT_OFF);
    Flags.Bit.SettingInProgress = 0;
    InputP1_Management = InputP1_Idle;
  }
}
//======================================================================

//======================================================================
void InputP2_Idle(void) {
  if (DigitalInput[1].Bit.InputStatus != 1) {
    TimeOutInputPin[1] = T_7SEC;
    InputP2_Management =  InputP2_Wait;
    Flags.Bit.SettingInProgress = 1;   
    return;
  }
}

void InputP2_Wait(void) {
  if ((TimeOutInputPin[1] > T_6SEC) && (DigitalInput[1].Bit.InputStatus != 0)) {
    //  Button P2 pressed for a time lower than 1 second and subsequently released
    //  Toggle Digital Output Relay 2
    InputP2_Management = InputP2_Released;    //  Button released
    Toggle_Output(SELECT_RELAY_2);
  } else if ((TimeOutInputPin[1] > T_3SEC) && (TimeOutInputPin[1] < T_4SEC) && (DigitalInput[1].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_ON);
     OnOff_Output(SELECT_LED_2, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_3, OUTPUT_OFF);
  } else if ((TimeOutInputPin[1] > 0) && (TimeOutInputPin[1] < T_3SEC) && (DigitalInput[1].Bit.InputStatus != 1)) {
     OnOff_Output(SELECT_LED_1, OUTPUT_OFF);
     OnOff_Output(SELECT_LED_2, OUTPUT_ON);
     OnOff_Output(SELECT_LED_3, OUTPUT_OFF);    
  } else if ((TimeOutInputPin[1] > T_3SEC) && (DigitalInput[1].Bit.InputStatus != 0)) {
    //  Button P2 pressed for a time lower than 4 second and subsequently released
    //  Restore FT1346M Factory Parameters
    ReadStringCmd_FLASH((uint8_t *)STR_FT1346M_FACTORY_PAR, strlen(STR_FT1346M_FACTORY_PAR), FALSE, FALSE);
    Cmd_39F511.GenericFlags.RestoreParametersFT1346M = 1;
    InputP2_Management = InputP2_Released;
  } else if ((TimeOutInputPin[1] == 0) && (DigitalInput[1].Bit.InputStatus != 0)) {
    //  Button P2 pressed for a time major than 6 second and subsequently released
    //  Restore MCP39F511 Factory Parameters
    ReadStringCmd_FLASH((uint8_t *)STR_MCP39F511_FACTORY_PAR, strlen(STR_MCP39F511_FACTORY_PAR), FALSE, FALSE);
    Cmd_39F511.GenericFlags.RestoreParametersMCP39F511 = 1;
    InputP2_Management = InputP2_Released;
  }
}

void InputP2_Released(void) {
  if (DigitalInput[1].Bit.InputStatus != 0) {
    OnOff_Output(SELECT_LED_1, OUTPUT_OFF);
    OnOff_Output(SELECT_LED_2, OUTPUT_OFF);
    OnOff_Output(SELECT_LED_3, OUTPUT_OFF);
    Flags.Bit.SettingInProgress = 0;
    InputP2_Management = InputP2_Idle;
  }
}
//======================================================================
