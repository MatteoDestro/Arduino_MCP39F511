//======================================================================
//  Sets digital Output
void SetOutputPin(void) {
  uint8_t Count = 0;
  OutputPin[0] = PIN_RELAY_1;
  OutputPin[1] = PIN_RELAY_2;
  OutputPin[2] = PIN_BUZZER;
  OutputPin[3] = PIN_LED_1;
  OutputPin[4] = PIN_LED_2;
  OutputPin[5] = PIN_LED_3;  
  OutputPin[6] = PIN_LED_4;

  do {
    pinMode(OutputPin[Count], OUTPUT);  
    digitalWrite(OutputPin[Count], LOW);
  } while (++Count < sizeof(OutputPin));
}
//======================================================================

//======================================================================
//  Function used to test Buzzer and leds
void PowerOnBuzzerLedTest(void) {
  uint8_t Count = 2;
  do {
    OnOff_Output(Count, OUTPUT_ON);
    OnOff_Output(SELECT_BUZZER, OUTPUT_ON);
    delay(100); 
    OnOff_Output(SELECT_BUZZER, OUTPUT_OFF);
    delay(100); 
  } while (++Count < sizeof(OutputPin));
  Count = 2;
  do {
    OnOff_Output(Count, OUTPUT_OFF);
    OnOff_Output(SELECT_BUZZER, OUTPUT_ON);
    delay(100); 
    OnOff_Output(SELECT_BUZZER, OUTPUT_OFF);
    delay(100); 
  } while (++Count < sizeof(OutputPin));  
}
//======================================================================

//======================================================================
//  Function used to SET or RESET digital output
//  If "OnOff" = 0 -> Output OFF
//  If "OnOff" = 1 -> Output ON
//  ----------------------------------------------
//  If "SelectOutput" = 0 -> Relay 1
//  If "SelectOutput" = 1 -> Relay 2
//  If "SelectOutput" = 2 -> Buzzer
//  If "SelectOutput" = 3 -> Led 1
//  If "SelectOutput" = 4 -> Led 2
//  If "SelectOutput" = 5 -> Led 3 
//  If "SelectOutput" = 6 -> Led 4
void OnOff_Output(uint8_t SelectOutput, uint8_t OnOff) {
  if (OnOff != 0) {
    digitalWrite(OutputPin[SelectOutput], HIGH);
    LastStateOutputPin[SelectOutput] = OUTPUT_STATE_ON;
  } else {
    digitalWrite(OutputPin[SelectOutput], LOW);
    LastStateOutputPin[SelectOutput] = OUTPUT_STATE_OFF;
  }
}
//======================================================================

//======================================================================
//  Function used to TOGGLE digital output
//  If "OnOff" = 0 -> Output OFF
//  If "OnOff" = 1 -> Output ON
//  ----------------------------------------------
//  If "SelectOutput" = 0 -> Relay 1
//  If "SelectOutput" = 1 -> Relay 2
//  If "SelectOutput" = 2 -> Buzzer
//  If "SelectOutput" = 3 -> Led 1
//  If "SelectOutput" = 4 -> Led 2
//  If "SelectOutput" = 5 -> Led 3 
//  If "SelectOutput" = 6 -> Led 4
void Toggle_Output(uint8_t SelectOutput) {
  if (LastStateOutputPin[SelectOutput] == OUTPUT_STATE_ON) {
    digitalWrite(OutputPin[SelectOutput], LOW);
    LastStateOutputPin[SelectOutput] = OUTPUT_STATE_OFF;
  } else if (LastStateOutputPin[SelectOutput] == OUTPUT_STATE_OFF) {
    digitalWrite(OutputPin[SelectOutput], HIGH); 
    LastStateOutputPin[SelectOutput] = OUTPUT_STATE_ON;
  }
}
//======================================================================

//======================================================================
void ThresholdBeep(uint8_t TimeOnOff, uint8_t BeepNum) {
  do {
    OnOff_Output(SELECT_BUZZER, OUTPUT_ON);
    delay(TimeOnOff); 
    OnOff_Output(SELECT_BUZZER, OUTPUT_OFF);
    delay(TimeOnOff); 
  } while (--BeepNum > 0);  
}
//======================================================================
