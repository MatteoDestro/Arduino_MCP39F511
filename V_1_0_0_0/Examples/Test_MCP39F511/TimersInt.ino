//======================================================================
#define Fosc              (unsigned long)16000000    //  Clock 16MHz
#define PRET              (unsigned long)256         //  Prescaler 256
#define MSEC              2                          //  Time Base: 2mSec
#define SLOWBASETIME_T5   (0xFFFF - ((Fosc * MSEC) / (PRET * 1000)))
//======================================================================

//======================================================================
void SetupTimer5(void) {   
  // initialize timer5
  cli();                // disable all interrupts
  TCCR5A = 0x00;
  TCCR5B = 0x00;
  TCCR5C = 0x00;
  TCNT5  = SLOWBASETIME_T5;
  TCCR5B = 0x04;
  TIMSK5 = 0x01;        // enable oveflow timer interrupt
  sei();                // enable all interrupts
}
//======================================================================

//======================================================================
ISR(TIMER5_OVF_vect) {
  TCNT5 = SLOWBASETIME_T5;

  if (DebouncingInput[0]      > 0) { DebouncingInput[0]--;      }
  if (DebouncingInput[1]      > 0) { DebouncingInput[1]--;      }
  if (TimeOutInputPin[0]      > 0) { TimeOutInputPin[0]--;      }
  if (TimeOutInputPin[1]      > 0) { TimeOutInputPin[1]--;      }

  if (TimeOutCheckActivePower       > 0) { TimeOutCheckActivePower--;       }
  if (TimeOutBeepAlarm              > 0) { TimeOutBeepAlarm--;              }
  if (TimeOutReadData               > 0) { TimeOutReadData--;               }
  if (TimeOutPrintData              > 0) { TimeOutPrintData--;              }
  if (TimeOutPrintSignActivePower   > 0) { TimeOutPrintSignActivePower--;   }
  if (TimeOutPrintSignReactivePower > 0) { TimeOutPrintSignReactivePower--; }
}
//======================================================================
