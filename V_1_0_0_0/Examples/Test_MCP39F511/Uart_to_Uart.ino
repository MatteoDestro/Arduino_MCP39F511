//======================================================================
//  Wait data from PC
void WaitDataFromPc(void) {
  String StrSerialInput = "";
  uint8_t Count;

  if (Cmd.GenericFlags.ReadDataInProgress != 0) {
    return;
  }
  if (Serial.available() > 0) {
    Serial.readBytes(Uart.Uart_Array, sizeof(Uart.Uart_Array));
  }
  if (Uart.Uart_Array[0] == HEADER_BYTE) {
    //  Received data by energyMeter Software
    Flags.Bit.PcUartRequest = 1;
    UartToUart = ProcessDataFromPc;  // Next Step  
  } else {
    //  Received generic string command
    //  TO DO TO DO
  }
}
//======================================================================

//======================================================================
//  Send data received from PC (EnergyMeter Software R.1.0.4.0) to the MCP39F511
void ProcessDataFromPc(void) {
  Uart.UartSendData(Uart.Uart_Array[1]);
  Uart.UartReceivedData();
  UartToUart = SendDataToPc;  // Next Step  
}
//======================================================================

//======================================================================
//  Send data received from MCP39F511 to the PC (EnergyMeter Software R.1.0.4.0)
void SendDataToPc(void) {
  uint8_t Count = 0;

  do {
    Serial.write(Uart.Uart_Array[Count]);
    Uart.Uart_Array[Count++] = 0x00; 
  } while (Count < sizeof(Uart.Uart_Array));

  Flags.Bit.PcUartRequest = 0;
  UartToUart = WaitDataFromPc;  // Idle Step  
}
//======================================================================
