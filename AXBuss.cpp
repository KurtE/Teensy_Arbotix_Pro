//=============================================================================
// File: AXBuss.cpp
//  Handle reading any input from the AX Buss, when in forwarding mode.
//  NOTE: May update this later to do this on SerialEvent
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================
#include <EEPROM.h>
#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"

//-----------------------------------------------------------------------------
// Define Global variables
//-----------------------------------------------------------------------------
bool g_data_output_to_usb = false;

//-----------------------------------------------------------------------------
// ProcessInputFromAXBuss - We want to do this in a way that will not
//    cause the function to have to wait. 
//-----------------------------------------------------------------------------
bool ProcessInputFromAXBuss(void)
{
#define RECIVE_CHAR_LOOP_COUNT 15
  int ch; 
  uint8_t loop_count;
  bool characters_read = false;

  // See if any characters are available. 
  if ((ch = HWSERIAL.read()) != -1)
  {
    characters_read = true;  
    g_data_output_to_usb = true;

    do
    {
      PCSerial.write(ch);
#if 1
      // Try to read in next character
      loop_count = RECIVE_CHAR_LOOP_COUNT;
      while (((ch = HWSERIAL.read()) == -1) && --loop_count)
      {
        delayMicroseconds(1);  
      }
#endif      
    } while (ch != -1);

    // Lets try to flush now
    MaybeFlushUSBOutputData();
  }
  return characters_read;
}

//-----------------------------------------------------------------------------
// If we are in pass through and we don't still have any data coming in to 
// us, maybe we should tell USB to send back the data now!
//-----------------------------------------------------------------------------
void MaybeFlushUSBOutputData()
{  
#ifdef PCSerial_USB  
// If we are communicating with USB, then maybe want to do flushes.  If not probably don't need to. 
  if (g_data_output_to_usb)
  {
    g_data_output_to_usb = false;
#ifdef DBGSerial
  DBGSerial.println("UF");
#endif  
    PCSerial.flush();
  }
#endif  
}

//-----------------------------------------------------------------------------
// axStatusPacket - Send status packet back through USB
//-----------------------------------------------------------------------------
void axStatusPacket(uint8_t err, uint8_t* data, uint8_t count_bytes) {
  uint16_t checksum = AX_ID_DEVICE + 2 + count_bytes + err;
#ifdef DBGSerial
  DBGSerial.printf("SP: %d %d\n\r", err, count_bytes);
#endif  
 digitalWriteFast(12, HIGH);

  PCSerial.write(0xff);
  PCSerial.write(0xff);
  PCSerial.write(AX_ID_DEVICE);
  PCSerial.write(2 + count_bytes);
  PCSerial.write(err);
  for (uint8_t i = 0; i < count_bytes; i++) {
    PCSerial.write(data[i]);
    checksum += data[i];
  }
  PCSerial.write(255 - (checksum % 256));
  digitalWriteFast(12, LOW);
  g_data_output_to_usb = true;
}


