//=============================================================================
// File: SyncRead.cpp
//  Handle the SyncRead command
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

//-----------------------------------------------------------------------------
// sync_read: this handles the sync read message and loops through each of the
// servos and reads the specified registers and packs the data back up into
// one usb message.
// Note: we pass through the ID as the other side validation will look to 
// make sure it matches...
//-----------------------------------------------------------------------------
void sync_read(uint8_t id, uint8_t* params, uint8_t nb_params) {

  // divert incoming data to a buffer for local processing
  g_passthrough_mode = AX_DIVERT;

  uint8_t addr = params[0];    // address to read in control table
  uint8_t nb_to_read = params[1];    // # of bytes to read from each servo
  uint8_t nb_servos = nb_params - 2;

  PCSerial.write(0xff);
  PCSerial.write(0xff);
  PCSerial.write(id);
  PCSerial.write(2 + (nb_to_read * nb_servos));
  PCSerial.write(0);  //error code

  // get ax data
  uint8_t checksum = id + (nb_to_read * nb_servos) + 2; // start accumulating the checksum
  uint8_t* servos = params + 2; // pointer to the ids of the servos to read from
  for (uint8_t servo_id = 0; servo_id < nb_servos; servo_id++) {
    if ( ax12GetRegister(servos[servo_id], addr, nb_to_read)) {
      for (uint8_t i = 0; i < nb_to_read; i++) {
        checksum += ax_rx_buffer[i + 5];
        PCSerial.write(ax_rx_buffer[i + 5]);
#ifdef DBGSerial
  DBGSerial.print(ax_rx_buffer[i + 5], HEX);
  DBGSerial.print(" ");
#endif
      }
    } else {
      for (uint8_t i = 0; i < nb_to_read; i++) {
        checksum += 0xFF;
        PCSerial.write(0xFF);
      }
    }
  }

  PCSerial.write(255 - ((checksum) % 256));
#ifdef DBGSerial
  DBGSerial.println("SF");
#endif
  PCSerial.flush();

  // allow data from USART to be sent directly to USB
  g_passthrough_mode = AX_PASSTHROUGH;
}


