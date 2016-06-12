//=============================================================================
// File: SyncRead.cpp
//  Handle the SyncRead command
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================

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
#define PACKET_OVERHEAD 6
void sync_read(uint8_t id, uint8_t* params, uint8_t nb_params) {

  uint8_t addr = params[0];    // address to read in control table
  uint8_t nb_to_read = params[1];    // # of bytes to read from each servo
  uint8_t nb_servos = nb_params - 2;

  // First do simple validation.
  if ((nb_to_read == 0) || (nb_servos == 0) ||
      (nb_to_read > (AX_BUFFER_SIZE - PACKET_OVERHEAD)) ||
      (((int16_t)nb_to_read * (int16_t)nb_servos) > (AX_BUFFER_SIZE - PACKET_OVERHEAD)) )
  {
    axStatusPacket(ERR_RANGE, NULL, 0);
    return;
  }

  // divert incoming data to a buffer for local processing
  g_passthrough_mode = AX_DIVERT;

#ifdef BUFFER_TO_PC
  g_abToPCCnt = 0;
  g_abToPCBuffer[g_abToPCCnt++] = (0xff);
  g_abToPCBuffer[g_abToPCCnt++] = (0xff);
  g_abToPCBuffer[g_abToPCCnt++] = (id);
  g_abToPCBuffer[g_abToPCCnt++] = (2 + (nb_to_read * nb_servos));
  g_abToPCBuffer[g_abToPCCnt++] = ((uint8_t)0);  //error code
#else
  PCSerial.write(0xff);
  PCSerial.write(0xff);
  PCSerial.write(id);
  PCSerial.write(2 + (nb_to_read * nb_servos));
  PCSerial.write((uint8_t)0);  //error code
#endif
  // get ax data
  uint8_t checksum = id + (nb_to_read * nb_servos) + 2; // start accumulating the checksum
  uint8_t* servos = params + 2; // pointer to the ids of the servos to read from
  for (uint8_t servo_id = 0; servo_id < nb_servos; servo_id++) {
    uint8_t id = servos[servo_id];
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM
    int checksum_out = ~((id + 6 + addr + nb_to_read) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(addr);
    ax12writeB(nb_to_read);
    ax12writeB(checksum_out);

    setRX(id);
    if (ax12ReadPacket(nb_to_read + 6) > 0) {
      for (uint8_t i = 0; i < nb_to_read; i++) {
        uint8_t b = ax_rx_buffer[i + 5];
        checksum += b ;
#ifdef BUFFER_TO_PC
        g_abToPCBuffer[g_abToPCCnt++] = b;
#else
        PCSerial.write(b);
#endif
#ifdef DBGSerial
        DBGSerial.print(b , HEX);
        DBGSerial.print(" ");
#endif
      }
    } else {
      for (uint8_t i = 0; i < nb_to_read; i++) {
        checksum += 0xFF;
#ifdef BUFFER_TO_PC
        g_abToPCBuffer[g_abToPCCnt++] = 0xFF;
#else
        PCSerial.write(0xFF);
#endif
      }
    }
  }

#ifdef BUFFER_TO_PC
  g_abToPCBuffer[g_abToPCCnt++] = (255 - ((checksum) % 256));
  PCSerial.write(g_abToPCBuffer, g_abToPCCnt);

#else
  PCSerial.write(255 - ((checksum) % 256));
#endif
#ifdef DBGSerial
  DBGSerial.println("SF");
#endif
  PCSerial.flush();

  // allow data from USART to be sent directly to USB
  g_passthrough_mode = AX_PASSTHROUGH;
}


