//=============================================================================
// File: AXBuss.cpp
//  Handle reading any input from the AX Buss, when in forwarding mode.
//  NOTE: May update this later to do this on SerialEvent
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
bool g_data_output_to_usb = false;
bool g_AX_IS_TX = false;
uint8_t ax_tohost_state = AX_SEARCH_FIRST_FF;
uint8_t ax_tohost_len;
uint8_t ax_receive_toggle = 0;

// See if doing single write to USB speeds things up...
#ifdef BUFFER_TO_PC
uint8_t g_abToPCBuffer[256]; // way more than enough buffer space
uint8_t g_abToPCCnt;
#endif
//-----------------------------------------------------------------------------
// ProcessInputFromAXBuss - We want to do this in a way that will not
//    cause the function to have to wait.
//-----------------------------------------------------------------------------
bool ProcessInputFromAXBuss(void)
{
  int ch;
  uint8_t loop_count;
  bool characters_read = false;

  // See if any characters are available.
  // We keep a quick and dirty state of message
  // processing as a way to see if we should try to quickly push data back to host
  if ((ch = HWSERIAL.read()) != -1)
  {
    characters_read = true;
    g_data_output_to_usb = true;
    ax_tohost_state = AX_SEARCH_FIRST_FF;   // only short cut works for packets that complete in one time frame
#ifdef BUFFER_TO_PC
    g_abToPCCnt = 0;   // no bytes to output
#endif
    do
    {
      debug_digitalWrite( DEBUG_PIN_AX_TO_HOST, HIGH);
#ifdef BUFFER_TO_PC
      g_abToPCBuffer[g_abToPCCnt++] = ch;
#else
      PCSerial.write(ch);
#endif
      debug_digitalWrite( DEBUG_PIN_AX_TO_HOST, LOW);

      // try to guess end of packets
      switch (ax_tohost_state) {
        case AX_SEARCH_FIRST_FF:
          if (ch == 0xFF) {
            ax_tohost_state = AX_SEARCH_SECOND_FF;
          }
          break;

        case AX_SEARCH_SECOND_FF:
          ax_tohost_state = (ch == 0xFF) ? PACKET_ID : AX_SEARCH_FIRST_FF;
          break;

        case PACKET_ID:
          ax_tohost_state = (ch == 0xFF) ? PACKET_ID : PACKET_LENGTH;
          break;

        case PACKET_LENGTH:
          ax_tohost_len = ch; // number of bytes remaining in packet.
          ax_tohost_state = AX_PASS_TO_SERVOS;
          break;

        case AX_PASS_TO_SERVOS:
          ax_tohost_len--;
          if (ax_tohost_len == 0)
            ax_tohost_state = AX_SEARCH_FIRST_FF;
          break;

        default:
          break;
      }
      // Try to read in next character
      loop_count = (ax_tohost_state == AX_SEARCH_FIRST_FF) ? 1 : RECIVE_CHAR_LOOP_COUNT;
      while (((ch = HWSERIAL.read()) == -1) && --loop_count)
      {
        delayMicroseconds(1);
      }
    } while (ch != -1);

    // Lets try to flush now
#ifdef BUFFER_TO_PC
    if (g_abToPCCnt) {
      debug_digitalWrite( DEBUG_PIN_AX_TO_HOST, HIGH);
      PCSerial.write(g_abToPCBuffer, g_abToPCCnt);
      debug_digitalWrite( DEBUG_PIN_AX_TO_HOST, LOW);
    }
#endif
    MaybeFlushDataBackToHost();
  }
  return characters_read;
}

//-----------------------------------------------------------------------------
// If we are in pass through and we don't still have any data coming in to
// us, maybe we should tell USB to send back the data now!
//-----------------------------------------------------------------------------
void MaybeFlushDataBackToHost()
{
#ifdef PCSerial_USB
  // If we are communicating with USB, then maybe want to do flushes.  If not probably don't need to.
  if (g_data_output_to_usb)
  {
    g_data_output_to_usb = false;
#ifdef DBGSerial
//    DBGSerial.println("UF");
#endif
    debug_digitalWrite( DEBUG_PIN_FLUSH_TO_HOST, HIGH);
    PCSerial.flush();
    debug_digitalWrite( DEBUG_PIN_FLUSH_TO_HOST, LOW);
  }
#endif
}

//-----------------------------------------------------------------------------
// axStatusPacket - Send status packet back through USB
//-----------------------------------------------------------------------------
void axStatusPacket(uint8_t err, uint8_t* data, uint8_t count_bytes) {
  uint16_t checksum = AX_ID_DEVICE + 2 + count_bytes + err;
#ifdef DBGSerial
  DBGSerial.printf("SP: %d %d", err, count_bytes);
#endif
  debug_digitalWrite( DEBUG_PIN_SEND_STATUS_PACKET, HIGH);


#ifdef BUFFER_TO_PC
  g_abToPCCnt = 0;
  g_abToPCBuffer[g_abToPCCnt++] = (0xff);
  g_abToPCBuffer[g_abToPCCnt++] = (0xff);
  g_abToPCBuffer[g_abToPCCnt++] = (AX_ID_DEVICE);
  g_abToPCBuffer[g_abToPCCnt++] = (2 + count_bytes);
  g_abToPCBuffer[g_abToPCCnt++] = (err);
  for (uint8_t i = 0; i < count_bytes; i++) {
    g_abToPCBuffer[g_abToPCCnt++] = (data[i]);
    checksum += data[i];
#ifdef DBGSerial
    DBGSerial.printf(" %x", data[i]);
#endif
  }
  g_abToPCBuffer[g_abToPCCnt++] = (255 - (checksum % 256));
  PCSerial.write(g_abToPCBuffer, g_abToPCCnt);
  PCSerial.flush();
#ifdef DBGSerial
  DBGSerial.printf(" %d\n\r", g_abToPCCnt);
#endif

#else
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
#endif
  debug_digitalWrite( DEBUG_PIN_SEND_STATUS_PACKET, LOW);
  g_data_output_to_usb = true;
}


