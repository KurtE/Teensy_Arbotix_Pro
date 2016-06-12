//=============================================================================
// File: BulkRead.cpp
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
void bulk_read(uint8_t id, uint8_t* params, uint8_t nb_params) {

}
