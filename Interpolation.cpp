//=============================================================================
// File: Interpolation.cpp
// Description: Add servo Interpolation to the Servo controller
//=============================================================================

//=============================================================================
// Header Files
//=============================================================================

#include <EEPROM.h>
#include <ax12Serial.h>
#include <BioloidSerial.h>
#include "globals.h"

// For AX12 like servos max values are 1023 if we shift left 3 bits still no problem fitting.
// for MX servos I believe max is 4095, so again should fit when shifted left 3, with room for sign bit...
typedef struct _poseinfo {
  uint16_t		pose_;
  uint16_t		next_pose_;
  uint16_t		speed_;
} POSEINFO;

POSEINFO g_aPoseinfo[TDSC_MAX_GM_SIZE];

// Define some Timer reference times and delta time.
uint32_t g_pose_next_frame_time = (uint32_t) - 1;

//-----------------------------------------------------------------------------
//  Forward references
//-----------------------------------------------------------------------------
extern void AbortGroupMove(void);
extern void ReadInCurrentPose(void);
extern void SetupGroupMove(void);

//-----------------------------------------------------------------------------
// ProcessGroupMoveCommand() - This gets called when the TDSC_GM_COMMAND
//         register is updated.  The bits of this command are interpretated
//         by this function.
//-----------------------------------------------------------------------------
void ProcessGroupMoveCommand(void)
{
  if (g_controller_registers[TDSC_GM_COMMAND] == 0)
    return;		// nothing specified.
  if (g_controller_registers[TDSC_GM_COMMAND] & TDSC_GM_CMD_ABORT)
  {
    AbortGroupMove();
  }
  // Hack if next frame is -1 first time through so better read in values.
  if ((g_controller_registers[TDSC_GM_COMMAND] & TDSC_GM_CMD_READ) || (g_pose_next_frame_time == (uint32_t) - 1))
  {
    ReadInCurrentPose();
  }
  // Now see if we have a new move to setup.
  if (g_controller_registers[TDSC_GM_COMMAND] & (TDSC_GM_CMD_CHAIN | TDSC_GM_CMD_NEW))
  {
    if (g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] && (g_controller_registers[TDSC_GM_COMMAND] & TDSC_GM_CMD_CHAIN ))
      g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] = TDSC_GM_CMD_CHAIN;	// just leave in chaining mode
    else
    {
      SetupGroupMove();
    }
  }
  else
    g_controller_registers[TDSC_GM_COMMAND] = 0;	// clear out command

}

//-----------------------------------------------------------------------------
// ProcessTimedMoveCommand() - Add a singe timed move...
//-----------------------------------------------------------------------------
void ProcessTimedMoveCommand()
{
  // First verify slot is valid
  uint8_t iSlot = g_controller_registers[TDSC_TM_SLOT];
  if (iSlot >= g_controller_registers[TDSC_GM_SERVO_CNT_TOTAL])
    return; // Nope bail.

  if (g_controller_registers[TDSC_TM_COMMAND] == 0)
    return;    // nothing specified.

  if (g_controller_registers[TDSC_TM_COMMAND] & TDSC_GM_CMD_ABORT)
  {
    if (g_aPoseinfo[iSlot].speed_)
    {
      g_aPoseinfo[iSlot].next_pose_ = g_aPoseinfo[iSlot].pose_ ;
      g_aPoseinfo[iSlot].speed_ = 0;
      g_controller_registers[TDSC_GM_SERVO_CNT_MOVING]--;  // was already moving so don't add twice.
    }
  }

  if (g_controller_registers[TDSC_TM_COMMAND] & TDSC_GM_CMD_READ)
  {
    uint16_t w = ax12GetRegister(g_controller_registers[TDSC_GM_SERVO_0_ID + iSlot], AX_GOAL_POSITION_L, 2);
    if (w != (uint16_t) - 1)
      g_aPoseinfo[iSlot].pose_ = w << BIOLOID_SHIFT;
    else
      g_aPoseinfo[iSlot].pose_ = (uint16_t)512 << BIOLOID_SHIFT;  // If read fails init to something...
  }

  // Now see if we have a new move to setup.
  if (g_controller_registers[TDSC_TM_COMMAND] & (TDSC_GM_CMD_CHAIN | TDSC_GM_CMD_NEW))
  {
    uint16_t wMoveTime = g_controller_registers[TDSC_TM_GOAL_POS_L] + (uint16_t)(g_controller_registers[TDSC_TM_MOVE_TIME_H] << 8);
    uint16_t wMoveIters = (wMoveTime / g_controller_registers[TDSC_GM_FRAME_TIME_MS]) + 1;

    g_aPoseinfo[iSlot].next_pose_ = ((uint16_t)(g_controller_registers[TDSC_TM_GOAL_POS_L] +
                                     ((uint16_t)(g_controller_registers[TDSC_TM_GOAL_POS_H]) << 8)) << BIOLOID_SHIFT);

    // Is this slot already moving?
    if (g_aPoseinfo[iSlot].speed_)
      g_controller_registers[TDSC_GM_SERVO_CNT_MOVING]--;  // was already moving so don't add twice.

    // Handle case where we already had a speed...
    if (g_aPoseinfo[iSlot].next_pose_ != g_aPoseinfo[iSlot].pose_)
    {
      if (g_aPoseinfo[iSlot].next_pose_ > g_aPoseinfo[iSlot].pose_)
      {
        g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].next_pose_ - g_aPoseinfo[iSlot].pose_) / wMoveIters + 1;
      } else {
        g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].pose_ - g_aPoseinfo[iSlot].next_pose_) / wMoveIters + 1;
      }
      g_controller_registers[TDSC_GM_SERVO_CNT_MOVING]++;
    } else
      g_aPoseinfo[iSlot].speed_ = 0;
  }
  g_controller_registers[TDSC_TM_COMMAND] = 0;
}


//-----------------------------------------------------------------------------
// ReadInCurrentPose() - Issue query to each of the servos to read in their
// 				current positions. Normally needed on first time through
//-----------------------------------------------------------------------------
void ReadInCurrentPose()
{
#ifdef DBGSerial
  DBGSerial.println("GM: Read in Pose");
#endif
  // Pass 1, cancel all, next pass look at PB for servo list to cancel
  // divert incoming data to a buffer for local processing
  // Tell the system we need the bytes ourself, don't pass on to the host
  g_passthrough_mode = AX_DIVERT;
  for (uint8_t iSlot = 0; iSlot < g_controller_registers[TDSC_GM_SERVO_CNT]; iSlot++) {
    // We need to load in the current position from the actual servo.
    // Need to better abstract where we get the ID from.
    g_aPoseinfo[iSlot].pose_ = ax12GetRegister(g_controller_registers[TDSC_GM_SERVO_0_ID + iSlot], AX_GOAL_POSITION_L, 2);
    if (g_aPoseinfo[iSlot].pose_ != (uint16_t) - 1)
      g_aPoseinfo[iSlot].pose_ <<= BIOLOID_SHIFT;
    else
      g_aPoseinfo[iSlot].pose_ = (uint16_t)512 << BIOLOID_SHIFT;	// If read fails init to something...

    g_aPoseinfo[iSlot].next_pose_ = g_aPoseinfo[iSlot].pose_ ;
    g_aPoseinfo[iSlot].speed_ = 0;
  }
  g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] = 0;	// Clear out the Interpolating state.
  g_passthrough_mode = AX_PASSTHROUGH;
}


//-----------------------------------------------------------------------------
// SetupGroupMove: Lets do a setup of the group move.
//    This assumes that the user has set the IDs of the servos in the
//    in the group move, and the new positions are also stored as registers.
//-----------------------------------------------------------------------------
void SetupGroupMove(void)
{
  if (g_controller_registers[TDSC_GM_SERVO_CNT] >= g_controller_registers[TDSC_GM_SERVO_CNT_TOTAL])
    g_controller_registers[TDSC_GM_SERVO_CNT_TOTAL] = g_controller_registers[TDSC_GM_SERVO_CNT];

  int8_t cServosInterpolating = g_controller_registers[TDSC_GM_SERVO_CNT_MOVING];
  uint16_t wMoveTime = g_controller_registers[TDSC_GM_MOVE_TIME_L] + (uint16_t)(g_controller_registers[TDSC_GM_MOVE_TIME_H] << 8);
  uint16_t wMoveIters = (wMoveTime / g_controller_registers[TDSC_GM_FRAME_TIME_MS]) + 1;

  // Now lets walk through the data extracting servos and positions
  // Try to hack in some performance helper if the user passes in the
  // servos in the order of the slots...
  uint8_t goal_pos_reg_index = TDSC_GM_SERVO_0_GOAL_POS_L;
  for (uint8_t iSlot = 0; iSlot < g_controller_registers[TDSC_GM_SERVO_CNT]; iSlot++)
  {
    g_aPoseinfo[iSlot].next_pose_ = ((uint16_t)(g_controller_registers[goal_pos_reg_index] +
                                     ((uint16_t)(g_controller_registers[goal_pos_reg_index + 1]) << 8)) << BIOLOID_SHIFT);
    goal_pos_reg_index += 2;
    // Compute Speed.
    if (g_aPoseinfo[iSlot].speed_)
      cServosInterpolating--;	// was already moving so don't add twice.
    // Handle case where we already had a speed...
    if (g_aPoseinfo[iSlot].next_pose_ != g_aPoseinfo[iSlot].pose_)
    {
      if (g_aPoseinfo[iSlot].next_pose_ > g_aPoseinfo[iSlot].pose_)
      {
        g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].next_pose_ - g_aPoseinfo[iSlot].pose_) / wMoveIters + 1;
      } else {
        g_aPoseinfo[iSlot].speed_ = (g_aPoseinfo[iSlot].pose_ - g_aPoseinfo[iSlot].next_pose_) / wMoveIters + 1;
      }
      cServosInterpolating++;
    } else
      g_aPoseinfo[iSlot].speed_ = 0;
  }

  // If we are now starting a new move 0 out the move timer and calculate a good timeout
  if (cServosInterpolating < 0)
    cServosInterpolating = 0;
  if (!g_controller_registers[TDSC_GM_SERVO_CNT_MOVING]) {
    g_pose_next_frame_time =  millis() + g_controller_registers[TDSC_GM_FRAME_TIME_MS];
    // fudge factor each servo adds 3 bytes (.01 second per byte) so to have commands end at the same time,
    // we fudge our start time to deal with this...
  }
  g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] = cServosInterpolating;   // don't clear if other move is happening
#ifdef DBGSerial
  DBGSerial.printf("GM: Pose T:%u I:%d C:%u\n\r", wMoveTime, wMoveIters, cServosInterpolating);
#endif

  // Clear out command
  g_controller_registers[TDSC_GM_COMMAND] = 0;

}


//-----------------------------------------------------------------------------
// AbortGroupMove - Command register was set to abort any current moves
//-----------------------------------------------------------------------------
void AbortGroupMove()
{
  // Pass 1, cancel all, next pass look at PB for servo list to cancel
  for (uint8_t iSlot = 0; iSlot < g_controller_registers[TDSC_GM_SERVO_CNT]; iSlot++) {
    g_aPoseinfo[iSlot].next_pose_ = g_aPoseinfo[iSlot].pose_ ;
    g_aPoseinfo[iSlot].speed_ = 0;
  }
  g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] = 0;	// Clear out the Interpolating state.
}

//-----------------------------------------------------------------------------
// PoseInterpolateStep - This is the main interpolation function.  We call this from the
// main loop.  It will first look to see if there are any moves pending and if we have
// waited long enough from the last time before it will process any changes in the servos
//-----------------------------------------------------------------------------

void PoseInterpolateStepTask(void) {
  // If no interpolation is active or a frame timeout has not happened yet return now.
  if (!g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] || (millis() < g_pose_next_frame_time))
    return;

  // Turn on main move pin
  // This one could do only once when a group move happens
  if (g_controller_registers[TDSC_GM_IO_PIN_MOVE_ACTIVE] != 0xff)
    digitalWriteFast(g_controller_registers[TDSC_GM_IO_PIN_MOVE_ACTIVE], HIGH);

  if (g_controller_registers[TDSC_GM_IO_PIN_MOVE_INTERPOLATE] != 0xff)
    digitalWriteFast(g_controller_registers[TDSC_GM_IO_PIN_MOVE_INTERPOLATE], HIGH);

  setAXtoTX();	// make sure we are in output mode.
  int length = 4 + (g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] * 3);   // 3 = id + pos(2byte)
  int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_POSITION_L;
  ax12writeB(0xFF);
  ax12writeB(0xFF);
  ax12writeB(0xFE);
  ax12writeB(length);
  ax12writeB(AX_SYNC_WRITE);
  ax12writeB(AX_GOAL_POSITION_L);
  ax12writeB(2);

  // Need to loop through all of the slots looking for items that need to be updated.
  // Also build a sync write to do the actual move.
  uint8_t bCntStillMoving = 0;
  for (uint8_t iSlot = 0; iSlot < g_controller_registers[TDSC_GM_SERVO_CNT_TOTAL]; iSlot++) {
    int diff = (int)g_aPoseinfo[iSlot].next_pose_ - (int)g_aPoseinfo[iSlot].pose_;
    if (diff) {
      if (diff > 0) {
        if (diff <= g_aPoseinfo[iSlot].speed_) {
          g_aPoseinfo[iSlot].pose_ = g_aPoseinfo[iSlot].next_pose_;
          g_aPoseinfo[iSlot].speed_ = 0;
        } else {
          g_aPoseinfo[iSlot].pose_ += g_aPoseinfo[iSlot].speed_;
          bCntStillMoving++;
        }
      } else {
        if (-diff <= g_aPoseinfo[iSlot].speed_) {
          g_aPoseinfo[iSlot].pose_ = g_aPoseinfo[iSlot].next_pose_;
          g_aPoseinfo[iSlot].speed_ = 0;
        } else {
          g_aPoseinfo[iSlot].pose_ -= g_aPoseinfo[iSlot].speed_;
          bCntStillMoving++;
        }
      }

      // Output the three bytes for this servo
      int temp = g_aPoseinfo[iSlot].pose_ >> BIOLOID_SHIFT;
      checksum += (temp & 0xff) + (temp >> 8) + g_controller_registers[TDSC_GM_SERVO_0_ID + iSlot];
      ax12writeB(g_controller_registers[TDSC_GM_SERVO_0_ID + iSlot]);
      ax12writeB(temp & 0xff);
      ax12writeB(temp >> 8);
    }
  }
  // And output the checksum for the move.
  ax12writeB(0xff - (checksum % 256));

  // Let host this output is done.
  if (g_controller_registers[TDSC_GM_IO_PIN_MOVE_INTERPOLATE] != 0xff)
    digitalWriteFast(g_controller_registers[TDSC_GM_IO_PIN_MOVE_INTERPOLATE], LOW);


  g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] = bCntStillMoving;	// Update cnt to still do...
  g_pose_next_frame_time +=  g_controller_registers[TDSC_GM_FRAME_TIME_MS];

  // fudge factor each servo adds 3 bytes (.01 second per byte) so to have commands end at the same time,
  // we fudge our start time to deal with this...
  if (bCntStillMoving)
  {
  }
  else
  {
    // We finished our group move, so cleanup anything we need to.
    if (g_controller_registers[TDSC_GM_IO_PIN_MOVE_ACTIVE] != 0xff)
      digitalWriteFast(g_controller_registers[TDSC_GM_IO_PIN_MOVE_ACTIVE], LOW);
    // See if there is a move waiting to be done.
    if (g_controller_registers[TDSC_GM_SERVO_CNT_MOVING] & TDSC_GM_CMD_CHAIN)
    {
      // We had a pending
      SetupGroupMove(); // this will setup the next move and clear out the status flag.
    }
  }
}

