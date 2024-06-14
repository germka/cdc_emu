/*
 * C++ Class for handling CD changer emulator on SAAB I-Bus
 * Copyright (C) 2016  Karlis Veilands
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * Created by: Karlis Veilands
 * Created on: Jun 4, 2015
 * Modified by: Roman Sinitsyn
 * Modified on: Feb 12, 2019
 * Modified by: Evgeniy Shabin
 * Modified on: May 25, 2021
 */

#ifndef __CDC_H__
#define __CDC_H__
#pragma once

/**
 * Various constants used for SID text control
 * Abbreviations:
 *      IHU - Infotainment Head Unit
 *      SPA - SAAB Park Assist
 *      EMS - Engine Management System
 */
#define DESIRED_ROW 0x82

//#define NODE_APL_ADR                  0x11    // IHU
#define NODE_APL_ADR                    0x15    // custom
//#define NODE_APL_ADR                  0x21    // ECS
//#define NODE_APL_ADR                  0x1F    // SPA
#define NODE_SID_FUNCTION_ID            0x17    // custom
//#define NODE_SID_FUNCTION_ID          0x18    // SPA
//#define NODE_SID_FUNCTION_ID          0x19    // IHU
//#define NODE_SID_FUNCTION_ID          0x32    // ECS
#define NODE_DISPLAY_RESOURCE_REQ       0x345   // custom
//#define NODE_DISPLAY_RESOURCE_REQ     0x348   // IHU
//#define NODE_DISPLAY_RESOURCE_REQ     0x0357  // SPA
//#define NODE_DISPLAY_RESOURCE_REQ     0x358   // ECS
#define NODE_WRITE_TEXT_ON_DISPLAY      0x325   // custom
//#define NODE_WRITE_TEXT_ON_DISPLAY    0x328   // IHU
//#define NODE_WRITE_TEXT_ON_DISPLAY    0x0337  // SPA
//#define NODE_WRITE_TEXT_ON_DISPLAY    0x33F   // ECS

/**
 * Other useful stuff
 */

#define STATE_CHANGED 0x80

/**
 * STEERING WHEEL AND SID BUTTONS
 * State - byte 0
 * AUDIO - byte 2
 * AUDIO - byte 4 dublicate
 * SID   - byte 3
 * SID   - byte 5 dublicate
 */
#define VOL_DOWN    0x80
#define VOL_UP      0x40
#define SRC         0x20
#define SEEK_NEXT   0x10
#define SEEK_PREV   0x08
#define NEXT        0x04

#define CLR         0x80
#define SET         0x40
#define DOWN        0x20
#define UP          0x10
#define NPANEL      0x08
#define CLOCK_UP    0x04
#define CLOCK_DOWN  0x02

/**
 * Pedals & Gear
 * State - byte 0
 * Gear  - byte 1
 * Pedal - byte 2
 */
#define REVERS_ON   0x02
#define ENGINE_ON   0xC0

/**
 * Automatic gearbox
 * State - byte 0
 * Gear  - byte 1
 * Shift - byte 2
 */

#define GEAR_FORVARD 0x05
#define GEAR_NEUTRAL 0x03
#define GEAR_REVERSE 0x02
#define SHIFT_PARK   0x01
#define SHIFT_DRIVE  0x04
#define SHIFT_LIMIT4 0x08
#define SHIFT_LIMIT3 0x07
#define SHIFT_LOW    0x06

/**
 * frame parameters
 */

#define CAN_DATA_LEN    8

/**
 * TX frames header:
 */
#define GENERAL_STATUS_CDC          0x3C8
#define NODE_STATUS_TX_CDC          0x6A2
#define SOUND_REQUEST               0x430

/**
 * RX frames header:
 */
#define VIN_STATUS_TRIONIC          0x4A0
#define CDC_CONTROL                 0x3C0
#define DISPLAY_RESOURCE_GRANT      0x368
#define NODE_STATUS_RX_IHU          0x6A1
#define STEERING_WHEEL_BUTTONS      0x290
#define PEDALS_GEAR                 0x280
#define AUTOMATIC_GEARBOX           0x3E0

/**
 * Timer definitions:
 */
#define NODE_STATUS_TX_DELAY        20
#define NODE_STATUS_TX_INTERVAL     140     // Replies to '6A1' request need to be sent with no more than 140ms interval; tolerances +/- 10%
#define CDC_STATUS_TX_BASETIME      950     // The CDC status frame must be sent periodically within this timeframe; tolerances +/- 10%
#define SID_CONTROL_TX_BASETIME     1000    // SID control/resource request frames needs to be sent within this timeframe; tolerances +/- 10%


/**
 * SID sound type definitions:
 */
#define SOUND_ACK                   0x04    // Short "Beep"
#define SOUND_TAC                   0x08    // "Tack"
#define SOUND_TIC                   0x10    // "Tick"
#define SOUND_SEATBELT              0x20    // Seat belt warning
#define SOUND_ALERT                 0x40    // Short "Ding-Dong"


#endif  // __CDC_H__
