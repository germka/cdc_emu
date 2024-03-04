#ifndef __BLUETOOTH_H__
#define __BLUETOOTH_H__
#pragma once
#include "stdbool.h"

void NextTrack(void);
void PrevTrack(void);
void PlayPause(void);
void audioPower(bool state);

/*
 * CSR Button delay
 */
#define CSR_SHORT_PRESS     150
#define CSR_LONG_PRESS      1000
#define CSR_VLONG_PRESS     2500
#define	CSR_VVLONG_PRESS    5000
#define CONFIRMATION_SOUND  true

/*
 * CSR Events (15 flashes max)
 */
#define POWER_ON            1
#define POWER_OFF           2
#define A2DP_CONNECTED      3
#define A2DP_DISCONNECTED   4
#define PLAY_PAUSE          5 //AVRCP
#define	STOP_PLAYING        6 //AVRCP
#define SKIP_FORWARD        7 //AVRCP
#define SKIP_BACKWARD       8 //AVRCP
#define FAST_F_PRESS        9 //AVRCP
#define FAST_F_RELEASE      10 //AVRCP
#define REWIND_PRESS        11 //AVRCP
#define REWIND_RELEASE      12 //AVRCP
#define CSR_MUTE_ON         13
#define CSR_MUTE_OFF        14
#define CSR_CONN_TIMEOUT    15
// #define CSR_AUTO_OFF        19

/*
 * CSR States
 */
#define CONNECTABLE		      1
#define CONNBLE_DISCTED	    2
#define	CONNECTED		        3
#define OUT_CALL		        4
#define INC_CALL		        5
#define ACTIVE_CALL		      6
#define A2DP_STREAMING	    7

#endif // __BLUETOOTH_H__
