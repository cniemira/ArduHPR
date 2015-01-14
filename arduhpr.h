#ifndef _ARDUHPR_H
#define _ARDUHPR_H

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

#define HPR_LAUNCH_RELAY_CLOSE_TIME		5 // uSec to keep the launch relay in order to light igniter
#define HPR_LAUNCH_MISFIRE_WARNING		5000 // uSec after ignition before warning
#define HPR_LAUNCH_MISFIRE_SCRUB		5000 // uSec after ignition before scrubbing
#define HPR_LAUNCH_SCRUB_LOCKOUT		60 // seconds

#define HPR_MODE_INITIALIZING			0 // power on						system bootup
#define HPR_MODE_SYSTEM_READY			1 // automatic						all sensors enabled, gps lock
#define HPR_MODE_ORIENTATED 			2 // MAV_CMD_PREFLIGHT_CALIBRATION	reference orientation locked (home set, gyros reset)
#define HPR_MODE_READY_TO_LAUNCH		3 // MAV_CMD_DO_SET_MODE 			launch pyros armed, high speed telemetry
									  	  //	+MAV_MODE_MANUAL_ARMED
#define HPR_MODE_WATCH_FOR_LAUNCH		4 // MAV_CMD_DO_SET_MODE
									  	  // 	+MAV_MODE_AUTO_ARMED		vehicle will not self-launch
#define HPR_MODE_POWERED_FLIGHT			5 // launch detected
#define HPR_MODE_GLIDE_FLIGHT			6 // end of powered flight			recovery pyos armed
#define HPR_MODE_RECOVERY				7 // apogee detected
#define HPR_MODE_LANDED					8 // landing detected 				hs telem off, audio alarm on
#define HPR_MODE_SCRUBBED				9 // MAV_CMD_DO_FLIGHTTERMINATION
#define HPR_MODE_RECOVERED				10// MAV_CMD_DO_SET_MODE
										  // 	+MAV_MODE_AUTO_DISARMED

#define HPR_STATUS_UNKNOWN				0
#define HPR_STATUS_NORMAL				1
#define HPR_STATUS_WATCHING				2 // launch has been triggered by not observed
#define HPR_STATUS_EMERGENCY			3

#define HPR_TELEM_SLOW_RATE				20;
#define HPR_TELEM_FAST_RATE				2;

enum HPRModes {
	INITIALIZING		= HPR_MODE_INITIALIZING,
	SYSTEM_READY		= HPR_MODE_SYSTEM_READY,
	ORIENTATED			= HPR_MODE_ORIENTATED,
	READY_TO_LAUNCH		= HPR_MODE_READY_TO_LAUNCH,
	WATCH_FOR_LAUNCH	= HPR_MODE_WATCH_FOR_LAUNCH,
	POWERED_FLIGHT		= HPR_MODE_POWERED_FLIGHT,
	GLIDE_FLIGHT		= HPR_MODE_GLIDE_FLIGHT,
	RECOVERY			= HPR_MODE_RECOVERY,
	LANDED				= HPR_MODE_LANDED,
	SCRUBBED			= HPR_MODE_SCRUBBED,
	RECOVERED			= HPR_MODE_RECOVERED,
};

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<11)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<12)

#if MAIN_LOOP_RATE == 400
 # define PERF_INFO_OVERTIME_THRESHOLD_MICROS 3000
#else
 # define PERF_INFO_OVERTIME_THRESHOLD_MICROS 10500
#endif


//autogenerate won't find these:
void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);

#endif // _ARDUHPR_H
