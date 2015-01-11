#ifndef _ARDUHPR_H
#define _ARDUHPR_H

#include <AP_HAL_Boards.h>
#include <AP_AHRS.h>
#include <AP_InertialNav.h>
#include <GCS.h> // has to be here for the build system to find gcs enums

#include "config.h"

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

// for mavlink SET_POSITION_TARGET messages
#define MAVLINK_SET_POS_TYPE_MASK_POS_IGNORE      ((1<<0) | (1<<1) | (1<<2))
#define MAVLINK_SET_POS_TYPE_MASK_VEL_IGNORE      ((1<<3) | (1<<4) | (1<<5))
#define MAVLINK_SET_POS_TYPE_MASK_ACC_IGNORE      ((1<<6) | (1<<7) | (1<<8))
#define MAVLINK_SET_POS_TYPE_MASK_FORCE           (1<<10)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_IGNORE      (1<<11)
#define MAVLINK_SET_POS_TYPE_MASK_YAW_RATE_IGNORE (1<<12)

extern AP_AHRS_DCM ahrs;
extern AP_InertialNav nav;
void ahrs_init(void);
void nav_init(float dt);

void barometer_accumulate(void);
void barometer_init(void);
void barometer_print(void);
void barometer_send_raw(mavlink_channel_t chan);
void barometer_update(void);

void compass_accumulate(void);
void compass_init(void);
const Vector3f &compass_get_field(int pos);
void compass_print(void);
void compass_update(void);

void gcs_init(void);
void gcs_heartbeat(void);
void gcs_send_text_P(gcs_severity severity, const prog_char_t *str);
void gcs_send_location(void);
void gcs_send_raw_data(void);

void gps_init(void);
int  gps_get_last_fix_time_ms(void);
const Location &gps_get_location(void);
int  gps_get_status(void);
const Vector3f &gps_get_vel(void);
void gps_print(void);
void gps_send_raw(mavlink_channel_t chan);
void gps_update(void);

void ins_init(void);
const Vector3f &ins_get_gryo(void);
void ins_print(void);
void ins_sample(void);
void ins_send_raw(mavlink_channel_t chan);
void ins_update(void);

#endif // _ARDUHPR_H
