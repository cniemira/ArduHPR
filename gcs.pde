#include <GCS.h>
#include <GCS_MAVLink.h>

#include "arduhpr.h"

#define CHECK_PAYLOAD_SIZE(id) if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_ ## id ## _LEN) return false

#if GCS_ON_UARTA == ENABLED
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];
#else
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS - 1;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS - 1];
#endif

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
		AP_GROUPEND
};

bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
	uint16_t txspace = comm_get_txspace(chan);

	if (gcs_telemetry_delayed(chan)) {
        return false;
    }

	switch(id) {
    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_EXTENDED_STATUS1:
        // send extended status only once vehicle has been initialized
        // to avoid unnecessary errors being reported to user
        /*
    	CHECK_PAYLOAD_SIZE(SYS_STATUS);
		send_extended_status1(chan);
		CHECK_PAYLOAD_SIZE(POWER_STATUS);
		gcs[chan-MAVLINK_COMM_0].send_power_status();
		*/
        break;

    case MSG_HEARTBEAT:
    	CHECK_PAYLOAD_SIZE(HEARTBEAT);
        gcs[chan-MAVLINK_COMM_0].last_heartbeat_time = hal.scheduler->millis();
        send_heartbeat(chan);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
//        send_hwstatus(chan);
        break;

    case MSG_LOCATION:
    	CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
    	send_location(chan);
        break;

    case MSG_GPS_RAW:
    	CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT); //FIXME: technically inaccurate when sending raw
    	gps_send_raw(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
//        gcs[chan-MAVLINK_COMM_0].send_raw_imu(ins, compass);
        ins_send_raw(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
//        gcs[chan-MAVLINK_COMM_0].send_scaled_pressure(barometer);
        barometer_send_raw(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
//        gcs[chan-MAVLINK_COMM_0].send_sensor_offsets(ins, compass, barometer);
        TEST_send_sensor_offsets(chan);
        break;

    case MSG_STATUSTEXT:
    	CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
//        gcs[chan-MAVLINK_COMM_0].send_system_time(gps);
        break;

	default:
		hal.console->printf_P(PSTR("GCS cannot send message type='%lu'\n"), id);
		break;
	}

	//PING
	//PARAMS (req read, list, set
	//GPS RAW
	//GPS STATUS
	//SCALED IMU
	//RAW IMU
	//RAW PRESSURE
	//SCALED PRESSURE
	//DEBUG
	//BATTERY STATUS
	//LOG DATA, REQUEST, ERASE
	//COMMAND INT, LONG, ACK


	return true;
}

static void gcs_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
        	gcs[i].send_message(id);
        } else {
        	hal.console->printf_P(PSTR("GCS %u not initialized.\n"), i);
        }
    }
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    const Vector3f &gyro = ins_get_gyro();
    mavlink_msg_attitude_send(
        chan,
        hal.scheduler->millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        gyro.x,
        gyro.y,
        gyro.z);
}

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint8_t system_status = MAV_STATE_ACTIVE;
    uint32_t custom_mode = 3; //FIXME AUTO=3

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_ROCKET,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.
    if (gps_get_status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps_get_last_fix_time_ms();
    } else {
        fix_time = hal.scheduler->millis();
    }

    const Location &loc = gps_get_location();
    const Vector3f &vel = gps_get_vel();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
		nav.get_latitude(),			// in 1E7 degrees
		nav.get_longitude(),		// in 1E7 degrees
        loc.alt * 10UL,				// millimeters above sea level
		nav.get_altitude() * 10,	// millimeters above ground
        vel.x * 100,  // X speed cm/s (+ve North)
        vel.y * 100,  // Y speed cm/s (+ve East)
        vel.x * -100, // Z speed cm/s (+ve up)
        0); //ahrs.yaw_sensor           // compass heading in 1/100 degree
}

static void NOINLINE TEST_send_sensor_offsets(mavlink_channel_t chan)
{
    // run this message at a much lower rate - otherwise it
    // pointlessly wastes quite a lot of bandwidth
    static uint8_t counter;
    if (counter++ < 10) {
        return;
    }
    counter = 0;

    //TODO: decide if this is necessary
    hal.console->printf_P(PSTR("send sensor offsets\n"));

    /*
    const Vector3f &mag_offsets = compass.get_offsets(0);
    const Vector3f &accel_offsets = ins.get_accel_offsets(0);
    const Vector3f &gyro_offsets = ins.get_gyro_offsets(0);

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_pressure(),
                                    barometer.get_temperature()*100,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
    */
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}

// are we still delaying telemetry to try to avoid Xbee bricking?
static bool gcs_telemetry_delayed(mavlink_channel_t chan)
{
	//FIXME
	return false;
}

void gcs_init(void)
{
	hal.console->printf_P(PSTR("Initializing %u GCS ports..."), num_gcs);

    // Register the mavlink service callback. This will run
    // anytime there are more than 5ms remaining in a call to
    // hal.scheduler->delay.
//    hal.scheduler->register_delay_callback(mavlink_delay_cb, 5);

//#if (CONFIG_HAL_BOARD != HAL_BOARD_APM2) | (APM2_IGNORE_UART_MUX == ENABLED)
    // we have a 2nd serial port for telemetry on all boards except
    // APM2. We actually do have one on APM2 but it isn't necessary as
    // a MUX is used
	hal.uartC->begin(map_baudrate(SERIAL1_BAUD), 256, 256);
	gcs[0].init(hal.uartC);
//#endif

#if GCS_ON_UARTA == ENABLED
    hal.console->printf_P(PSTR(" +UART-A"));
    gcs[1].init(hal.uartA);
#endif

    gcs_heartbeat();

    // identify ourselves correctly with the ground station
    //FIXME: mavlink_system.sysid = global.sysid_this_mav;
    mavlink_system.sysid = MAV_SYSTEM_ID;

    hal.console->println();
    gcs_send_text_P(SEVERITY_LOW, PSTR("GCS Initialized"));
}

void gcs_attitude(void)
{
	gcs_send_message(MSG_ATTITUDE);
}

void gcs_heartbeat(void)
{
	gcs_send_message(MSG_HEARTBEAT);
}

void gcs_send_location(void)
{
	gcs_send_message(MSG_LOCATION);
}

void gcs_send_raw_data(void)
{
	gcs_send_message(MSG_GPS_RAW);
	gcs_send_message(MSG_RAW_IMU1); //ins
	gcs_send_message(MSG_RAW_IMU2); //barometer
	gcs_send_message(MSG_RAW_IMU3); //offsets
}

void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
            gcs[i].send_text_P(severity, str);
            /*
            hal.console->printf_P(PSTR("gcs send (%i): "), i);
            hal.console->printf_P(str);
            hal.console->println();
            */
        }
    }
}

