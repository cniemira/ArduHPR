//FIXME: move to ins.pde
void ml_send_attitude(mavlink_channel_t chan)
{
    const Vector3f &gyro = ins.get_gyro();
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

void ml_send_heartbeat(mavlink_channel_t chan)
{
    uint8_t base_mode = 0;
    uint8_t system_status = MAV_STATE_ACTIVE;
//    uint32_t custom_mode = 3; //FIXME AUTO=3

    uint8_t hpr_state = hpr_get_state();
    uint8_t hpr_status = hpr_get_status();

    switch (hpr_status)
    {
    case HPR_STATUS_UNKNOWN:
    	system_status = MAV_STATE_UNINIT;
    	break;

    case HPR_STATUS_EMERGENCY:
    	system_status = MAV_STATE_EMERGENCY;
    	break;

    case HPR_STATUS_WATCHING:
    	base_mode = MAV_MODE_AUTO_ARMED | MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
    	system_status = MAV_STATE_CRITICAL;
    	break;

    default:
        switch (hpr_state) {
        case HPR_MODE_INITIALIZING:
        	system_status = MAV_STATE_CALIBRATING;
        	break;

        case HPR_MODE_SYSTEM_READY:
        case HPR_MODE_ORIENTATED:
        case HPR_MODE_RECOVERED:				//FIXME: a post-active status would be nice
        	system_status = MAV_STATE_STANDBY;
        	break;

        case HPR_MODE_READY_TO_LAUNCH:
        	base_mode = MAV_MODE_MANUAL_ARMED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
        	system_status = MAV_STATE_STANDBY;
        	break;

        case HPR_MODE_WATCH_FOR_LAUNCH:
        	base_mode = MAV_MODE_AUTO_ARMED;
        	break;

        case HPR_MODE_POWERED_FLIGHT:
        case HPR_MODE_RECOVERY:
        case HPR_MODE_LANDED:
        	base_mode = MAV_MODE_AUTO_ARMED | MAV_MODE_FLAG_AUTO_ENABLED;
			break;

        case HPR_MODE_GLIDE_FLIGHT:
        	// indicate that recovery pyros have auto-armed
        	base_mode = MAV_MODE_AUTO_ARMED | MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
			break;
        }
    break;
    }

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_ROCKET,
		MAV_AUTOPILOT_INVALID, // MAV_AUTOPILOT_GENERIC
        base_mode,
        hpr_state,
        system_status);
}

void ml_send_raw_imu(mavlink_channel_t chan)
{
    const Vector3f &accel = ins.get_accel(0);
    const Vector3f &gyro = ins.get_gyro(0);
    const Vector3f &mag = compass.get_field(0);

    mavlink_msg_raw_imu_send(
        chan,
        hal.scheduler->micros(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z);
}

void ml_send_scaled_pressure(mavlink_channel_t chan)
{
    float pressure = barometer.get_pressure();
    mavlink_msg_scaled_pressure_send(
        chan,
        hal.scheduler->millis(),
        pressure*0.01f, 									// hectopascal
        (pressure - barometer.get_ground_pressure())*0.01f, // hectopascal
        barometer.get_temperature()*100); 					// 0.01 degrees C
}

static void NOINLINE ml_send_location(mavlink_channel_t chan)
{
    uint32_t fix_time;
    // if we have a GPS fix, take the time as the last fix time. That
    // allows us to correctly calculate velocities and extrapolate
    // positions.
    // If we don't have a GPS fix then we are dead reckoning, and will
    // use the current boot time as the fix time.
    if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        fix_time = gps.last_fix_time_ms();
    } else {
        fix_time = hal.scheduler->millis();
    }

//    const Location &loc = gps_get_location();
    const Vector3f &vel = gps.velocity();
    mavlink_msg_global_position_int_send(
        chan,
        fix_time,
		nav.get_latitude(),			// in 1E7 degrees
		nav.get_longitude(),		// in 1E7 degrees
        gps_location.alt * 10UL,	// millimeters above sea level
		nav.get_altitude() * 10,	// millimeters above ground
        vel.x * 100,  				// X speed cm/s (+ve North)
        vel.y * 100,  				// Y speed cm/s (+ve East)
        vel.x * -100, 				// Z speed cm/s (+ve up)
		ahrs.yaw_sensor);			// compass heading in 1/100 degree
}

void NOINLINE ml_send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
        	gcs[i].send_message(id);
        } else {
        	hal.console->printf_P(PSTR("GCS %u not initialized.\n"), i);
        }
    }
}

void NOINLINE ml_send_sensor_offsets(mavlink_channel_t chan)
{
    // run this message at a much lower rate - otherwise it
    // pointlessly wastes quite a lot of bandwidth
    static uint8_t counter;
    if (counter++ < 10) {
        return;
    }
    counter = 0;

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
}

void NOINLINE ml_send_statustext(mavlink_channel_t chan)
{
    mavlink_statustext_t *s = &gcs[chan-MAVLINK_COMM_0].pending_status;
    mavlink_msg_statustext_send(
        chan,
        s->severity,
        s->text);
}
