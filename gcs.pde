#define CHECK_PAYLOAD_SIZE(id) if (txspace < MAVLINK_NUM_NON_PAYLOAD_BYTES+MAVLINK_MSG_ID_ ## id ## _LEN) return false

const AP_Param::GroupInfo GCS_MAVLINK::var_info[] PROGMEM = {
		AP_GROUPEND
};

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
//                      ^ because Consistency is too_hard    :-/
{
	bool result = false;

	switch (msg->msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
		hal.console->printf_P(PSTR("rcvd: HBEAT\n"));
		if(msg->sysid != mavlink_system.sysid)
		{
			return;
		}
		failsafe_heartbeat();
		break;

//TODO
//	case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
//	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
//	case MAVLINK_MSG_ID_PARAM_SET:
//		break;

	case MAVLINK_MSG_ID_COMMAND_LONG:
		mavlink_command_long_t packet;
		mavlink_msg_command_long_decode(msg, &packet);
		switch(packet.command) {

		case MAV_CMD_NAV_TAKEOFF:
			hal.console->printf_P(PSTR("rcvd: TAKEOFF\n"));
			result = hpr_launch(); // READY_FOR_LAUNCH -> POWERED_FLIGHT
			break;

		case MAV_CMD_DO_SET_MODE:
			switch ((uint8_t)packet.param1) {
			case MAV_MODE_AUTO_ARMED:
				hal.console->printf_P(PSTR("rcvd: AUTOARM\n"));
				result = hpr_set_watch_for_launch(); // ORIENTATED -> WATCH_FOR_LAUNCH
				break;

			case MAV_MODE_AUTO_DISARMED:
				hal.console->printf_P(PSTR("rcvd: AUTODIS\n"));
				switch (hpr_get_state())
				{
				case HPR_MODE_LANDED:
					result = hpr_set_recovered(); // LANDED -> RECOVERED
					break;
				case HPR_MODE_WATCH_FOR_LAUNCH:
					result = hpr_unset_watch_for_launch(); // WATCH_FOR_LAUNCH -> ORIENTATED
					break;
				break;
				default:
					result = false;
				}
				break;

			case MAV_MODE_MANUAL_ARMED:
				hal.console->printf_P(PSTR("rcvd: MANARM\n"));
				result = hpr_set_ready_to_launch(); // ORIENTATED -> READY_TO_LAUNCH
				break;

			case MAV_MODE_MANUAL_DISARMED:
				hal.console->printf_P(PSTR("rcvd: MANDIS\n"));
				result = hpr_unset_ready_to_launch(); // READY_TO_LAUNCH -> ORIENTATED
				break;

			case MAV_MODE_FLAG_CUSTOM_MODE_ENABLED:
				hal.console->printf_P(PSTR("rcvd: CUSTOM %u\n"), (uint8_t)packet.param2);
				// attempt to go directly into a mode:
				switch ((uint8_t)packet.param2) {
				case HPR_MODE_ORIENTATED:
					result = hpr_set_orientated();
					break;
				case HPR_MODE_READY_TO_LAUNCH:
					result = hpr_set_ready_to_launch();
					break;
				case HPR_MODE_WATCH_FOR_LAUNCH:
					result = hpr_set_watch_for_launch();
					break;
				case HPR_MODE_POWERED_FLIGHT:
					result = hpr_launch();
					break;
				case HPR_MODE_SCRUBBED:
					result = hpr_set_scrubbed();
					break;
				case HPR_MODE_RECOVERED:
					result = hpr_set_recovered();
					break;
				default:
					result = false;
					break;
				}
				break;
			default:
				hal.console->printf_P(PSTR("rcvd: OTHERSETMODE\n"));
				result = false;
				break;
			} // MAV_CMD_DO_SET_MODE
			break;

		case MAV_CMD_DO_SET_HOME:
			hal.console->printf_P(PSTR("rcvd: DOSETHOME\n"));
			gps_set_home();
			result = true;
			break;

//		case MAV_CMD_DO_SET_PARAMETER:
//			break;

		case MAV_CMD_PREFLIGHT_CALIBRATION:
			hal.console->printf_P(PSTR("rcvd: PRECAL\n"));
			result = hpr_set_orientated(); // SYSTEM_READY -> ORIENTATED
			break;

		case MAV_CMD_DO_FLIGHTTERMINATION:
			hal.console->printf_P(PSTR("rcvd: TERM\n"));
			result = hpr_set_scrubbed(); // *<POWERED_FLIGHT -> SCRUBBED
			break;

// TODO: consider
//		case MAV_CMD_DO_PARACHUTE:
//			break;
//

		default:
			hal.console->printf_P(PSTR("rcvd: OTHERLONG\n"));
			result = false;
			break;

		} //packet.command switch

		mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result?MAV_RESULT_ACCEPTED:MAV_RESULT_FAILED);
		break;

	case MAVLINK_MSG_ID_COMMAND_ACK:
		hal.console->printf_P(PSTR("rcvd: CMDACK\n"));
		gcs_command_acks++;
		break;

//TODO
//	case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
//	case MAVLINK_MSG_ID_LOG_ERASE:
//	case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
//	case MAVLINK_MSG_ID_LOG_REQUEST_END:
//		break;

	default:
		hal.console->printf_P(PSTR("rcvd: OTHERCMD\n"));
		break;
	}
}

bool GCS_MAVLINK::try_send_message(enum ap_message id)
{
	uint16_t txspace = comm_get_txspace(chan);

	/*
	if (gcs_telemetry_delayed(chan)) {
        return false;
    }
    */

	switch(id)
	{
    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        ml_send_attitude(chan);
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
        ml_send_heartbeat(chan);
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
//        send_hwstatus(chan);
        break;

    case MSG_LOCATION:
    	CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
    	ml_send_location(chan);
        break;

    case MSG_GPS_RAW:
    	CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT); //FIXME: technically inaccurate when sending raw
    	gps.send_mavlink_gps_raw(chan);
        break;

    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
//        gcs[chan-MAVLINK_COMM_0].send_raw_imu(ins, compass);
        ml_send_raw_imu(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
//        gcs[chan-MAVLINK_COMM_0].send_scaled_pressure(barometer);
        ml_send_scaled_pressure(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
//        gcs[chan-MAVLINK_COMM_0].send_sensor_offsets(ins, compass, barometer);
        ml_send_sensor_offsets(chan);
        break;

    case MSG_STATUSTEXT:
    	CHECK_PAYLOAD_SIZE(STATUSTEXT);
        ml_send_statustext(chan);
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
//        gcs[chan-MAVLINK_COMM_0].send_system_time(gps);
        break;

	default:
		hal.console->printf_P(PSTR("GCS cannot send message type='%lu'\n"), id);
		break;
	}

	return true;
}

void gcs_init(void)
{
	hal.console->printf_P(PSTR("Initializing %u GCS ports..."), num_gcs);

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

    // identify ourselves correctly with the ground station
    //FIXME: mavlink_system.sysid = global.sysid_this_mav;
    mavlink_system.sysid = MAV_SYSTEM_ID;
    gcs_send_heartbeat();

    hal.console->println();
    gcs_send_text_P(SEVERITY_LOW, PSTR("GCS Initialized"));
}

void gcs_send_attitude(void)
{
	ml_send_message(MSG_ATTITUDE);
}

void gcs_send_heartbeat(void)
{
	ml_send_message(MSG_HEARTBEAT);
}

void gcs_send_location(void)
{
	ml_send_message(MSG_LOCATION);
}

void gcs_send_raw_data(void)
{
	ml_send_message(MSG_GPS_RAW);
	ml_send_message(MSG_RAW_IMU1); //ins
	ml_send_message(MSG_RAW_IMU2); //barometer
	ml_send_message(MSG_RAW_IMU3); //offsets
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

// are we still delaying telemetry to try to avoid Xbee bricking?
/*
/bool gcs_telemetry_delayed(mavlink_channel_t chan)
{
	//FIXME
	return false;
}
*/

void gcs_update(void)
{
    for (uint8_t i=0; i<num_gcs; i++) {
        if (gcs[i].initialised) {
        	gcs[i].update(NULL); //this will call .handleMessage
        } else {
        	hal.console->printf_P(PSTR("GCS %u not initialized.\n"), i);
        }
    }
}
