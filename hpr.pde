void hpr_init(void)
{
	hal.console->printf_P(PSTR("Initializing HPR..."));
	hpr_state = HPR_MODE_INITIALIZING;
	hpr_status = HPR_STATUS_UNKNOWN;
	hal.console->println();
}

static bool hpr_deploy(int pyro)
{
	return false;
}

int hpr_get_state(void)
{
	return hpr_state;
}

int hpr_get_status(void)
{
	return hpr_status;
}

static bool hpr_in_state(int okay_state)
{
	if (hpr_status != HPR_STATUS_NORMAL)
	{
		gcs_send_text_P(SEVERITY_HIGH, PSTR("Rocket state is abnormal!"));
		return false;
	}

	if (hpr_state != okay_state)
	{
		gcs_send_text_P(SEVERITY_HIGH, PSTR("Illegal change state command!"));
		return false;
	}

	return true;
}

bool hpr_launch(void)
{
	if (!hpr_in_state(HPR_MODE_READY_TO_LAUNCH))
	{
		return false;
	}

	if (!hpr_system_check(true))
	{
		//TODO: Scrub the launch
		return false;
	}

	ignitionTimer = hal.scheduler->micros();
	//TODO: fire launch igniter

	hpr_state = HPR_MODE_WATCH_FOR_LAUNCH;
	hpr_status = HPR_STATUS_WATCHING;
	return true;
}

// system_ready -> orientated
bool hpr_set_orientated(void)
{
	if (!hpr_in_state(HPR_MODE_SYSTEM_READY))
	{
		return false;
	}

	if (!hpr_system_check(true))
	{
		return false;
	}

	gps_update();
	ahrs.reset_gyro_drift();
	ahrs.reset();
	ahrs.set_home(gps_location);
	hpr_state = HPR_MODE_ORIENTATED;
	return true;
}

bool hpr_set_ready_to_launch(void)
{
	if (!hpr_in_state(HPR_MODE_ORIENTATED))
	{
		return false;
	}

	global.telemetry_freq = HPR_TELEM_FAST_RATE;
	hpr_state = HPR_MODE_READY_TO_LAUNCH;
	return true;
}

bool hpr_set_recovered(void)
{
	if (!hpr_in_state(HPR_MODE_LANDED))
	{
		return false;
	}

	global.telemetry_freq = HPR_TELEM_SLOW_RATE;

	//TODO: turn the notifier off
	hpr_state = HPR_MODE_RECOVERED;
	return true;
}

bool hpr_set_scrubbed(void)
{
	if (hpr_status == HPR_STATUS_WATCHING)
	{
		gcs_send_text_P(SEVERITY_HIGH, PSTR("IGNITER ACTIVE - CANNOT SCRUB!"));
		return false;
	}

	if(hpr_state > WATCH_FOR_LAUNCH) {
		gcs_send_text_P(SEVERITY_HIGH, PSTR("FLIGHT ACTIVE - CANNOT SCRUB!"));
		return false;
	}

	global.telemetry_freq = HPR_TELEM_SLOW_RATE;
	hpr_state = HPR_MODE_SCRUBBED;
	return true;
}

// watch for launch is set in two ways: either by a launch command,
// or manually via this function
bool hpr_set_watch_for_launch(void)
{
	if (!hpr_in_state(HPR_MODE_ORIENTATED))
	{
		return false;
	}

	global.telemetry_freq = HPR_TELEM_FAST_RATE;
	hpr_state = HPR_MODE_WATCH_FOR_LAUNCH;
	return true;
}

static bool hpr_system_check(bool announce)
{
	if (!barometer.healthy())
	{
		if (announce)
			gcs_send_text_P(SEVERITY_HIGH, PSTR("Barometer not healthy!"));
		return false;
	}

	return true;
}

bool hpr_unset_ready_to_launch(void)
{
	if (!hpr_in_state(HPR_MODE_READY_TO_LAUNCH))
	{
		return false;
	}

	global.telemetry_freq = HPR_TELEM_SLOW_RATE;
	hpr_state = HPR_MODE_ORIENTATED;
	return true;
}

bool hpr_unset_watch_for_launch(void)
{
	// note: this will verify that we are not in status=WATCHING
	// you cannot exit watch_for_launch manually when an igniter has fired
	if (!hpr_in_state(HPR_MODE_WATCH_FOR_LAUNCH)) {
		return false;
	}

	global.telemetry_freq = HPR_TELEM_SLOW_RATE;
	hpr_state = HPR_MODE_ORIENTATED;
	return true;
}

void hpr_update(void)
{
	switch (hpr_state) {
	case HPR_MODE_INITIALIZING:
		if(hpr_system_check(false))
		{
			hpr_state = HPR_MODE_SYSTEM_READY;
			hpr_status = HPR_STATUS_NORMAL;
		}
		break;

	case HPR_MODE_SYSTEM_READY:
		break;

	case HPR_MODE_ORIENTATED:
		break;

	case HPR_MODE_READY_TO_LAUNCH:
		break;

	case HPR_MODE_WATCH_FOR_LAUNCH:
		break;

	case HPR_MODE_POWERED_FLIGHT:
		break;

	case HPR_MODE_GLIDE_FLIGHT:
		break;

	case HPR_MODE_RECOVERY:
		break;

	case HPR_MODE_LANDED:
		break;

	case HPR_MODE_SCRUBBED:
		break;

	case HPR_MODE_RECOVERED:
		break;
	}
}

