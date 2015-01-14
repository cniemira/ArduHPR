void ahrs_init(void)
{
	hal.console->printf_P(PSTR("Initializing AHRS..."));
	ahrs.init();
	ahrs.set_vehicle_class(AHRS_VEHICLE_UNKNOWN);
	ahrs.reset_gyro_drift();
	ahrs.set_fast_gains(true);
	ahrs.set_compass(&compass);
	ahrs.reset();
	hal.console->println();
}

void ahrs_update(void)
{
	ahrs.update();
}

void nav_init(void)
{
	hal.console->printf_P(PSTR("Initializing NAV..."));
	nav.init();
	hal.console->println();
}

void nav_update(float dt)
{
	nav.update(dt);
}
