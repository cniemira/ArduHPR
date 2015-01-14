void barometer_accumulate(void)
{
    barometer.accumulate();
}

void barometer_init(void)
{
	hal.console->printf_P(PSTR("Initializing Barometer..."));
	barometer.init();
    barometer.calibrate();
    baro_glitch.reset();

    barometer_update();
    if(!barometer.healthy()) {
    	hal.console->printf_P(PSTR("FAILED!"));
    	gcs_send_text_P(SEVERITY_HIGH, PSTR("Barometer failed to initialize"));
    }
    hal.console->println();
}

void barometer_print(void)
{
	barometer_update();

	if (!barometer.healthy()) {
		hal.console->println("not healthy");
		return;
	}

	hal.console->print("Pressure:");
	hal.console->print(barometer.get_pressure());
	hal.console->print(" Temperature:");
	hal.console->print(barometer.get_temperature());
	hal.console->print(" Altitude:");
	hal.console->print(baro_alt);
	hal.console->printf(" climb=%.2f samples=%u",
		baro_climbrate,
		(unsigned)barometer.get_pressure_samples());
	hal.console->println();
}

// return barometric altitude in centimeters
void barometer_update(void)
{
    barometer.read();
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    // run glitch protection and update AP_Notify if home has been initialized
    baro_glitch.check_alt();
}
