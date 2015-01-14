// initialize compass
void compass_init(void)
{
	hal.console->printf_P(PSTR("Initializing Compass..."));
    if (!compass.init() || !compass.read()) {
    	hal.console->printf_P(PSTR("FAILED!"));
    	gcs_send_text_P(SEVERITY_HIGH, PSTR("Compass failed to initialize"));
        return;
    }

    compass.set_and_save_offsets(0,0,0,0);
    compass.set_declination(ToRad(0.0));
    hal.console->println();
}

void compass_accumulate(void)
{
	compass.accumulate();
}

// report_compass - displays compass information.  Also called by compassmot.pde
void compass_print(void)
{
	uint8_t i;
    Vector3f offsets;

    if (!compass.healthy()) {
        hal.console->println("not healthy");
        return;
    }

	hal.console->printf_P(PSTR("Mag Dec: %4.4f"),
                    degrees(compass.get_declination()));
    for (i=0; i<compass.get_count(); i++) {
        offsets = compass.get_offsets(i);
        hal.console->printf_P(PSTR(" off-%d: %4.4f, %4.4f, %4.4f;"),
                        (int)i,
                        offsets.x,
                        offsets.y,
                        offsets.z);
    }
	hal.console->println();
}

// should be called at 10hz
void compass_update(void)
{
    compass.read();
}
