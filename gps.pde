void gps_init(void)
{
#if GPS_PROTOCOL != GPS_PROTOCOL_IMU
    // standard gps running. Note that we need a 256 byte buffer for some
    // GPS types (eg. UBLOX)
    hal.uartB->begin(38400, 256, 16);
#endif

//	gps.init(&DataFlash);
}

void gps_print(void)
{
	//const Location &loc = gps.location();
	gps_update();
	hal.console->print("Lat: ");
	print_latlon(hal.console, gps_location.lat);
	hal.console->print(" Lon: ");
	print_latlon(hal.console, gps_location.lng);
	hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
		gps_location.alt * 0.01f,
		gps.ground_speed(),
		(int)gps.ground_course_cd() / 100,
		gps.num_sats(),
		gps.time_week(),
		gps.time_week_ms(),
		gps.status());
}

void gps_set_home(void)
{
    gps_is_home = true;

    ahrs.set_home(gps_location);
    nav.setup_home_position();

    // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
//                        scale_long_down = longitude_scale(current_location);
//                        scale_long_up   = 1.0f/scale_long_down;

    // set system clock for log timestamps
    hal.util->set_system_clock(gps.time_epoch_usec());

//                        if (global.compass_enabled) {
        // Set compass declination automatically
        compass.set_initial_location(gps_location.lat, gps_location.lng);
//                        }
}

// called at 50hz
void gps_update(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    static uint8_t ground_start_count = 10;     // counter used to grab at least 10 reads before commiting the Home location
    bool gps_updated = false;

    gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);

            gps_updated = true;
        }
    }

    if (gps_updated) {
    	gps_location = gps.location();

        // run glitch protection and update AP_Notify if home has been initialized
        if (global.home_is_set) {
            gps_glitch.check_position();
        }

        // checks to initialize home
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

            // check if we can initialize home yet
            if (!gps_is_home) {

                // if we have a 3d lock and valid location
                if(gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps_location.lat != 0) {
                    if (ground_start_count > 0 ) {
                        ground_start_count--;
                    } else {
                        // after 10 successful reads store home location
                        // ap.home_is_set will be true so this will only happen once
                        ground_start_count = 0;
                        gps_set_home();
                    }
                } else {
                    // start again if we lose 3d lock
                    ground_start_count = 10;
                }
            }

            //If we are not currently armed, and we're ready to
            //enter RTK mode, then capture current state as home,
            //and enter RTK fixes!
            //FIXME: This should be !motor.armed() &&
            if (gps.can_calculate_base_pos()) {
                gps.calculate_base_pos();
            }
        }
    }

    // check for loss of gps
    //failsafe_gps_check();
}
