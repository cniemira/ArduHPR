#include <AP_GPS.h>
#include <AP_GPS_Glitch.h>

// These seem to be required here for the build tooling
#include <GCS.h>
#include <GCS_MAVLink.h>

#include "arduhpr.h"

// GPS
AP_GPS gps;
GPS_Glitch gps_glitch(gps);
//static struct Location current_location;
// We use atan2 and other trig techniques to calculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
//static float scale_long_up = 1;
// Sometimes we need to remove the scaling for distance calcs
//static float scale_long_down = 1;

static bool is_home = false;


void gps_init(void)
{
//	gps.init(&DataFlash);
}

void gps_send_raw(mavlink_channel_t chan)
{
	gps.send_mavlink_gps_raw(chan);
}

int gps_get_last_fix_time_ms(void)
{
	return gps.last_fix_time_ms();
}

const Location &gps_get_location(void)
{
	const Location &loc = gps.location();
	return loc;
}

int gps_get_status(void)
{
	return gps.status();
}

const Vector3f &gps_get_vel(void)
{
	const Vector3f &vel = gps.velocity();
	return vel;
}

void gps_print(void)
{
	const Location &loc = gps.location();
	hal.console->print("Lat: ");
	print_latlon(hal.console, loc.lat);
	hal.console->print(" Lon: ");
	print_latlon(hal.console, loc.lng);
	hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
						loc.alt * 0.01f,
						gps.ground_speed(),
						(int)gps.ground_course_cd() / 100,
						gps.num_sats(),
						gps.time_week(),
						gps.time_week_ms(),
						gps.status());
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
        // run glitch protection and update AP_Notify if home has been initialized
        if (global_state.home_is_set) {
            gps_glitch.check_position();
        }

        // checks to initialize home and take location based pictures
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {

            // check if we can initialize home yet
            if (!is_home) {
            	Location loc = gps.location();

                // if we have a 3d lock and valid location
                if(gps.status() >= AP_GPS::GPS_OK_FIX_3D && loc.lat != 0) {
                    if (ground_start_count > 0 ) {
                        ground_start_count--;
                    } else {
                        // after 10 successful reads store home location
                        // ap.home_is_set will be true so this will only happen once
                        ground_start_count = 0;
                        is_home = true;

                        ahrs.set_home(loc);
                        nav.setup_home_position();

                        // update navigation scalers.  used to offset the shrinking longitude as we go towards the poles
//                        scale_long_down = longitude_scale(current_location);
//                        scale_long_up   = 1.0f/scale_long_down;

                        // set system clock for log timestamps
                        hal.util->set_system_clock(gps.time_epoch_usec());

//                        if (global.compass_enabled) {
                            // Set compass declination automatically
                            compass.set_initial_location(loc.lat, loc.lng);
//                        }
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
