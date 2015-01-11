#include <AP_Compass.h>

#include <GCS.h>
#include <GCS_MAVLink.h>

#include "arduhpr.h"

// Compass
#if CONFIG_COMPASS == HAL_COMPASS_PX4
AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

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

const Vector3f &compass_get_field(int pos)
{
	const Vector3f &field = compass.get_field(pos);
	return field;
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
