#include <AP_AHRS.h>
#include <AP_InertialNav.h>

#include "arduhpr.h"

// This seems to work for a pretty stupid reason
//extern AP_InertialSensor ins;
//extern AP_Baro barometer;
//extern AP_GPS gps;
//extern GPS_Glitch gps_glitch;
//extern Baro_Glitch baro_glitch;
// All of the sensors are configured static in their own classes
AP_AHRS_DCM ahrs(ins, barometer, gps);
AP_InertialNav nav(ahrs, barometer, gps_glitch, baro_glitch);

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
