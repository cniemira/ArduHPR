#include <AP_Baro.h>
#include <AP_Baro_Glitch.h>

#include <GCS.h>
#include <GCS_MAVLink.h>

#include "arduhpr.h"

// Barometer
#if CONFIG_BARO == HAL_BARO_BMP085
AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
 #error Unrecognized CONFIG_BARO setting
#endif
Baro_Glitch baro_glitch(barometer);

static int32_t baro_alt;            // barometer altitude in cm above home
static float baro_climbrate;        // barometer climbrate in cm/s

void barometer_accumulate(void)
{
    barometer.accumulate();
}

void barometer_init(void)
{
	barometer.init();
//    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    barometer.calibrate();
    //barometer.update_calibration();
    // reset glitch protection to use new baro alt
    baro_glitch.reset();
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

void barometer_print(void)
{
	float alt;

	barometer.read();

	alt = barometer.get_altitude();
	if (!barometer.healthy()) {
		hal.console->println("not healthy");
		return;
	}

	hal.console->print("Pressure:");
	hal.console->print(barometer.get_pressure());
	hal.console->print(" Temperature:");
	hal.console->print(barometer.get_temperature());
	hal.console->print(" Altitude:");
	hal.console->print(alt);
	hal.console->printf(" climb=%.2f samples=%u",
				  barometer.get_climb_rate(),
				  (unsigned)barometer.get_pressure_samples());
	hal.console->println();
}

void barometer_send_raw(mavlink_channel_t chan)
{
    float pressure = barometer.get_pressure();
    mavlink_msg_scaled_pressure_send(
        chan,
        hal.scheduler->millis(),
        pressure*0.01f, // hectopascal
        (pressure - barometer.get_ground_pressure())*0.01f, // hectopascal
        barometer.get_temperature()*100); // 0.01 degrees C
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
