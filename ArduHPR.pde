#define FIRMWARE_ID "ArduHPR v0.1a"

/*
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_NavEKF.h>
#include <AP_Rally.h>

#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_PX4.h>
*/

// NO SPACES IN THIS BLOCK
#include <stdio.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_Boards.h>
#include <AP_AHRS.h>
#include <AP_Baro.h>
#include <AP_Baro_Glitch.h>
#include <AP_Common.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <AP_GPS_Glitch.h>
#include <AP_Igniter.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>
#include <AP_Menu.h>
//#include <AP_Parachute.h>
#include <AP_Scheduler.h>
#include <GCS.h>
#include <GCS_MAVLink.h>
#include "arduhpr.h"
#include "config.h"
// NO SPACES IN THE ABOVE

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


//#include "Parameters.h"

/*---------------------------------------------------------------------------*/
// Barometer
/*---------------------------------------------------------------------------*/
#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
 #error Unrecognized CONFIG_BARO setting
#endif
Baro_Glitch baro_glitch(barometer);

static int32_t	baro_alt;				// barometer altitude in cm above home
static float	baro_climbrate;			// barometer climbrate in cm/s


/*---------------------------------------------------------------------------*/
// Compass
/*---------------------------------------------------------------------------*/
#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif


/*---------------------------------------------------------------------------*/
// Failsafe
/*---------------------------------------------------------------------------*/
static bool failsafe_active = false;
static uint32_t failsafe_last_heartbeat_usec;
static uint16_t failsafe_last_mainLoop_count;
static uint32_t failsafe_last_scheduler_usec;
static bool in_failsafe;


/*---------------------------------------------------------------------------*/
// Globals
/*---------------------------------------------------------------------------*/
struct {
	uint8_t home_is_set :		1;
//	uint8_t	mode :				HPR_MODE_INITIALIZING;
//	uint8_t	status :			HPR_STATUS_UNKNOWN;
	uint8_t telemetry_freq :	HPR_TELEM_SLOW_RATE;
	uint8_t	usb_connected : 	1;
	static struct Parameters params;
} global;


/*---------------------------------------------------------------------------*/
// Global Positioning System
/*---------------------------------------------------------------------------*/
static AP_GPS gps;
static GPS_Glitch gps_glitch(gps);
static Location gps_location;
//static struct Location current_location;
// We use atan2 and other trig techniques to calculate angles
// We need to scale the longitude up to make these calcs work
// to account for decreasing distance between lines of longitude away from the equator
//static float scale_long_up = 1;
// Sometimes we need to remove the scaling for distance calcs
//static float scale_long_down = 1;
static bool gps_is_home = false;


/*---------------------------------------------------------------------------*/
// Ground Control System
/*---------------------------------------------------------------------------*/
#if GCS_ON_UARTA == ENABLED
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS];
#else
static const uint8_t num_gcs = MAVLINK_COMM_NUM_BUFFERS - 1;
static GCS_MAVLINK gcs[MAVLINK_COMM_NUM_BUFFERS - 1];
#endif

static uint8_t gcs_command_acks;


/*---------------------------------------------------------------------------*/
// High Power Rocketry
/*---------------------------------------------------------------------------*/
//TODO: move this stuff into global
static uint8_t hpr_state;
static uint8_t hpr_status;
static uint32_t ignitionTimer = 0;

/*---------------------------------------------------------------------------*/
// Igniter
/*---------------------------------------------------------------------------*/
//static Igniter igniter;


/*---------------------------------------------------------------------------*/
// Inertial Sensor
/*---------------------------------------------------------------------------*/
static AP_InertialSensor ins;
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;


/*---------------------------------------------------------------------------*/
// Navigation
/*---------------------------------------------------------------------------*/
static AP_AHRS_DCM ahrs(ins, barometer, gps);
static AP_InertialNav nav(ahrs, barometer, gps_glitch, baro_glitch);
static float navDelta = 0.02;
static uint32_t navTimer = 0;


/*---------------------------------------------------------------------------*/
// Parachutes
/*---------------------------------------------------------------------------*/
//AP_Parachute parachute[NUM_PARACHUTES];


/*---------------------------------------------------------------------------*/
// Performance
/*---------------------------------------------------------------------------*/
static uint16_t perf_info_loop_count;
static uint32_t perf_info_max_time;
static uint16_t perf_info_long_running;

/*---------------------------------------------------------------------------*/
// Scheduler
/*---------------------------------------------------------------------------*/
static AP_Scheduler scheduler;
/*
  1    = 100hz
  2    = 50hz
  4    = 25hz
  10   = 10hz
  20   = 5hz
  33   = 3hz
  50   = 2hz
  100  = 1hz
  1000 = 0.1hz
*/
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
	{ barometer_accumulate,  2,    250 },
	{ compass_accumulate,    2,    420 },
	{ gcs_send_attitude,     2,    100 },
	{ gps_update,            2,    900 },

	{ hpr_update,            4,    100 }, // main function

//	{ battery_update,       10,    720 },
	{ compass_update,       10,    720 },
	{ barometer_update,     10,   1000 },

	{ cli_update,           50,    100 },
    { gcs_send_location,    50,    100 },

	// 1.0 hz
    { gcs_send_heartbeat,  100,    150 },

	// 0.2 hz
	{ gcs_send_raw_data,   250,   1800 },

//	{ perf_update,        1000,    200 },
};


/*---------------------------------------------------------------------------*/
void setup(void)
/*---------------------------------------------------------------------------*/
{
	global.usb_connected = hal.gpio->usb_connected();

// the APM2 has a MUX setup where the first serial port switches
// between USB and a TTL serial connection. When on USB we use
// SERIAL0_BAUD, but when connected as a TTL serial port we run it
// at SERIAL1_BAUD.
	//	hal.uartA->begin(map_baudrate(global.serial0_baud), 256, 256);
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    if (global.usb_connected) {
        //FIXME: hal.uartA->begin(map_baudrate(global.serial0_baud));
        hal.uartA->begin(map_baudrate(SERIAL0_BAUD), 256, 256);
    } else {
        //FIXME: hal.uartA->begin(map_baudrate(global.serial1_baud));
        hal.uartA->begin(map_baudrate(SERIAL1_BAUD), 256, 256);
    }
#endif

	hal.console->printf_P(PSTR("\n\n" FIRMWARE_STRING "\nFree RAM: %u\n"),
		hal.util->available_memory());

	failsafe_init();

//	config_init();

	//notify.init(true);
	gcs_init();

	// set up the sensors
	//battery_init();
	barometer_init();
	compass_init();
	gps_init();
	ins_init();

	// set up the navigation systems
	nav_init();
	ahrs_init();

	// set up parachutes and igniter
	parachute_init();
	igniter_init();

	// set up program logic
	hpr_init();

// run the timer a bit slower on APM2 to reduce the interrupt load on the CPU
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    hal.scheduler->set_timer_speed(500);
#endif

    hal.uartA->set_blocking_writes(false);
    hal.uartB->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);

    hal.console->print_P(PSTR("\nReady.\nHit 'Enter' 3 times to start CLI.\n\n"));
    gcs_send_text_P(SEVERITY_LOW, PSTR("System ready"));
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

/*---------------------------------------------------------------------------*/
void loop(void)
/*---------------------------------------------------------------------------*/
{
	uint32_t timer = hal.scheduler->micros();
	perf_info_update(timer - navTimer);
	ins.wait_for_sample();

	navDelta = (float)(timer - navTimer) / 1000000.f;
	navTimer = timer;

	ahrs_update();
	nav_update(navDelta);

    scheduler.tick();
    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - hal.scheduler->micros();
    scheduler.run(time_available);
}

AP_HAL_MAIN();
