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

#include <stdio.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_VRBRAIN.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>

#include <AP_Common.h>
#include <AP_Scheduler.h>

//#include "Parameters.h"
#include "arduhpr.h"


const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// Globals
//static Parameters global;

union {
    struct {
    	uint8_t  home_is_set : 1;
        uint8_t  usb_connected : 1;
    };
    uint32_t value;
} global_state;


// Program Loops
static AP_Scheduler scheduler;

// Program Scheduler
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
//    { ins_update,            1,   1000 }, //this is called by ahrs.update()
	{ gps_update,            2,    900 },
	{ barometer_accumulate,  2,    250 },
//	{ compass_accumulate,    2,    420 },

//	{ battery_update,       10,    720 },
	{ compass_update,       10,    720 },
	{ barometer_update,     10,   1000 },

//    { gcs_send_location,    20,    100 },

	// 1.0 hz
	//{ one_hz_loop,        50,   1000 },
	{ gcs_attitude,        100,    150 },
    { gcs_heartbeat,       100,    150 },

	// 0.2 hz
    { five_sec_loop,       250,   1800 },
	{ gcs_send_raw_data,   250,   1800 },

//	{ perf_update,        1000,     200 },
};

static float navDelta = 0.02;
static uint32_t navTimer;


/*---------------------------------------------------------------------------*/

void setup(void)
{
	global_state.usb_connected = hal.gpio->usb_connected();

	//	hal.uartA->begin(map_baudrate(global.serial0_baud), 256, 256);
#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // the APM2 has a MUX setup where the first serial port switches
    // between USB and a TTL serial connection. When on USB we use
    // SERIAL0_BAUD, but when connected as a TTL serial port we run it
    // at SERIAL1_BAUD.
    if (global_state.usb_connected) {
        //FIXME: hal.uartA->begin(map_baudrate(global.serial0_baud));
        hal.uartA->begin(map_baudrate(SERIAL0_BAUD), 256, 256);
    } else {
        //FIXME: hal.uartA->begin(map_baudrate(global.serial1_baud));
        hal.uartA->begin(map_baudrate(SERIAL1_BAUD), 256, 256);
    }
#endif

	hal.console->printf_P(PSTR("\n\n" FIRMWARE_STRING "\nFree RAM: %u\n"),
		hal.util->available_memory());

//	battery_init();
//	config_init();

	//notify.init(true);
	gcs_init();

	// setup the sensors
	barometer_init();
	compass_init();
	gps_init();
	ins_init();

	nav_init();
	ahrs_init();

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // run the timer a bit slower on APM2 to reduce the interrupt load on the CPU
    hal.scheduler->set_timer_speed(500);
#endif

//    hal.scheduler->register_timer_failsafe(failsafe_check, 1000);

    hal.uartA->set_blocking_writes(false);
    hal.uartB->set_blocking_writes(false);
    hal.uartC->set_blocking_writes(false);

    hal.console->print_P(PSTR("\nReady\n\n"));
    gcs_send_text_P(SEVERITY_LOW, PSTR("System ready"));
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}

void loop(void)
{
	ins_sample();
	uint32_t timer = hal.scheduler->micros();
	//perf_info_check_loop_time(timer - fast_loopTimer);

	navDelta = (float)(timer - navTimer) / 1000000.f;
	navTimer = timer;

	ahrs.update();
	nav_update(navDelta);

    scheduler.tick();

    //FIXME: timings are all off
//    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - hal.scheduler->micros();
//    scheduler.run(time_available);
    scheduler.run(MAIN_LOOP_MICROS);
}

static void one_hz_loop(void)
{
//    barometer_print();
    //compass_print();
//    gps_print();
//    ins_print();
//	hal.console->printf("->gcs_send_location\n");
//    gcs_send_location();
}

static void five_sec_loop(void)
{
//	gcs_send_text_P(SEVERITY_LOW, PSTR("5 second loop"));
//	if (global_state.usb_connected) {
//		gcs_send_text_P(SEVERITY_LOW, PSTR("YES USB"));
//	} else {
//		gcs_send_text_P(SEVERITY_LOW, PSTR("NO USB"));
//	}
	hal.console->printf_P(PSTR("alt = %u/%u\n"), gps_get_location().alt * 10UL, nav.get_altitude() * 10UL);
	return;
}


AP_HAL_MAIN();
