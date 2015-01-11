#ifndef __ARDUHPR_CONFIG_H__
#define __ARDUHPR_CONFIG_H__

#define APM2_IGNORE_UART_MUX DISABLED
#define GCS_ON_UARTA DISABLED

#ifdef __AVR_ATmega1280__
#error ATmega1280 is not supported
#endif

#ifndef CONFIG_HAL_BOARD
#error CONFIG_HAL_BOARD must be defined to build ArduHPR
#endif

#define CONFIG_BARO HAL_BARO_DEFAULT
#define CONFIG_COMPASS HAL_COMPASS_DEFAULT
#define MAGNETOMETER ENABLED

#define CLI_ENABLED DISABLED

#define FS_BATT_DISABLED DISABLED

#ifndef FS_BATT_VOLTAGE_DEFAULT
 # define FS_BATT_VOLTAGE_DEFAULT       10.5f       // default battery voltage below which failsafe will be triggered
#endif

#ifndef FS_BATT_MAH_DEFAULT
 # define FS_BATT_MAH_DEFAULT             0         // default battery capacity (in mah) below which failsafe will be triggered
#endif

#ifdef HAL_SERIAL0_BAUD_DEFAULT
# define SERIAL0_BAUD HAL_SERIAL0_BAUD_DEFAULT
#endif

#if HAL_CPU_CLASS < HAL_CPU_CLASS_75 || CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
 // low power CPUs (APM1, APM2 and SITL)
 # define MAIN_LOOP_RATE    100
 # define MAIN_LOOP_SECONDS 0.01
 # define MAIN_LOOP_MICROS  10000
#else
 // high power CPUs (Flymaple, PX4, Pixhawk, VRBrain)
 # define MAIN_LOOP_RATE    400
 # define MAIN_LOOP_SECONDS 0.0025
 # define MAIN_LOOP_MICROS  2500
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

#ifndef SERIAL0_BAUD
 # define SERIAL0_BAUD                   115200
#endif
#ifndef SERIAL1_BAUD
 # define SERIAL1_BAUD                    57600
#endif
#ifndef SERIAL2_BAUD
 # define SERIAL2_BAUD                    57600
#endif
/*
  build a firmware version string.
  GIT_VERSION comes from Makefile builds
*/
#ifndef GIT_VERSION
#define FIRMWARE_STRING FIRMWARE_ID
#else
#define FIRMWARE_STRING FIRMWARE_ID " (" GIT_VERSION ")"
#endif

#endif // __ARDUHPR_CONFIG_H__
