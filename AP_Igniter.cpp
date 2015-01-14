#include "AP_Igniter.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo Igniter::var_info[] PROGMEM = {
		AP_GROUPEND
};

void Igniter::Init()
{
	//set_update_rate(_speed_hz);
}

void Igniter::set_update_rate( uint16_t speed_hz )
{
	_speed_hz = speed_hz;
}

// enable - starts allowing signals to be sent to motors
void Igniter::enable()
{
    // enable output channels
	/*
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]));
    hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_7]));
    */
}

// output_min - sends minimum values out to the motor and trim values to the servos
void Igniter::output_min()
{
    // send minimum value to each motor
	/*
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_1]), _servo1.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_2]), _servo2.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_3]), _servo3.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_4]), _servo4.radio_trim);
    hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[AP_MOTORS_MOT_7]), _rc_throttle.radio_min);
    */
}
