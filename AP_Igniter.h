#ifndef AP_IGNITER_H
#define AP_IGNITER_H

//#include <AP_HAL.h>
//#include <AP_Common.h>
#include <AP_Param.h>
//#include <AP_Relay.h>

class Igniter {
public:
	void Init(void);

	void enable(void);
	void output_min(void);
	void set_update_rate(uint16_t speed_hz);

	static const struct AP_Param::GroupInfo        var_info[];

private:
	int _speed_hz;
};

#endif
