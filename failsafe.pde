// failsafe attempts to detect main loop lockup and disarm the motors
void failsafe_init()
{
	hal.console->printf_P(PSTR("Initializing Failsafe..."));
	hal.scheduler->register_timer_failsafe(failsafe_update, 1000);
	hal.console->println();
}

void failsafe_heartbeat()
{
	failsafe_last_heartbeat_usec = hal.scheduler->micros();
}

// called by the scheduler
void failsafe_update()
{
    uint32_t tnow = hal.scheduler->micros();

  /*
    if (mainLoop_count != failsafe_last_mainLoop_count) {
        // the main loop is running, all is OK
        failsafe_last_mainLoop_count = mainLoop_count;
        failsafe_last_scheduler_usec = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!failsafe_active && tnow - failsafe_last_scheduler_usec > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors.
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors.armed()) {
            motors.output_min();
        }
        // log an error
        Log_Write_Error(ERROR_SUBSYSTEM_CPU,ERROR_CODE_FAILSAFE_OCCURRED);
    }

    if (failsafe_active && tnow - failsafe_last_scheduler_usec > 1000000) {
        // disarm motors every second
    	failsafe_last_scheduler_usec = tnow;
        if(motors.armed()) {
            motors.armed(false);
            motors.output();
        }
    }
    */
}
