void perf_info_print(void)
{
    if (scheduler.debug()) {
        hal.console->printf_P(PSTR("PERF: %u/%u %lu\n"),
                            (unsigned)perf_info_long_running,
                            (unsigned)perf_info_loop_count,
                            (unsigned long)perf_info_max_time);
    }
    perf_info_reset();
}

// perf_info_reset - reset all records of loop time to zero
void perf_info_reset()
{
    perf_info_loop_count = 0;
    perf_info_max_time = 0;
    perf_info_long_running = 0;
}

// perf_info_update - check latest loop time vs min, max and overtime threshold
void perf_info_update(uint32_t time_in_micros)
{
    perf_info_loop_count++;
    if( time_in_micros > perf_info_max_time) {
        perf_info_max_time = time_in_micros;
    }
    if( time_in_micros > PERF_INFO_OVERTIME_THRESHOLD_MICROS ) {
        perf_info_long_running++;
    }
}
