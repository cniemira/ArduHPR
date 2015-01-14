void ins_init(void)
{
	hal.console->printf_P(PSTR("Initializing INS..."));
    ins.init(AP_InertialSensor::COLD_START,
    		 ins_sample_rate);
    ins.init_accel();
    hal.console->println();
}

void ins_print(void)
{
    Vector3f accel = ins.get_accel();
    Vector3f gyro = ins.get_gyro();

    /*
	const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();

    hal.console->printf_P(PSTR("A_off: %4.2f, %4.2f, %4.2f\nA_scale: %4.2f, %4.2f, %4.2f\n"),
                    (float)accel_offsets.x,                           // Pitch
                    (float)accel_offsets.y,                           // Roll
                    (float)accel_offsets.z,                           // YAW
                    (float)accel_scale.x,                             // Pitch
                    (float)accel_scale.y,                             // Roll
                    (float)accel_scale.z);                            // YAW

    hal.console->printf_P(PSTR("G_off: %4.2f, %4.2f, %4.2f\n"),
                    (float)gyro_offsets.x,
                    (float)gyro_offsets.y,
                    (float)gyro_offsets.z);
    */

    hal.console->printf_P(PSTR("Accel X:%4.2f \t Y:%4.2f \t Z:%4.2f \t Gyro X:%4.2f \t Y:%4.2f \t Z:%4.2f\n"),
    								  accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
}
