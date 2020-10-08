#ifndef USER_CFG_H__
#define USER_CFG_H__

struct gotchi_configuration {
	uint16_t log_auto_stop_idle_time;
	float log_auto_stop_low_voltage;
	float log_auto_start_duty_cycle;
	uint8_t log_interval_hz;
	
	uint8_t multi_esc_mode; //0,1 = Single ESC Mode
							//2 = Dual ESC Mode
							//4 = Quad ESC Mode
	uint8_t multi_esc_ids[4];
	uint32_t gps_baud_rate;

	float alert_low_voltage;
	float alert_esc_temp;
	float alert_motor_temp;

	uint32_t cfg_version;
};

#endif
