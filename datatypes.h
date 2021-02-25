/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

// PACKET_START, LOG_MSG_TYPE, LENGTH, MESSAGE, PACKET_END

#define PACKET_START 0x0d
#define PACKET_END 0x0a

typedef enum {
	DEBUG = 0,
	HEADER,
	ESC,
	ESC_DELTA,
	GPS,
	GPS_DELTA,
	IMU,
	BMS,
	FREESK8
} LOG_MSG_TYPES;

typedef struct {
	uint16_t version; // Log file version identifies changes in struct(s)
	uint8_t multi_esc_mode;
	uint8_t log_frequency;
} LOG_HEADER;

typedef struct {
	time_t dt;

	uint16_t esc_id;
	uint16_t vin; // Div/10
	int16_t motor_temp; // Div/10
	int16_t mosfet_temp; // Div/10

	int16_t duty_cycle; // Div/1000
	int16_t motor_current; // Div/10
	int16_t battery_current; // Div/10
	uint16_t watt_hours; // Div/100

	uint16_t watt_hours_regen; // Div/100
	uint8_t fault;
	uint8_t not_used; //NOTE: padding
	uint32_t not_used2;  //NOTE: padding

	int32_t e_rpm;
	uint32_t e_distance;
} LOG_ESC;

typedef struct {
	uint8_t dt; // Up to 255 seconds elapsed
	//NOTE: 8 bit padding
	uint16_t esc_id;

	int8_t vin; // Div/10 // +-12.7 change
	//NOTE: 8 bit padding
	int8_t motor_temp; // Div/10 // +-12.7 change
	int8_t mosfet_temp; // Div/10 // +-12.7 change

	int16_t duty_cycle; // Div/1000
	int16_t motor_current; // Div/10

	int16_t battery_current; // Div/10
	int8_t watt_hours; // Div/100 // +-1.27 change
	int8_t watt_hours_regen; // Div/100 // +-1.27 change

	int16_t e_rpm; // +-32767
	int16_t e_distance; // +-32767
	uint8_t fault;
	//NOTE: 8 bit padding
} LOG_ESC_DELTA;

typedef struct {
	time_t dt;
	uint8_t satellites;
	//NOTE: 8 bit padding
	uint16_t altitude; // Div/10
	uint16_t speed; // Div/10
	//NOTE: 16 bit padding
	int32_t latitude; // Div/100000
	int32_t longitude; // Div/100000
} LOG_GPS;

typedef struct {
	uint8_t dt; // Up to 255 seconds elapsed
	int8_t satellites;
	int8_t altitude; // Div/10 // +-12.7 change
	int8_t speed; // Div/10 // +-12.7 change
	int16_t latitude; // Div/100000 // +-0.32767
	int16_t longitude; // Div/100000 // +-0.32767
} LOG_GPS_DELTA;

typedef enum {
	TIME_SYNC = 0, // When a time sync event occurs while logging
	USER_FLAG, // When the user wants to flag a moment while logging
} LOG_FREESK8_EVENTS;

typedef struct {
	uint8_t event_type;
	//NOTE: 7 bytes lost in data alignment
	uint8_t unused;
	uint16_t unused2;
	uint32_t unused3;

	int64_t event_data;
} LOG_FREESK8;

typedef enum {
	MOTE_PACKET_BATT_LEVEL = 0,
	MOTE_PACKET_BUTTONS,
	MOTE_PACKET_ALIVE,
	MOTE_PACKET_FILL_RX_BUFFER,
	MOTE_PACKET_FILL_RX_BUFFER_LONG,
	MOTE_PACKET_PROCESS_RX_BUFFER,
	MOTE_PACKET_PROCESS_SHORT_BUFFER,
	MOTE_PACKET_PAIRING_INFO
} MOTE_PACKET;

// Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING,
	COMM_GPD_SET_FSW,
	COMM_GPD_BUFFER_NOTIFY,
	COMM_GPD_BUFFER_SIZE_LEFT,
	COMM_GPD_FILL_BUFFER,
	COMM_GPD_OUTPUT_SAMPLE,
	COMM_GPD_SET_MODE,
	COMM_GPD_FILL_BUFFER_INT8,
	COMM_GPD_FILL_BUFFER_INT16,
	COMM_GPD_SET_BUFFER_INT_SCALE,
	COMM_GET_VALUES_SETUP,
	COMM_SET_MCCONF_TEMP,
	COMM_SET_MCCONF_TEMP_SETUP,
	COMM_GET_VALUES_SELECTIVE,
	COMM_GET_VALUES_SETUP_SELECTIVE,
	COMM_EXT_NRF_PRESENT,
	COMM_EXT_NRF_ESB_SET_CH_ADDR,
	COMM_EXT_NRF_ESB_SEND_DATA,
	COMM_EXT_NRF_ESB_RX_DATA,
	COMM_EXT_NRF_SET_ENABLED,
	COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
	COMM_DETECT_APPLY_ALL_FOC,
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
	COMM_ERASE_NEW_APP_ALL_CAN,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN,
	COMM_PING_CAN,
	COMM_APP_DISABLE_OUTPUT,
	COMM_TERMINAL_CMD_SYNC,
	COMM_GET_IMU_DATA
} COMM_PACKET_ID;

// Orientation data
typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;
	float integralFBx;
	float integralFBy;
	float integralFBz;
	float accMagP;
	int initialUpdateDone;
} ATTITUDE_INFO;

typedef struct {
	double v_in;
	double temp_mos;
	double temp_mos_1;
	double temp_mos_2;
	double temp_mos_3;
	double temp_motor;
	double current_motor;
	double current_in;
	double foc_id;
	double foc_iq;
	double rpm;
	double duty_now;
	double amp_hours;
	double amp_hours_charged;
	double watt_hours;
	double watt_hours_charged;
	int tachometer;
	int tachometer_abs;
	double position;
	uint8_t fault_code;
	int vesc_id;
	double vd;
	double vq;
} TELEMETRY_DATA;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR,
	FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE,
	FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE,
	FAULT_CODE_MCU_UNDER_VOLTAGE,
	FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET,
	FAULT_CODE_ENCODER_SPI,
	FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE,
	FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE,
	FAULT_CODE_FLASH_CORRUPTION,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2,
	FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3,
	FAULT_CODE_UNBALANCED_CURRENTS,
	FAULT_CODE_BRK,
	FAULT_CODE_RESOLVER_LOT,
	FAULT_CODE_RESOLVER_DOS,
	FAULT_CODE_RESOLVER_LOS
} mc_fault_code;

const char* mc_fault_to_string(mc_fault_code fault) {
	switch (fault) {
		case FAULT_CODE_NONE: return "FAULT_CODE_NONE"; break;
		case FAULT_CODE_OVER_VOLTAGE: return "FAULT_CODE_OVER_VOLTAGE"; break;
		case FAULT_CODE_UNDER_VOLTAGE: return "FAULT_CODE_UNDER_VOLTAGE"; break;
		case FAULT_CODE_DRV: return "FAULT_CODE_DRV"; break;
		case FAULT_CODE_ABS_OVER_CURRENT: return "FAULT_CODE_ABS_OVER_CURRENT"; break;
		case FAULT_CODE_OVER_TEMP_FET: return "FAULT_CODE_OVER_TEMP_FET"; break;
		case FAULT_CODE_OVER_TEMP_MOTOR: return "FAULT_CODE_OVER_TEMP_MOTOR"; break;
		case FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE"; break;
		case FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE: return "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE"; break;
		case FAULT_CODE_MCU_UNDER_VOLTAGE: return "FAULT_CODE_MCU_UNDER_VOLTAGE"; break;
		case FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET: return "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET"; break;
		case FAULT_CODE_ENCODER_SPI: return "FAULT_CODE_ENCODER_SPI"; break;
		case FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE"; break;
		case FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE: return "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE"; break;
		case FAULT_CODE_FLASH_CORRUPTION: return "FAULT_CODE_FLASH_CORRUPTION";
		case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1";
		case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2";
		case FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3: return "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3";
		case FAULT_CODE_UNBALANCED_CURRENTS: return "FAULT_CODE_UNBALANCED_CURRENTS";
		case FAULT_CODE_BRK: return "FAULT_CODE_BRK";
		default: return "FAULT_UNKNOWN"; break;
	}
}

#endif /* DATATYPES_H_ */
