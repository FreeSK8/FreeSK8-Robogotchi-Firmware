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
	uint16_t not_used2; //NOTE: padding

	uint16_t watt_hours_regen; // Div/100
	uint8_t fault;
	uint8_t not_used; //NOTE: padding
	uint32_t watt_hours; // Div/100

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
	int16_t altitude; // Div/10
	int16_t speed; // Div/10
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
	//NOTE: 7 bytes unused in data alignment
	uint8_t unused;
	uint16_t unused2;
	uint32_t unused3;

	int64_t event_data;
} LOG_FREESK8;


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
	COMM_GET_IMU_DATA,
	COMM_BM_CONNECT,
	COMM_BM_ERASE_FLASH_ALL,
	COMM_BM_WRITE_FLASH,
	COMM_BM_REBOOT,
	COMM_BM_DISCONNECT,
	COMM_BM_MAP_PINS_DEFAULT,
	COMM_BM_MAP_PINS_NRF5X,
	COMM_ERASE_BOOTLOADER,
	COMM_ERASE_BOOTLOADER_ALL_CAN,
	COMM_PLOT_INIT,
	COMM_PLOT_DATA,
	COMM_PLOT_ADD_GRAPH,
	COMM_PLOT_SET_GRAPH,
	COMM_GET_DECODED_BALANCE,
	COMM_BM_MEM_READ,
	COMM_WRITE_NEW_APP_DATA_LZO,
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO,
	COMM_BM_WRITE_FLASH_LZO,
	COMM_SET_CURRENT_REL,
	COMM_CAN_FWD_FRAME,
	COMM_SET_BATTERY_CUT,
	COMM_SET_BLE_NAME,
	COMM_SET_BLE_PIN,
	COMM_SET_CAN_MODE,
	COMM_GET_IMU_CALIBRATION,
	COMM_GET_MCCONF_TEMP, // Firmware 5.2 added

	// Custom configuration for hardware
	COMM_GET_CUSTOM_CONFIG_XML, // Firmware 5.2 added
	COMM_GET_CUSTOM_CONFIG, // Firmware 5.2 added
	COMM_GET_CUSTOM_CONFIG_DEFAULT, // Firmware 5.2 added
	COMM_SET_CUSTOM_CONFIG, // Firmware 5.2 added

	// BMS commands
	COMM_BMS_GET_VALUES, // Firmware 5.2 added
	COMM_BMS_SET_CHARGE_ALLOWED, // Firmware 5.2 added
	COMM_BMS_SET_BALANCE_OVERRIDE, // Firmware 5.2 added
	COMM_BMS_RESET_COUNTERS, // Firmware 5.2 added
	COMM_BMS_FORCE_BALANCE, // Firmware 5.2 added
	COMM_BMS_ZERO_CURRENT_OFFSET, // Firmware 5.2 added

	// FW updates commands for different HW types
	COMM_JUMP_TO_BOOTLOADER_HW, // Firmware 5.2 added
	COMM_ERASE_NEW_APP_HW, // Firmware 5.2 added
	COMM_WRITE_NEW_APP_DATA_HW, // Firmware 5.2 added
	COMM_ERASE_BOOTLOADER_HW, // Firmware 5.2 added
	COMM_JUMP_TO_BOOTLOADER_ALL_CAN_HW, // Firmware 5.2 added
	COMM_ERASE_NEW_APP_ALL_CAN_HW, // Firmware 5.2 added
	COMM_WRITE_NEW_APP_DATA_ALL_CAN_HW, // Firmware 5.2 added
	COMM_ERASE_BOOTLOADER_ALL_CAN_HW, // Firmware 5.2 added

	COMM_SET_ODOMETER, // Firmware 5.2 added
} COMM_PACKET_ID;


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

#endif /* DATATYPES_H_ */
