/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "nrf_fstorage.h"
#include "nrf_drv_qspi.h"
#include "nrf_drv_spi.h"
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_fus.h"
#include "app_uart.h"
#include "gps_uart.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "bsp.h"
#include "security_manager.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define BIT(x) 1<<x

#ifdef NRF52840_XXAA
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "nrf_drv_power.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#endif

#include "packet.h"
#include "buffer.h"
#include "datatypes.h"

#include "crc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "command_interface.h"
#include <time.h>

#include "nrf_drv_twi.h"
const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);

#include "lwgps/lwgps.h"

////////////////////////////////////////
// GPS handle
////////////////////////////////////////
lwgps_t hgps;
static LOG_GPS log_message_gps;
static LOG_GPS_DELTA log_message_gps_delta;

////////////////////////////////////////
// Button input
////////////////////////////////////////
#include "nrf_gpio.h"
#define PIN_BUTTON 10
#define PIN_BUTTON2 9
#define isButtonPressed !nrf_gpio_pin_read(PIN_BUTTON)
#define isButton2Pressed !nrf_gpio_pin_read(PIN_BUTTON2)

////////////////////////////////////////
// ESC data
////////////////////////////////////////
static volatile TELEMETRY_DATA esc_telemetry;
static volatile int esc_rx_cnt = 0;
static LOG_ESC log_message_esc;
static LOG_ESC_DELTA log_message_esc_delta;

////////////////////////////////////////
// Display
////////////////////////////////////////
#define HAS_DISPLAY 1
#if HAS_DISPLAY
static volatile bool update_display = false; // Set to true to trigger I2C communication with OLED
static char display_text_buffer[32] = {0};
#include "SSD1306.h"
#include "Adafruit_GFX.h"
void i2c_oled_comm_handle(uint8_t hdl_address, uint8_t *hdl_buffer, size_t hdl_buffer_size)
{
	nrf_drv_twi_tx(&m_twi_master, hdl_address, hdl_buffer, hdl_buffer_size, false);
}
#endif

////////////////////////////////////////
// Time tracking
////////////////////////////////////////
#include "rtc.h"
volatile bool update_rtc = false; // Set to true to trigger I2C communication with RTC module
volatile bool rtc_time_has_sync = false; // Set to true when the RTC has been set by GPS or Mobile app
struct tm * tmTime;
time_t currentTime;
static char datetimestring[ 64 ] = { 0 };
static volatile bool log_file_active = false;
static volatile bool write_logdata_now = false;
static volatile bool gps_signal_locked = false;
static time_t time_esc_last_responded; // For triggering TX and RX pin swapping
static time_t time_gps_last_responded; // For detecting stale data

////////////////////////////////////////
// Fault tracking
////////////////////////////////////////
struct esc_fault {
	uint8_t fault_code;
	uint16_t fault_count;
	uint16_t esc_id;
	time_t dt_first_seen;
	time_t dt_last_seen;
};
static uint16_t fault_count = 0; // Count of messages containing fault codes
#define RECENT_FAULT_LIMIT 12 // Limit the fault code historical array
static uint8_t recent_fault_index = 0; // Track current index of historical array
static struct esc_fault recent_faults[RECENT_FAULT_LIMIT] = {0}; // Historical array of faults

////////////////////////////////////////
// Piezo
////////////////////////////////////////
#include "buzzer/nrf_pwm.h"
#include "buzzer/melody_notes.h"
#define PIN_PIEZO 8

int melody_notes=0;
int melody_wholenote = 0;
int melody_divider = 0;
int melody_note_duration = 0;
int melody_this_note = 0;
bool is_melody_playing = false;
bool is_melody_playing_pause = false;
uint32_t melody_next_note = 0;
int *melody;

void set_frequency_and_duty_cycle(uint32_t frequency, uint32_t duty_cycle_percent)
{
    nrf_pwm_set_max_value((16000000 + (frequency / 2)) / frequency);
    nrf_pwm_set_value(0, (16000000 / frequency) * duty_cycle_percent / 100);
}

uint32_t app_timer_ms(uint32_t ticks)
{
	// eg. (7 + 1) * 1000 / 32768
	//   = 8000 / 32768
	//   = 0.24414062
	float numerator = ((float)0/*TODO: APP_TIMER_PRESCALER*/ + 1.0f) * 1000.0f;
	float denominator = (float)APP_TIMER_CLOCK_FREQ;
	float ms_per_tick = numerator / denominator;

	uint32_t ms = ms_per_tick * ticks;

	return ms;
}

uint32_t millis(void)
{
	return app_timer_ms(app_timer_cnt_get());
}

void melody_play(int index, bool interrupt_melody)
{
	if (is_melody_playing && !interrupt_melody)
	{
		return;
	}
	switch (index)
	{
		case MELODY_TAKEONME:
			melody = (int*)&melody_takeonme;
			// sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
			// there are two values per note (pitch and duration), so for each note there are four bytes
			melody_notes=sizeof(melody_takeonme)/sizeof(melody_takeonme[0])/2;
			// this calculates the duration of a whole note in ms (60s/tempo)*4 beats
			melody_wholenote = (60000 * 4) / tempo_takeonme;
		break;
		case MELODY_ESC_FAULT:
			melody = (int*)&melody_esc_fault;
			melody_notes=sizeof(melody_esc_fault)/sizeof(melody_esc_fault[0])/2;
			melody_wholenote = (60000 * 4) / tempo_esc_fault;
		break;
		case MELODY_BLE_FAIL:
			melody = (int*)&melody_ble_fail;
			melody_notes=sizeof(melody_ble_fail)/sizeof(melody_ble_fail[0])/2;
			melody_wholenote = (60000 * 4) / tempo_ble_fail;
		break;
		case MELODY_NOKIA:
			melody = (int*)&melody_nokia;
			melody_notes=sizeof(melody_nokia)/sizeof(melody_nokia[0])/2;
			melody_wholenote = (60000 * 4) / tempo_nokia;
		break;
		case MELODY_BLE_SUCCESS:
			melody = (int*)&melody_ble_success;
			melody_notes=sizeof(melody_ble_success)/sizeof(melody_ble_success[0])/2;
			melody_wholenote = (60000 * 4) / tempo_ble_success;
		break;
		case MELODY_STORAGE_LIMIT:
			melody = (int*)&melody_storage_limit;
			melody_notes=sizeof(melody_storage_limit)/sizeof(melody_storage_limit[0])/2;
			melody_wholenote = (60000 * 4) / tempo_storage_limit;
		break;
		case MELODY_ESC_TEMP:
			melody = (int*)&melody_esc_temp;
			melody_notes=sizeof(melody_esc_temp)/sizeof(melody_esc_temp[0])/2;
			melody_wholenote = (60000 * 4) / tempo_esc_temp;
		break;
		case MELODY_MOTOR_TEMP:
			melody = (int*)&melody_motor_temp;
			melody_notes=sizeof(melody_motor_temp)/sizeof(melody_motor_temp[0])/2;
			melody_wholenote = (60000 * 4) / tempo_motor_temp;
		break;
		case MELODY_VOLTAGE_LOW:
			melody = (int*)&melody_voltage_low;
			melody_notes=sizeof(melody_voltage_low)/sizeof(melody_voltage_low[0])/2;
			melody_wholenote = (60000 * 4) / tempo_voltage_low;
		break;
		case MELODY_ASC:
			melody = (int*)&melody_ascending;
			melody_notes=sizeof(melody_ascending)/sizeof(melody_ascending[0])/2;
			melody_wholenote = (60000 * 4) / tempo_ascending;
		break;
		case MELODY_DESC:
			melody = (int*)&melody_descending;
			melody_notes=sizeof(melody_descending)/sizeof(melody_descending[0])/2;
			melody_wholenote = (60000 * 4) / tempo_descending;
		break;
		case MELODY_STARTUP:
			melody = (int*)&melody_startup;
			melody_notes=sizeof(melody_startup)/sizeof(melody_startup[0])/2;
			melody_wholenote = (60000 * 4) / tempo_startup;
		break;
		case MELODY_GPS_LOCK:
			melody = (int*)&melody_gps_locked;
			melody_notes=sizeof(melody_gps_locked)/sizeof(melody_gps_locked[0])/2;
			melody_wholenote = (60000 * 4) / tempo_gps_locked;
		break;
		case MELODY_GPS_LOST:
			melody = (int*)&melody_gps_lost;
			melody_notes=sizeof(melody_gps_lost)/sizeof(melody_gps_lost[0])/2;
			melody_wholenote = (60000 * 4) / tempo_gps_lost;
		break;
		default:
		//TODO: add default melody
		break;
	}

	melody_divider = 0;
	melody_note_duration = 0;
	melody_this_note = 0; // Play from the beginning
	is_melody_playing = true;
	melody_next_note = millis();
	nrf_pwm_set_enabled(true);
}

void melody_step(void)
{
	if (is_melody_playing)
	{
		// Check if we've reached the end or the user pressed button 2
		// to stop the melody
		if ((melody_this_note >= melody_notes *2) || isButton2Pressed)
		{
			melody_this_note = 0;
			is_melody_playing = false;
			nrf_pwm_set_enabled(false);
			NRF_LOG_INFO("end of melody");
			NRF_LOG_FLUSH();
			return;
		}

		// Check if it's time to play the next note
		uint32_t now = millis();
		if (now >= melody_next_note)
		{
			// calculates the duration of each note
			melody_divider = melody[melody_this_note + 1];
			if (melody_divider > 0) {
				// regular note, just proceed
				melody_note_duration = (melody_wholenote) / melody_divider;
			} else if (melody_divider < 0) {
				// dotted notes are represented with negative durations!!
				melody_note_duration = (melody_wholenote) / abs(melody_divider);
				melody_note_duration *= 1.5; // increases the duration in half for dotted notes
			}

			if (is_melody_playing_pause)
			{
				// stop the waveform generation before the next note.
				set_frequency_and_duty_cycle((uint32_t)(melody[melody_this_note]), 0);
				melody_next_note = now + (melody_note_duration * 0.1);
				is_melody_playing_pause = false; // Set to false so we play a note on the next step
				melody_this_note += 2; // Increment current note by 1 (note + duration)
			}
			else
			{
				// we only play the note for 90% of the duration, leaving 10% as a pause
				set_frequency_and_duty_cycle((uint32_t)(melody[melody_this_note]), 50);
				melody_next_note = now + (melody_note_duration * 0.9);
				is_melody_playing_pause = true; // Set to true so we pause on the next step
			}
		}
	}
	else
	{
		//TODO: This should not be necessary but a steady tone
		//      seems to happen in some situations.
		//TODO: This is not a patch and not the best solution
		nrf_pwm_set_enabled(false);
	}
}

void buzzer_init(void)
{
    nrf_pwm_config_t pwm_config =
		{.num_channels  = 1,
		.gpio_num       = {PIN_PIEZO},
		.ppi_channel    = {0,1,2,3,4,5},
		.gpiote_channel = {0,1},
		.mode           = PWM_MODE_BUZZER_255};

    nrf_pwm_init(&pwm_config);

	while(false)
	{
		// iterate over the notes of the melody. 
		// Remember, the array is twice the number of notes (notes + durations)
		for (melody_this_note = 0; melody_this_note < melody_notes * 2; melody_this_note = melody_this_note + 2) {

			// calculates the duration of each note
			melody_divider = melody[melody_this_note + 1];
			if (melody_divider > 0) {
			// regular note, just proceed
			melody_note_duration = (melody_wholenote) / melody_divider;
			} else if (melody_divider < 0) {
			// dotted notes are represented with negative durations!!
			melody_note_duration = (melody_wholenote) / abs(melody_divider);
			melody_note_duration *= 1.5; // increases the duration in half for dotted notes
			}

			// we only play the note for 90% of the duration, leaving 10% as a pause
			set_frequency_and_duty_cycle((uint32_t)(melody[melody_this_note]), 50); //tone(buzzer, melody[melody_this_note], noteDuration*0.9);

			// Wait for the specief duration before playing the next note.
			nrf_delay_ms(melody_note_duration*0.9); //delay(noteDuration);

			// stop the waveform generation before the next note.
			set_frequency_and_duty_cycle((uint32_t)(melody[melody_this_note]), 0);//noTone(buzzer);
			nrf_delay_ms(melody_note_duration*0.1); //delay(noteDuration);

			nrf_gpio_pin_toggle(13); //LED
		}
	}

}

//TODO: replace uses of beep_speaker_blocking
void beep_speaker_blocking(int duration_ms, int duty_haha_duty)
{
	nrf_pwm_set_enabled(true);
	set_frequency_and_duty_cycle((uint32_t)3100, 50);
	nrf_delay_ms(duration_ms);
	set_frequency_and_duty_cycle((uint32_t)3100, 0);
	nrf_pwm_set_enabled(false);
}

////////////////////////////////////////
//LITTLEFS
////////////////////////////////////////
bool sync_in_progress = false;
time_t lastTimeBoardMoved = 0;
int log_file_stop();
void log_file_start();

uint32_t boot_count = 0;
#include "lfs.h"

int qspi_read(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, void *buffer, lfs_size_t size)
{
  uint32_t addr = c->block_size * block + off;
  uint32_t err_code = nrf_drv_qspi_read((uint8_t*)buffer, size, addr);
  return err_code;
}

int qspi_prog(const struct lfs_config *c, lfs_block_t block,
        lfs_off_t off, const void *buffer, lfs_size_t size)
{
  uint32_t addr = c->block_size * block + off;
  uint32_t err_code = nrf_drv_qspi_write((uint8_t*)buffer, size, addr);
  APP_ERROR_CHECK(err_code);
  return err_code;
} 

int qspi_erase(const struct lfs_config *c, lfs_block_t block)
{
  uint32_t addr = c->block_size * block;
  uint32_t err_code = nrf_drv_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, addr);
  APP_ERROR_CHECK(err_code);
  return err_code;
}

int qspi_sync(const struct lfs_config *c)
{
  return LFS_ERR_OK;
}

// variables used by the filesystem
uint16_t lfs_file_count = 0;
static lfs_t lfs;
static lfs_file_t file;
static uint8_t lfs_read_buf[256]; // Must be cache_size
static uint8_t lfs_prog_buf[256]; // Must be cache_size
static uint8_t lfs_lookahead_buf[16];	// 128/8=16
static uint8_t lfs_file_buf[256]; // Must be cache size
static struct lfs_file_config lfs_file_config;

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = &qspi_read,
    .prog  = &qspi_prog,
    .erase = &qspi_erase,
    .sync  = &qspi_sync,

    // block device configuration
    .read_size = 4,
    .prog_size = 4,
    .block_size = 4096,
    .block_count = 8192, //4096 bytes/block @ 256Mbit (33554432 bytes) = 8192 blocks
    .cache_size = 256,
    .lookahead_size = 16,
    .block_cycles = 500,

	.read_buffer = lfs_read_buf,
	.prog_buffer = lfs_prog_buf,
	.lookahead_buffer = lfs_lookahead_buf,
};

////////////////////////////////////////
// User configuration
////////////////////////////////////////
void user_cfg_set(void);
void user_cfg_get(void);
#include "user_cfg.h"
static int multiESCIndex = 0;
const struct gotchi_configuration gotchi_cfg_default = {
	.log_auto_stop_idle_time = 300,
	.log_auto_stop_low_voltage = 20.0,
	.log_auto_start_erpm = 2000,
	.log_interval_hz = 1,
	.log_auto_erase_when_full = 0,

	.multi_esc_mode = 0,
	.multi_esc_ids = {0,0,0,0},

	.gps_baud_rate = NRF_UARTE_BAUDRATE_9600,

	.alert_low_voltage = 0.0,
	.alert_esc_temp = 0.0,
	.alert_motor_temp = 0.0,
	.alert_storage_at_capacity = 0,

	.cfg_version = 3 // Expected configuration version, increment with changes to struct
};

struct gotchi_configuration gotchi_cfg_user = {
	.log_auto_stop_idle_time = 300,
	.log_auto_stop_low_voltage = 20.0,
	.log_auto_start_erpm = 2000,
	.log_interval_hz = 1,
	.log_auto_erase_when_full = 0,

	.multi_esc_mode = 0,
	.multi_esc_ids = {0,0,0,0},

	.gps_baud_rate = NRF_UARTE_BAUDRATE_9600,

	.alert_low_voltage = 0.0,
	.alert_esc_temp = 0.0,
	.alert_motor_temp = 0.0,
	.alert_storage_at_capacity = 0,

	.cfg_version = 0
};

////////////////////////////////////////
// Tracking free space
////////////////////////////////////////
static volatile int lfs_percent_free = 0;
uint8_t lfs_free_space_check(void)
{
	int lfs_blocks_allocated = lfs_fs_size(&lfs);
	lfs_percent_free =  100 - (int)((double)lfs_blocks_allocated / (double)cfg.block_count * 100);
	NRF_LOG_INFO("FS Blocks Allocated: %ld", lfs_blocks_allocated);
	NRF_LOG_INFO("FS BlockCount: %d Percentage Free: %d", cfg.block_count, lfs_percent_free);
	NRF_LOG_FLUSH();

#if HAS_DISPLAY
	Adafruit_GFX_setCursor(88,16);
	snprintf(display_text_buffer,4,"%02d%%", lfs_percent_free);
	Adafruit_GFX_print(display_text_buffer);
	update_display = true;
#endif

	if (gotchi_cfg_user.alert_storage_at_capacity != 0 && 100 - lfs_percent_free >= gotchi_cfg_user.alert_storage_at_capacity)
	{
		melody_play(MELODY_STORAGE_LIMIT, false); // Play storage at capacity alert, do not interrupt
	}

	return lfs_percent_free;
}

////////////////////////////////////////////////////////////
// Everything that once was but has been heavily modified
////////////////////////////////////////////////////////////

#ifndef MODULE_BUILTIN
#define MODULE_BUILTIN					0
#endif

#ifndef MODULE_FREESK8
#define MODULE_FREESK8					1
#endif


#define APP_BLE_CONN_CFG_TAG			1										   /**< A tag identifying the SoftDevice BLE configuration. */

#ifdef NRF52840_XXAA
#if MODULE_BUILTIN
#define DEVICE_NAME					 "FreeSK8 Receiver"
#elif defined(MODULE_FREESK8)
#define DEVICE_NAME   					"FreeSK8 Robogotchi"
#else
#define DEVICE_NAME					 "VESC 52840 UART"
#endif
#else
#if MODULE_BUILTIN
#define DEVICE_NAME					 "VESC 52832 BUILTIN"
#else
#define DEVICE_NAME					 "VESC 52832 UART"
#endif
#endif

#define FUS_SERVICE_UUID_TYPE		   BLE_UUID_TYPE_VENDOR_BEGIN				  /**< UUID type for the FreeSK8 UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO		   3										   /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL				64										  /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION				18000									   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL			   MSEC_TO_UNITS(7.5, UNIT_1_25_MS)			 /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL			   MSEC_TO_UNITS(35, UNIT_1_25_MS)			 /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY				   0										   /**< Slave latency. */
#define CONN_SUP_TIMEOUT				MSEC_TO_UNITS(4000, UNIT_10_MS)			 /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)					   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)					  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT	3										   /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF					   0xDEADBEEF								  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#ifdef NRF52840_XXAA
#define UART_TX_BUF_SIZE				1024
#define UART_RX_BUF_SIZE				1024
#else
#error Gurl, whatchu doing?
#endif

#define PACKET_VESC						0
#define PACKET_BLE						1

#ifdef NRF52840_XXAA
#if MODULE_BUILTIN
#define UART_RX							26
#define UART_TX							25
#define LED_PIN							27
#elif defined(MODULE_FREESK8)
#define UART_RX							6
#define UART_TX							7
#define LED_PIN							5
#else
#error Define MODULE_FREESK8 or MODULE_BUILTIN
#endif
#else
#error Firmware is deisnged for 52840_XXAA
#endif


// Private variables
APP_TIMER_DEF(m_packet_timer);
APP_TIMER_DEF(m_logging_timer);
APP_TIMER_DEF(m_telemetry_timer);

BLE_FUS_DEF(m_fus, NRF_SDH_BLE_TOTAL_LINK_COUNT);								   /**< BLE FUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);														   /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);															 /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);												 /**< Advertising module instance. */

static uint16_t   m_conn_handle		  = BLE_CONN_HANDLE_INVALID;				 /**< Handle of the current connection. */
uint16_t   m_ble_fus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;			/**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]		  =										  /**< Universally unique service identifier. */
{
	{BLE_UUID_FUS_SERVICE, FUS_SERVICE_UUID_TYPE}
};
static volatile bool					m_is_enabled = true;
static volatile bool					m_uart_error = false;
static volatile int						m_other_comm_disable_time = 0;

app_uart_comm_params_t m_uart_comm_params =
{
		.rx_pin_no	= UART_RX,
		.tx_pin_no	= UART_TX,
		.rts_pin_no   = 0,
		.cts_pin_no   = 0,
		.flow_control = APP_UART_FLOW_CONTROL_DISABLED,
		.use_parity   = false,
#if defined (UART_PRESENT)
		.baud_rate	= NRF_UART_BAUDRATE_115200
#else
		.baud_rate	= NRF_UARTE_BAUDRATE_115200
#endif
};
gps_uart_comm_params_t m_gpsuart_comm_params =
{
		.rx_pin_no	= 2,
		.tx_pin_no	= 3,
		.rts_pin_no   = 0,
		.cts_pin_no   = 0,
		.flow_control = GPS_UART_FLOW_CONTROL_DISABLED,
		.use_parity   = false,
		.baud_rate	= NRF_UARTE_BAUDRATE_4800

};
// Functions
void ble_printf(const char* format, ...);

#ifdef NRF52840_XXAA
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
		app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN	   NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN	   NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT	  NRF_DRV_USBD_EPOUT1

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
		cdc_acm_user_ev_handler,
		CDC_ACM_COMM_INTERFACE,
		CDC_ACM_DATA_INTERFACE,
		CDC_ACM_COMM_EPIN,
		CDC_ACM_DATA_EPIN,
		CDC_ACM_DATA_EPOUT,
		APP_USBD_CDC_COMM_PROTOCOL_NONE
);

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event) {
	switch (event) {
	case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
//		nrf_gpio_pin_set(LED_PIN);
		// Setup first transfer
		char rx;
		app_usbd_cdc_acm_read(&m_app_cdc_acm, &rx, 1);
		break;
	}
	case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
//		nrf_gpio_pin_clear(LED_PIN);
		break;
	case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
		break;
	case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
		ret_code_t ret;
		char rx;

		do {
			ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, &rx, 1);
		} while (ret == NRF_SUCCESS);
		break;
	}
	default:
		break;
	}
}

static void usbd_user_ev_handler(app_usbd_event_type_t event) {
	switch (event) {
	case APP_USBD_EVT_DRV_SUSPEND:
		break;
	case APP_USBD_EVT_DRV_RESUME:
		break;
	case APP_USBD_EVT_STARTED:
		break;
	case APP_USBD_EVT_STOPPED:
		app_usbd_disable();
		break;
	case APP_USBD_EVT_POWER_DETECTED:
		if (!nrf_drv_usbd_is_enabled()) {
			app_usbd_enable();
		}
		break;
	case APP_USBD_EVT_POWER_REMOVED:
		app_usbd_stop();
		break;
	case APP_USBD_EVT_POWER_READY:
		app_usbd_start();
		break;
	default:
		break;
	}
}
#endif

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *		  how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num	Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
	app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

#include "peer_manager.h"
#include "peer_manager_handler.h"
static void advertising_start(bool erase_bonds);
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
#define SEC_PARAM_BOND              1                                   /**< Perform bonding. */
#define SEC_PARAM_MITM              1                                   /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC              0                                   /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS          0                                   /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES   BLE_GAP_IO_CAPS_DISPLAY_ONLY                /**< No I/O capabilities. */
#define SEC_PARAM_OOB               0                                   /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE      7                                   /**< Minimum encryption key size in octets. */
#define SEC_PARAM_MAX_KEY_SIZE      16                                  /**< Maximum encryption key size in octets. */

#define PASSKEY_LENGTH              6                                   /**< Length of pass-key received by the stack for display. */

static void passkey_init(uint32_t ble_pin)
{
	char passkey[6];
	itoa(ble_pin, passkey, 10);
	ble_opt_t ble_opt;
	ble_opt.gap_opt.passkey.p_passkey = (uint8_t*)&passkey[0];
	(void)sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &ble_opt);
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static volatile bool is_connection_secure = false;
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            pm_conn_sec_status_t conn_sec_status;

            // Check if the link is authenticated (meaning at least MITM).
            err_code = pm_conn_sec_status_get(p_evt->conn_handle, &conn_sec_status);
            APP_ERROR_CHECK(err_code);

            if (conn_sec_status.mitm_protected)
            {
                NRF_LOG_INFO("Link secured. Role: %d. conn_handle: %d, Procedure: %d",
                             ble_conn_state_role(p_evt->conn_handle),
                             p_evt->conn_handle,
                             p_evt->params.conn_sec_succeeded.procedure);
				is_connection_secure = true;
				melody_play(MELODY_BLE_SUCCESS, true); // Play BLE Success sound
				// Notify user connection successful
				Adafruit_GFX_setCursor(64, 0);
				Adafruit_GFX_print("BLE OK");
				update_display = true;
            }
            else
            {
                // The peer did not use MITM, disconnect.
                NRF_LOG_INFO("Collector did not use MITM, disconnecting");
                err_code = pm_peer_id_get(m_conn_handle, &m_peer_to_be_deleted);
                APP_ERROR_CHECK(err_code);
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_CONN_SEC_FAILED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			// Notify user connection failed
			Adafruit_GFX_setCursor(64, 0);
			Adafruit_GFX_print("BLEPIN");
			update_display = true;
			melody_play(MELODY_BLE_FAIL, false); // Play BLE Failed sound. Do not interrupt (may happen repeatedly)
            break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
	uint32_t				err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
	BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&sec_mode);

	err_code = sd_ble_gap_device_name_set(&sec_mode,
			(const uint8_t *) DEVICE_NAME,
			strlen(DEVICE_NAME));
	APP_ERROR_CHECK(err_code);

	memset(&gap_conn_params, 0, sizeof(gap_conn_params));

	gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
	gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
	gap_conn_params.slave_latency	 = SLAVE_LATENCY;
	gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

	err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
		sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 8);
    }
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *		  application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
	APP_ERROR_HANDLER(nrf_error);
}

void ble_send_logbuffer(unsigned char *data, unsigned int len) {
	if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint32_t err_code = NRF_SUCCESS;
		int ind = 0;

		while (len > 0) {
			if (m_conn_handle == BLE_CONN_HANDLE_INVALID ||
					(err_code != NRF_ERROR_BUSY && err_code != NRF_SUCCESS && err_code != NRF_ERROR_RESOURCES)) {
				break;
			}

			uint16_t max_len = m_ble_fus_max_data_len;
			uint16_t tmp_len = len > max_len ? max_len : len;
			err_code = ble_fus_logdata_send(&m_fus, data + ind, &tmp_len, m_conn_handle);

			if (err_code != NRF_ERROR_RESOURCES && err_code != NRF_ERROR_BUSY) {
				len -= tmp_len;
				ind += tmp_len;
			}
		}
	}
}

static void fus_data_handler(ble_fus_evt_t * p_evt) {
	if (p_evt->type == BLE_FUS_EVT_RX_DATA) {
		for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++) {
			//NRF_LOG_INFO("tx_data[%03d] = 0x%02x", i, p_evt->params.rx_data.p_data[i]);
			//NRF_LOG_FLUSH();
			packet_process_byte(p_evt->params.rx_data.p_data[i], PACKET_BLE);
		}
	}
	else if (p_evt->type == BLE_FUS_EVT_RXLOG_DATA) {
		for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++) {
			//NRF_LOG_INFO("rx_data[%d] = %c", i, p_evt->params.rx_data.p_data[i]);
			//NRF_LOG_FLUSH();
			command_interface_process_byte(p_evt->params.rx_data.p_data[i]);
		}
	}

}

static void services_init(void) {
	uint32_t		   err_code;
	ble_fus_init_t	 fus_init;
	nrf_ble_qwr_init_t qwr_init = {0};

	// Initialize Queued Write Module.
	qwr_init.error_handler = nrf_qwr_error_handler;

	err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);

	// Initialize FUS.
	memset(&fus_init, 0, sizeof(fus_init));

	fus_init.data_handler = fus_data_handler;

	err_code = ble_fus_init(&m_fus, &fus_init);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
	APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
	uint32_t			   err_code;
	ble_conn_params_init_t cp_init;

	memset(&cp_init, 0, sizeof(cp_init));

	cp_init.p_conn_params				  = NULL;
	cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
	cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
	cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
	cp_init.start_on_notify_cccd_handle	= BLE_GATT_HANDLE_INVALID;
	cp_init.disconnect_on_fail			 = true;
	cp_init.evt_handler					= NULL;
	cp_init.error_handler				  = conn_params_error_handler;

	err_code = ble_conn_params_init(&cp_init);
	APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
	switch (ble_adv_evt) {
	case BLE_ADV_EVT_FAST:
//		bsp_indication_set(BSP_INDICATE_ADVERTISING);
		break;
	case BLE_ADV_EVT_IDLE:
//		sleep_mode_enter();
		advertising_start(false);
		break;
	default:
		break;
	}
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) {
	ret_code_t err_code;

	pm_handler_secure_on_connection(p_ble_evt);

	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		nrf_gpio_pin_set(LED_PIN);
		NRF_LOG_INFO("Connected");
		m_peer_to_be_deleted = PM_PEER_ID_INVALID;
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
		APP_ERROR_CHECK(err_code);
		sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, 8);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		nrf_gpio_pin_clear(LED_PIN);
		is_connection_secure = false;
		NRF_LOG_INFO("Disconnected");
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
		// Check if the last connected peer had not used MITM, if so, delete its bond information.
		if (m_peer_to_be_deleted != PM_PEER_ID_INVALID)
		{
			err_code = pm_peer_delete(m_peer_to_be_deleted);
			APP_ERROR_CHECK(err_code);
			NRF_LOG_DEBUG("Collector's bond deleted");
			m_peer_to_be_deleted = PM_PEER_ID_INVALID;
		}
		break;

	case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
		ble_gap_phys_t const phys =
		{
			.rx_phys = BLE_GAP_PHY_AUTO,
			.tx_phys = BLE_GAP_PHY_AUTO,
		};
		sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
	} break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
	case BLE_GAP_EVT_PASSKEY_DISPLAY:
	{
		char passkey[PASSKEY_LENGTH + 1];
		memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, PASSKEY_LENGTH);
		passkey[PASSKEY_LENGTH] = 0;

		NRF_LOG_INFO("Passkey: %s", nrf_log_push(passkey));
	} break;

	case BLE_GAP_EVT_AUTH_KEY_REQUEST:
		NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
	break;

	case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
		NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
	break;

	case BLE_GAP_EVT_AUTH_STATUS:
		NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
			p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
			p_ble_evt->evt.gap_evt.params.auth_status.bonded,
			p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
			*((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
			*((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
	break;
	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		// No system attributes have been stored.
		sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
		break;

	case BLE_GATTC_EVT_TIMEOUT:
		// Disconnect on GATT Client timeout event.
		sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		break;

	case BLE_GATTS_EVT_TIMEOUT:
		// Disconnect on GATT Server timeout event.
		sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
		break;
	case BLE_GATTS_EVT_WRITE:
	case BLE_GATTS_EVT_HVN_TX_COMPLETE:

	break;
	default:
		NRF_LOG_INFO("No handler programmed for: p_ble_evt->header.evt_id 0x%02x", p_ble_evt->header.evt_id);
		NRF_LOG_FLUSH();
		// No implementation needed.
		break;
	}
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
	nrf_sdh_enable_request();

	// Configure the BLE stack using the default settings.
	// Fetch the start address of the application RAM.
	uint32_t ram_start = 0;
	nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);

	// Enable BLE stack.
	nrf_sdh_ble_enable(&ram_start);

	// Register a handler for BLE events.
	NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt) {
	if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
		m_ble_fus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
		NRF_LOG_INFO("MTU Updated to %d bytes", m_ble_fus_max_data_len);
		NRF_LOG_FLUSH();
//		ble_printf("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
	}
}

void gatt_init(void) {
	ret_code_t err_code;

	err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
	APP_ERROR_CHECK(err_code);
}

void uart_event_handle(app_uart_evt_t * p_event) {
	switch (p_event->evt_type) {
	case APP_UART_DATA_READY: {
//		uint8_t byte;
//		while (app_uart_get(&byte) == NRF_SUCCESS) {
//			packet_process_byte(byte, PACKET_VESC);
//		}
	} break;

	case APP_UART_COMMUNICATION_ERROR:
//		m_uart_error = true;
		break;

	case APP_UART_FIFO_ERROR:
//		m_uart_error = true;
		break;

	default:
		break;
	}
}
void uart_gps_event_handle(gps_uart_evt_t * p_event) {
	switch (p_event->evt_type) {
	case GPS_UART_DATA_READY: {
	} break;
	case GPS_UART_COMMUNICATION_ERROR:
		break;
	case GPS_UART_FIFO_ERROR:
		break;
	default:
		break;
	}
}

static void uart_init(void) {
	uint32_t err_code;
	APP_UART_FIFO_INIT(&m_uart_comm_params,
			UART_RX_BUF_SIZE,
			UART_TX_BUF_SIZE,
			uart_event_handle,
			APP_IRQ_PRIORITY_LOW,
			err_code);
	APP_ERROR_CHECK(err_code);
}

static void uart_swap_pins(void) {

	app_uart_close();

	uint32_t old_rx_pin = m_uart_comm_params.rx_pin_no;
	m_uart_comm_params.rx_pin_no = m_uart_comm_params.tx_pin_no;
	m_uart_comm_params.tx_pin_no = old_rx_pin;

	packet_reset(PACKET_VESC);
	uart_init();
}

static void gps_init(void) {
	uint32_t err_code;
	m_gpsuart_comm_params.baud_rate = gotchi_cfg_user.gps_baud_rate;
	GPS_UART_FIFO_INIT(&m_gpsuart_comm_params,
			UART_RX_BUF_SIZE,
			64,
			uart_gps_event_handle,
			APP_IRQ_PRIORITY_LOW,
			err_code);
	APP_ERROR_CHECK(err_code);
}

static void advertising_init(void) {
	uint32_t err_code;
	ble_advertising_init_t init;

	memset(&init, 0, sizeof(init));

	init.advdata.name_type		  = BLE_ADVDATA_FULL_NAME;
	init.advdata.include_appearance = false;
	init.advdata.flags			  = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

	init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
	init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

	init.config.ble_adv_fast_enabled  = true;
	init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
	init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
	init.evt_handler = on_adv_evt;

	err_code = ble_advertising_init(&m_advertising, &init);
	APP_ERROR_CHECK(err_code);

	ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

static void uart_send_buffer(unsigned char *data, unsigned int len) {
	for (int i = 0;i < len;i++) {
		app_uart_put(data[i]);
	}
}

static void ble_send_buffer(unsigned char *data, unsigned int len) {
	if (m_conn_handle != BLE_CONN_HANDLE_INVALID) {
		uint32_t err_code = NRF_SUCCESS;
		int ind = 0;

		while (len > 0) {
			if (m_conn_handle == BLE_CONN_HANDLE_INVALID ||
					(err_code != NRF_ERROR_BUSY && err_code != NRF_SUCCESS && err_code != NRF_ERROR_RESOURCES)) {
				break;
			}

			uint16_t max_len = m_ble_fus_max_data_len;
			uint16_t tmp_len = len > max_len ? max_len : len;
			err_code = ble_fus_data_send(&m_fus, data + ind, &tmp_len, m_conn_handle);

			if (err_code != NRF_ERROR_RESOURCES && err_code != NRF_ERROR_BUSY) {
				len -= tmp_len;
				ind += tmp_len;
			}
		}
	}
}

static void process_packet_ble(unsigned char *data, unsigned int len) {
	if (data[0] == COMM_ERASE_NEW_APP ||
			data[0] == COMM_WRITE_NEW_APP_DATA ||
			data[0] == COMM_ERASE_NEW_APP_ALL_CAN ||
			data[0] == COMM_WRITE_NEW_APP_DATA_ALL_CAN) {
		m_other_comm_disable_time = 5000;
	}

	if(!is_connection_secure) {
		NRF_LOG_INFO("Connection is not yet secure. Sorry. I can't help you at this time.");
		NRF_LOG_FLUSH();
		//TODO: send response to device that ble is not secure? or check on device if connection has been secured?
		return;
	}

	CRITICAL_REGION_ENTER();
	packet_send_packet(data, len, PACKET_VESC);
	CRITICAL_REGION_EXIT();
}

void update_status_packet(char * buffer);
void send_status_packet()
{
	if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		static unsigned char response_buffer[64];
		update_status_packet((char *)response_buffer);
		ble_send_logbuffer(response_buffer, strlen((const char *)response_buffer));
	}
}

static bool is_fault_new(uint8_t p_fault_code, int p_esc_id)
{
	for(int i=0; i<recent_fault_index;++i)
	{
		if(p_fault_code == recent_faults[i].fault_code && p_esc_id == recent_faults[i].esc_id)
		{
			return false;
		}
	}
	return true;
}

static void update_fault(uint8_t p_fault_code, int p_esc_id)
{
	for(int i=0; i<recent_fault_index;++i)
	{
		if(p_fault_code == recent_faults[i].fault_code && p_esc_id == recent_faults[i].esc_id)
		{
			++recent_faults[i].fault_count;
			recent_faults[i].dt_last_seen = currentTime;
			return;
		}
	}
}

#define FW5_PACKET_LENGTH 73
static void process_packet_vesc(unsigned char *data, unsigned int len) {
	// Additionally comparing with FW5_PACKET_LENGTH to safeguard against non-esc communication
	if (data[0] == COMM_GET_VALUES && len == FW5_PACKET_LENGTH){
		int32_t index = 1;
		esc_telemetry.temp_mos = buffer_get_float16(data,10.0,&index);
		esc_telemetry.temp_motor = buffer_get_float16(data,10.0,&index);
		esc_telemetry.current_motor = buffer_get_float32(data,100.0,&index);
		esc_telemetry.current_in = buffer_get_float32(data,100.0,&index);
		esc_telemetry.foc_id = buffer_get_float32(data,100.0,&index);
		esc_telemetry.foc_iq = buffer_get_float32(data,100.0,&index);
		esc_telemetry.duty_now = buffer_get_float16(data,1000.0,&index);
		esc_telemetry.rpm = buffer_get_float32(data,1.0,&index);
		esc_telemetry.v_in = buffer_get_float16(data,10.0,&index);
		esc_telemetry.amp_hours = buffer_get_float32(data,10000.0,&index);
		esc_telemetry.amp_hours_charged = buffer_get_float32(data,10000.0,&index);
		esc_telemetry.watt_hours = buffer_get_float32(data,10000.0,&index);
		esc_telemetry.watt_hours_charged = buffer_get_float32(data,10000.0,&index);
		esc_telemetry.tachometer = buffer_get_int32(data,&index);
		esc_telemetry.tachometer_abs = buffer_get_int32(data,&index);
		esc_telemetry.fault_code = data[index++];
		esc_telemetry.position = buffer_get_float32(data,10.0,&index);
		esc_telemetry.vesc_id = data[index++];
		esc_telemetry.temp_mos_1 = buffer_get_float16(data,10.0,&index);
		esc_telemetry.temp_mos_2 = buffer_get_float16(data,10.0,&index);
		esc_telemetry.temp_mos_3 = buffer_get_float16(data,10.0,&index);
		esc_telemetry.vd = buffer_get_float32(data,100.0,&index);
		esc_telemetry.vq = buffer_get_float32(data,100.0,&index);

		//TODO: If there is a fault we would like to write_logdata_now but still need a sensible rate limit to protect storage
		// Write data to log file if log_file_active and we have been flagged to write this iteration
		if (log_file_active && write_logdata_now)
		{
			// Clear write now flag
			write_logdata_now = false;

			// If we have not yet logged a full ESC message do so now
			if (log_message_esc.dt == 0 || currentTime % 120 == 0 ||
				// Or we have drifted too far from the last record we must write a full ESC message
				(
					currentTime - log_message_esc.dt > 255 ||
					esc_telemetry.v_in * 100 - log_message_esc.vin > 127 ||
					esc_telemetry.temp_motor * 100 - log_message_esc.motor_temp > 127 ||
					esc_telemetry.temp_mos * 100 - log_message_esc.mosfet_temp > 127 ||
					esc_telemetry.watt_hours * 100 - log_message_esc.watt_hours > 127 ||
					esc_telemetry.watt_hours_charged * 100 - log_message_esc.watt_hours_regen > 127 ||
					abs(esc_telemetry.rpm - log_message_esc.e_rpm) > 32767 ||
					abs(esc_telemetry.tachometer_abs - log_message_esc.e_distance) > 32767
				)
			)
			{
				//TODO: duplicated code
				log_message_esc.dt = currentTime;
				log_message_esc.esc_id = esc_telemetry.vesc_id;
				log_message_esc.vin = esc_telemetry.v_in * 100;
				log_message_esc.motor_temp = esc_telemetry.temp_motor * 100;
				log_message_esc.mosfet_temp = esc_telemetry.temp_mos * 100;
				log_message_esc.duty_cycle = esc_telemetry.duty_now * 10;
				log_message_esc.motor_current = esc_telemetry.current_motor * 10;
				log_message_esc.battery_current = esc_telemetry.current_in * 10;
				log_message_esc.watt_hours = esc_telemetry.watt_hours * 100;
				log_message_esc.watt_hours_regen = esc_telemetry.watt_hours_charged * 100;
				log_message_esc.e_rpm = esc_telemetry.rpm;
				log_message_esc.e_distance = esc_telemetry.tachometer_abs;
				log_message_esc.fault = esc_telemetry.fault_code;

				// Write ESC telemetry data
				size_t bytes_written = 0;
				char start[3] = {PACKET_START, ESC, sizeof(log_message_esc)};
				char end[1] = {PACKET_END};
				bytes_written += lfs_file_write(&lfs, &file, &start, sizeof(start));
				bytes_written += lfs_file_write(&lfs, &file, &log_message_esc, sizeof(log_message_esc));
				bytes_written += lfs_file_write(&lfs, &file, &end, sizeof(end));

				NRF_LOG_INFO("ESC Bytes Written: %ld",bytes_written);
				NRF_LOG_FLUSH();
			}
			else
			{
				// Update delta message
				log_message_esc_delta.dt = currentTime - log_message_esc.dt;
				log_message_esc_delta.esc_id = esc_telemetry.vesc_id;
				log_message_esc_delta.vin = esc_telemetry.v_in * 100 - log_message_esc.vin;
				log_message_esc_delta.motor_temp = esc_telemetry.temp_motor * 010 - log_message_esc.motor_temp;
				log_message_esc_delta.mosfet_temp = esc_telemetry.temp_mos * 100 - log_message_esc.mosfet_temp;
				log_message_esc_delta.duty_cycle = esc_telemetry.duty_now * 10 - log_message_esc.duty_cycle;
				log_message_esc_delta.motor_current = esc_telemetry.current_motor * 10 - log_message_esc.motor_current;
				log_message_esc_delta.battery_current = esc_telemetry.current_in * 10 - log_message_esc.battery_current;
				log_message_esc_delta.watt_hours = esc_telemetry.watt_hours * 100 - log_message_esc.watt_hours;
				log_message_esc_delta.watt_hours_regen = esc_telemetry.watt_hours_charged * 100 - log_message_esc.watt_hours_regen;
				log_message_esc_delta.e_rpm = esc_telemetry.rpm - log_message_esc.e_rpm;
				log_message_esc_delta.e_distance = esc_telemetry.tachometer_abs - log_message_esc.e_distance;
				log_message_esc_delta.fault = esc_telemetry.fault_code;

				//Update full message
				//TODO: duplicated code
				log_message_esc.dt = currentTime;
				log_message_esc.esc_id = esc_telemetry.vesc_id;
				log_message_esc.vin = esc_telemetry.v_in * 100;
				log_message_esc.motor_temp = esc_telemetry.temp_motor * 100;
				log_message_esc.mosfet_temp = esc_telemetry.temp_mos * 100;
				log_message_esc.duty_cycle = esc_telemetry.duty_now * 10;
				log_message_esc.motor_current = esc_telemetry.current_motor * 10;
				log_message_esc.battery_current = esc_telemetry.current_in * 10;
				log_message_esc.watt_hours = esc_telemetry.watt_hours * 100;
				log_message_esc.watt_hours_regen = esc_telemetry.watt_hours_charged * 100;
				log_message_esc.e_rpm = esc_telemetry.rpm;
				log_message_esc.e_distance = esc_telemetry.tachometer_abs;
				log_message_esc.fault = esc_telemetry.fault_code;

				// Write out ESC DELTA message
				size_t bytes_written = 0;
				char start[3] = {PACKET_START, ESC_DELTA, sizeof(log_message_esc_delta)};
				char end[1] = {PACKET_END};
				bytes_written += lfs_file_write(&lfs, &file, &start, sizeof(start));
				bytes_written += lfs_file_write(&lfs, &file, &log_message_esc_delta, sizeof(log_message_esc_delta));
				bytes_written += lfs_file_write(&lfs, &file, &end, sizeof(end));
				NRF_LOG_INFO("ESC DELTA Bytes Written: %ld", bytes_written);
				NRF_LOG_FLUSH();
			}
		}

		// Fault monitoring
		if (esc_telemetry.fault_code != 0) {
			// Track total number of faults seen
			++fault_count;

			// Track fault codes that have been seen
			if (is_fault_new(esc_telemetry.fault_code, esc_telemetry.vesc_id))
			{
				recent_faults[recent_fault_index].fault_code = esc_telemetry.fault_code;
				recent_faults[recent_fault_index].fault_count = 1;
				recent_faults[recent_fault_index].esc_id = esc_telemetry.vesc_id;
				recent_faults[recent_fault_index].dt_first_seen = currentTime;
				recent_faults[recent_fault_index].dt_last_seen = currentTime;
				if(recent_fault_index < RECENT_FAULT_LIMIT)
				{
					++recent_fault_index;
				}
			} else {
				update_fault(esc_telemetry.fault_code, esc_telemetry.vesc_id);
			}

			// Alert user, don't interrupt current melody
			melody_play(MELODY_ESC_FAULT, false); // Play fault sound, do not interrupt
		}

		if (gotchi_cfg_user.alert_low_voltage != 0.0 && esc_telemetry.v_in < gotchi_cfg_user.alert_low_voltage) {
			melody_play(MELODY_VOLTAGE_LOW, false); // Play fault sound, do not interrupt
		}
		if (gotchi_cfg_user.alert_esc_temp != 0.0 && esc_telemetry.temp_mos > gotchi_cfg_user.alert_esc_temp) {
			melody_play(MELODY_ESC_TEMP, false); // Play fault sound, do not interrupt
		}
		if (gotchi_cfg_user.alert_motor_temp != 0.0 && esc_telemetry.temp_motor > gotchi_cfg_user.alert_motor_temp) {
			melody_play(MELODY_MOTOR_TEMP, false); // Play fault sound, do not interrupt
		}

		++esc_rx_cnt;
#if HAS_DISPLAY
		// Update fault code count on display
		Adafruit_GFX_setCursor(106,16);
		sprintf(display_text_buffer,"F%02d", recent_fault_index);
		Adafruit_GFX_print(display_text_buffer);

		// Move dot each time we process an ESC VALUES packet
		if (esc_rx_cnt % 2 == 0) {
			Adafruit_GFX_fillCircle(120, 2, 2, WHITE);
			Adafruit_GFX_fillCircle(110, 2, 2, BLACK);
		} else {
			Adafruit_GFX_fillCircle(120, 2, 2, BLACK);
			Adafruit_GFX_fillCircle(110, 2, 2, WHITE);
		}
		update_display = true;
#endif
	}

	// Watch telemetry data to trigger logging
	// If we are logging now see if we should stop
	if (log_file_active) {
		if(esc_telemetry.v_in < gotchi_cfg_user.log_auto_stop_low_voltage) {
			log_file_stop();
			NRF_LOG_INFO("Logging stopped due to power drop");
			NRF_LOG_FLUSH();
		} else if (currentTime - lastTimeBoardMoved > gotchi_cfg_user.log_auto_stop_idle_time) {
			log_file_stop();
			NRF_LOG_INFO("Logging stopped due to inactivity");
			NRF_LOG_FLUSH();
		} else if ((int)fabs(esc_telemetry.rpm) > gotchi_cfg_user.log_auto_start_erpm) {
			// We are moving while logging. Keep it up!
			lastTimeBoardMoved = currentTime;
		}
	}
	// We are not logging, see if we should start
	else if ((int)fabs(esc_telemetry.rpm) > gotchi_cfg_user.log_auto_start_erpm && !sync_in_progress) {
		log_file_start();
		NRF_LOG_INFO("Logging started automatically");
		NRF_LOG_FLUSH();
	}

	// Finish packet processing
	if (data[0] == COMM_EXT_NRF_ESB_SET_CH_ADDR) {
		//NRF_LOG_INFO("COMM_EXT_NRF_ESB_SET_CH_ADDR 0x%02x", data[1]);
		//NRF_LOG_FLUSH();
	} else if (data[0] == COMM_EXT_NRF_ESB_SEND_DATA) {
		//Send data to the user's remote
		//NRF_LOG_INFO("COMM_EXT_NRF_ESB_SEND_DATA");
		//NRF_LOG_FLUSH();
	} else if (data[0] == COMM_EXT_NRF_SET_ENABLED) {
		//NRF_LOG_INFO("COMM_EXT_NRF_SET_ENABLED");
		//NRF_LOG_FLUSH();
	} else {
		if (m_is_enabled) {
			packet_send_packet(data, len, PACKET_BLE);
		}
	}
}

void ble_printf(const char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = COMM_PRINT;
	len = vsnprintf(print_buffer + 1, 254, format, arg);
	va_end (arg);

	if(len > 0) {
		packet_send_packet((unsigned char*)print_buffer, (len < 254) ? len + 1 : 255, PACKET_BLE);
	}
}

void cdc_printf(const char* format, ...) {
#ifdef NRF52840_XXAA
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	len = vsnprintf(print_buffer, sizeof(print_buffer), format, arg);
	va_end (arg);

	if(len > 0) {
		app_usbd_cdc_acm_write(&m_app_cdc_acm, print_buffer,
				len < sizeof(print_buffer) ? len : sizeof(print_buffer));
	}
#else
	(void)format;
#endif
}

static void packet_timer_handler(void *p_context) {
	(void)p_context;
	packet_timerfunc();

	CRITICAL_REGION_ENTER();
	if (m_other_comm_disable_time > 0) {
		m_other_comm_disable_time--;
	}
	CRITICAL_REGION_EXIT();
}

static void logging_timer_handler(void *p_context) {
	(void)p_context;
	static char gps_status[11] = {0};

	// Increment time by 1 second as this is called at 1Hz
	currentTime++;
	tmTime = localtime( &currentTime );

	strftime(datetimestring, 64, "%Y-%m-%dT%H:%M:%S", tmTime);

	// Write GPS status to display
	Adafruit_GFX_setCursor(64,8);
	snprintf(gps_status, sizeof(gps_status), "GPS %02d S%01d%01d", hgps.seconds, hgps.is_valid, hgps.fix);
	Adafruit_GFX_print(gps_status);
	update_display = true;

	// If logging is active and GPS is valid and fixed log GPS data
	if (log_file_active && hgps.is_valid && hgps.fix > 0)
	{
		// If we have not yet logged a full GPS message do so now
		if 	(log_message_gps.dt == 0 || currentTime % 120 == 0 ||
				// Or we have drifted too far from the last record we must write a full GPS message
				(
					currentTime - log_message_gps.dt > 255 ||
					fabs(hgps.altitude) * 10 - log_message_gps.altitude > 127 ||
					fabs(hgps.speed) * 10 - log_message_gps.speed > 127 ||
					fabs(hgps.latitude) * 10000 - abs(log_message_gps.latitude) > 32767 ||
					fabs(hgps.longitude) * 10000 - abs(log_message_gps.longitude) > 32767
				)
			)
		{
			//TODO: duplicated code
			log_message_gps.dt = currentTime;
			log_message_gps.satellites = hgps.sats_in_view;
			log_message_gps.altitude = fabs(hgps.altitude) * 10;
			log_message_gps.speed = fabs(hgps.speed) * 10;
			log_message_gps.latitude = hgps.latitude * 100000;
			log_message_gps.longitude = hgps.longitude * 100000;

			// Write out full GPS message
			size_t bytes_written = 0;
			char start[3] = {PACKET_START, GPS, sizeof(log_message_gps)};
			char end[1] = {PACKET_END};
			bytes_written += lfs_file_write(&lfs, &file, &start, sizeof(start));
			bytes_written += lfs_file_write(&lfs, &file, &log_message_gps, sizeof(log_message_gps));
			bytes_written += lfs_file_write(&lfs, &file, &end, sizeof(end));
			NRF_LOG_INFO("GPS Bytes Written: %ld", bytes_written);
			NRF_LOG_FLUSH();
		}
		// We can write a GPS Delta message!
		else
		{
			// Update delta message
			log_message_gps_delta.dt = currentTime - log_message_gps.dt;
			log_message_gps_delta.satellites = hgps.sats_in_view - log_message_gps.satellites;
			log_message_gps_delta.altitude = fabs(hgps.altitude) * 10 - log_message_gps.altitude;
			log_message_gps_delta.speed = fabs(hgps.speed) * 10 - log_message_gps.speed;
			log_message_gps_delta.latitude = (int32_t)(hgps.latitude * 100000) - log_message_gps.latitude;
			log_message_gps_delta.longitude = (int32_t)(hgps.longitude * 100000) - log_message_gps.longitude;

			// Update full message
			//TODO: duplicated code
			log_message_gps.dt = currentTime;
			log_message_gps.satellites = hgps.sats_in_view;
			log_message_gps.altitude = fabs(hgps.altitude) * 10;
			log_message_gps.speed = fabs(hgps.speed) * 10;
			log_message_gps.latitude = hgps.latitude * 100000;
			log_message_gps.longitude = hgps.longitude * 100000;

			// Write out GPS DELTA message
			size_t bytes_written = 0;
			char start[3] = {PACKET_START, GPS_DELTA, sizeof(log_message_gps_delta)};
			char end[1] = {PACKET_END};
			bytes_written += lfs_file_write(&lfs, &file, &start, sizeof(start));
			bytes_written += lfs_file_write(&lfs, &file, &log_message_gps_delta, sizeof(log_message_gps_delta));
			bytes_written += lfs_file_write(&lfs, &file, &end, sizeof(end));
			NRF_LOG_INFO("GPS DELTA Bytes Written: %ld", bytes_written);
			NRF_LOG_FLUSH();
		}
	}

	// Track GPS signal acquisition and loss
	if (gps_signal_locked && (!hgps.is_valid || hgps.fix < 1))
	{
		gps_signal_locked = false;
		melody_play(MELODY_GPS_LOST, true); // Play GPS lost melody, interrupt
	}
	else if (!gps_signal_locked && hgps.is_valid && hgps.fix > 0)
	{
		gps_signal_locked = true;
		melody_play(MELODY_GPS_LOCK, false); // Play GPS locked melody, do not interrupt
	}

	// Sync filesystem contents every 60 seconds
	if (log_file_active && currentTime % 60 == 0)
	{
		lfs_file_sync(&lfs, &file);

		// Check for free space after committing data to storage
		if (lfs_free_space_check() < 10) //TODO: define percentage free limit
		{
			// Check if user wants to erase old files automatically
			if (gotchi_cfg_user.log_auto_erase_when_full == 1)
			{
				//TODO: call method to erase oldest file until free space is happy
				log_file_stop();
			}
			else
			{
				// Stop logging to prevent corruption
				log_file_stop();
			}
		}
	}

	//TODO: Sync GPS time - this works but is UTC and we've made no commitment to timezones
	/*
	if (!rtc_time_has_sync && !log_file_active && hgps.seconds != 247 && hgps.seconds != 0)
	{
		// Update time in memory
		tmTime->tm_year = 2000 + hgps.year - 1900;
		tmTime->tm_mon = hgps.month - 1;
		tmTime->tm_mday = hgps.date;
		tmTime->tm_hour = hgps.hours;
		tmTime->tm_min = hgps.minutes;
		tmTime->tm_sec = hgps.seconds;
		currentTime = mktime(tmTime);
		currentTime += -6 * 60 * 60; //TODO: Add the user's offset from configuration

		// Update time on RTC
		update_rtc = true;

		NRF_LOG_INFO("Setting time from GPS");
		NRF_LOG_FLUSH();
	}
	*/

	// Check if the ESC has not responded in 3 seconds and try swapping the TX and RX pins
	if (currentTime - time_esc_last_responded > 3)
	{
		NRF_LOG_INFO("ESC has not responded in 3 seconds. Swapping UART TX and RX pins");
		NRF_LOG_FLUSH();
		uart_swap_pins();
		// Reset the countdown
		time_esc_last_responded = currentTime;
	}

	// Check if the GPS has not provided data while a signal lock was true
	if (currentTime - time_gps_last_responded > 3)
	{
		NRF_LOG_INFO("GPS has not provided any data for 3 seconds.");
		NRF_LOG_FLUSH();
		// Clear the GPS seconds for display
		hgps.seconds = 0;
		// Clear valid GPS status and fix flags for display
		hgps.is_valid = 0;
		hgps.fix = 0;
		// Clear sats in view for status packet
		hgps.sats_in_view = 0;
		if (gps_signal_locked)
		{
			// Clear signal lock flag
			gps_signal_locked = false;
			melody_play(MELODY_GPS_LOST, true); // Play GPS lost melody, interrupt
		}
		time_gps_last_responded = currentTime;
	}
}

static void telemetry_timer_handler(void *p_context) {
	(void)p_context;
	// Set flag to write data when a response is received
	write_logdata_now = true;

	// Requesting ESC telemetry
	static unsigned char telemetryPacket[] = {0x02, 0x01, COMM_GET_VALUES, 0x40, 0x84, 0x03};
	static unsigned char telemetryPacketCAN[] = {0x02, 0x03, COMM_FORWARD_CAN, 0x00, COMM_GET_VALUES, 0x00, 0x00, 0x03};
	static uint16_t crc;
	switch (gotchi_cfg_user.multi_esc_mode)
	{
		case 2: //Dual ESC Mode (1 CAN FWD)
			//NRF_LOG_INFO("Dual ESC Mode"); NRF_LOG_FLUSH();
			switch (multiESCIndex++)
			{
				case 0:
					//NRF_LOG_INFO("requesting esc values locally"); NRF_LOG_FLUSH();
					uart_send_buffer(telemetryPacket, 6);
				break;
				case 1:
					//NRF_LOG_INFO("requesting esc values over can"); NRF_LOG_FLUSH();
					telemetryPacketCAN[3] = gotchi_cfg_user.multi_esc_ids[0];
					crc = crc16(telemetryPacketCAN + 2, 3);
					telemetryPacketCAN[5] = crc >> 8;
					telemetryPacketCAN[6] = crc & 0xff;
					uart_send_buffer(telemetryPacketCAN, 8);
					multiESCIndex = 0; // Reset cycle
				break;
				default:
					multiESCIndex = 0; // Config switched, reset
				break;
			}
		break;
		case 4:
			NRF_LOG_INFO("Quad ESC Mode"); NRF_LOG_FLUSH();
			switch (multiESCIndex++)
			{
				case 0:
					uart_send_buffer(telemetryPacket, 6);
				break;
				case 1:
					telemetryPacketCAN[3] = gotchi_cfg_user.multi_esc_ids[0];
					crc = crc16(telemetryPacketCAN + 2, 3);
					telemetryPacketCAN[5] = crc >> 8;
					telemetryPacketCAN[6] = crc & 0xff;
					uart_send_buffer(telemetryPacketCAN, 8);
				break;
				case 2:
					telemetryPacketCAN[3] = gotchi_cfg_user.multi_esc_ids[1];
					crc = crc16(telemetryPacketCAN + 2, 3);
					telemetryPacketCAN[5] = crc >> 8;
					telemetryPacketCAN[6] = crc & 0xff;
					uart_send_buffer(telemetryPacketCAN, 8);
				break;
				case 3:
					telemetryPacketCAN[3] = gotchi_cfg_user.multi_esc_ids[2];
					crc = crc16(telemetryPacketCAN + 2, 3);
					telemetryPacketCAN[5] = crc >> 8;
					telemetryPacketCAN[6] = crc & 0xff;
					uart_send_buffer(telemetryPacketCAN, 8);
					multiESCIndex = 0; // Reset cycle
				break;
				default:
					multiESCIndex = 0;
				break;
			}
		break;
		default:
			// Request from single ESC only
			uart_send_buffer(telemetryPacket, 6);
	}
}

void display_file_count(void)
{
	NRF_LOG_INFO("display_file_count");
	NRF_LOG_FLUSH();
#if HAS_DISPLAY
	Adafruit_GFX_setCursor(0,8);
	sprintf(display_text_buffer,"%d files   ", lfs_file_count);
	Adafruit_GFX_print(display_text_buffer);
	update_display = true;
#endif
}

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with OLED.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
static ret_code_t twi_master_init(void)
{
	ret_code_t ret;
	const nrf_drv_twi_config_t config =
	{
	   .scl				= 27,
	   .sda				= 26,
	   .frequency		  = NRF_TWI_FREQ_400K,
	   .interrupt_priority = APP_IRQ_PRIORITY_MID,
	   .clear_bus_init	 = true
	};

	ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

	if (NRF_SUCCESS == ret)
	{
		nrf_drv_twi_enable(&m_twi_master);
	}

	return ret;
}



////////////////////////////////////////
// QSPI
////////////////////////////////////////
#define QSPI_STD_CMD_WRSR   0x01
#define QSPI_STD_CMD_RSTEN  0x66
#define QSPI_STD_CMD_RST	0x99

static void configure_memory()
{
	uint8_t temporary = 0x40;
	uint32_t err_code;
	nrf_qspi_cinstr_conf_t cinstr_cfg = {
		.opcode	   = QSPI_STD_CMD_RSTEN,
		.length	   = NRF_QSPI_CINSTR_LEN_1B,
		.io2_level = true,
		.io3_level = true,
		.wipwait   = true,
		.wren	   = true
	};

	// Send reset enable
	err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	// Send reset command
	cinstr_cfg.opcode = QSPI_STD_CMD_RST;
	err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, NULL, NULL);
	APP_ERROR_CHECK(err_code);

	// Switch to qspi mode
	cinstr_cfg.opcode = QSPI_STD_CMD_WRSR;
	cinstr_cfg.length = NRF_QSPI_CINSTR_LEN_2B;
	err_code = nrf_drv_qspi_cinstr_xfer(&cinstr_cfg, &temporary, NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("QSPI Memory Configured");
	NRF_LOG_FLUSH();
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void qspi_init()
{
	uint32_t err_code=0;

	NRF_LOG_INFO("QSPI initializing");
	NRF_LOG_FLUSH();

	nrf_drv_qspi_config_t config = NRF_DRV_QSPI_DEFAULT_CONFIG;

	err_code = nrf_drv_qspi_init(&config, NULL, NULL);

	if(err_code != NRF_SUCCESS)
	{
		NRF_LOG_INFO("nrf_drv_qspi_init response %d", err_code);
		NRF_LOG_FLUSH();
#if HAS_DISPLAY
		Adafruit_GFX_setCursor(0,16);
		sprintf(display_text_buffer,"QSPI Init Failed");
		Adafruit_GFX_print(display_text_buffer);
		SSD1306_display();
#endif
		while(1);
	}

	NRF_LOG_INFO("QSPI driver initialized");
	NRF_LOG_FLUSH();

	configure_memory();
}

int log_file_stop()
{
	if (log_file_active)
	{
		log_file_active = false;

#if HAS_DISPLAY
		Adafruit_GFX_setCursor(0,16);
		sprintf(display_text_buffer,"Log inactive");
		Adafruit_GFX_print(display_text_buffer);
		update_display = true;
#endif
		// Clear log messages for delta processing
		memset(&log_message_gps,0,sizeof(log_message_gps));
		memset(&log_message_esc,0,sizeof(log_message_esc));

		int lfs_close_result = lfs_file_close(&lfs, &file);
		NRF_LOG_INFO("log_file_stop::lfs_file_close() result: %d", lfs_close_result);
		NRF_LOG_FLUSH();
		melody_play(MELODY_DESC, false); // Play log stop melody, do not interrupt
		return lfs_close_result;
	}
	return -1;
}

void log_file_start()
{
	if (sync_in_progress)
	{
		NRF_LOG_INFO("log_file_start() aborting due to sync_in_progress");
		NRF_LOG_FLUSH();
		return;
	}

	// Flag the board as in motion to begin the countdown to idle
	lastTimeBoardMoved = currentTime;

	// Stop a previous log if in progress
	if (log_file_active)
	{
		log_file_stop();
	}

	//TODO: check for free space
	if (lfs_free_space_check() < 10) //TODO: define percentage free limit
	{
		// Check if user wants to erase old files automatically
		if (gotchi_cfg_user.log_auto_erase_when_full == 1)
		{
			//TODO: call method to erase oldest file until free space is happy
			return;
		}
		else
		{
			// Do not log, do not pass go, do not collect $200
			return;
		}
	}

	// Create the new file
	char filename[64];
	strftime( filename, 64, "/FreeSK8Logs/%Y-%m-%dT%H:%M:%S", tmTime );

	NRF_LOG_INFO("Creating log file: %s",filename);
	NRF_LOG_FLUSH();

	int file_open_result = lfs_file_opencfg(&lfs, &file, filename, LFS_O_WRONLY | LFS_O_CREAT, &lfs_file_config);
	if (file_open_result >= 0)
	{
		NRF_LOG_INFO("log_file_active");
		NRF_LOG_FLUSH();
		log_file_active = true;
		multiESCIndex = 0; // Always request from primary ESC first
		++lfs_file_count;
		display_file_count();
#if HAS_DISPLAY
		Adafruit_GFX_setCursor(0,16);
		sprintf(display_text_buffer,"Log active  ");
		Adafruit_GFX_print(display_text_buffer);
		update_display = true;
#endif
		melody_play(MELODY_ASC, false); // Play log started melody, do not interrupt
	} else {
		NRF_LOG_ERROR("log_file_start::lfs_file_open: Failed with result: %d", file_open_result);
		NRF_LOG_FLUSH();
	}
}

void update_status_packet(char * buffer)
{
	// Update the buffer with the a status response packet
	sprintf(buffer, "status,OK,%d,%d,%d,%d,%d,%d,%d", log_file_active, fault_count, recent_fault_index, lfs_percent_free, lfs_file_count, hgps.fix, hgps.sats_in_view);
}

uint16_t create_fault_packet(char * buffer)
{
	NRF_LOG_INFO("fault data size: %d", sizeof(struct esc_fault));
	NRF_LOG_FLUSH();
	NRF_LOG_INFO("timesize %d", sizeof(time_t));
	NRF_LOG_FLUSH();
	NRF_LOG_INFO("dt %ld", recent_faults[0].dt_first_seen);
	NRF_LOG_FLUSH();

	uint16_t buffer_position = 0;
	buffer_position = sprintf(buffer, "faults,%02d,", recent_fault_index);
	//TODO: include all faults
	//      requires multiple messages and I'm not interested atm
	for(int i=0; i<6;++i)
	{
		memcpy(buffer + buffer_position, &recent_faults[i], sizeof(struct esc_fault));
		buffer_position += sizeof(struct esc_fault);
		//sprintf(buffer + buffer_position, "%d,%d,%d,%ld,", recent_faults[i].fault_code, recent_faults[i].fault_count, recent_faults[i].esc_id, recent_faults[i].dt_last_seen)
	}
	return buffer_position;
}

void littlefs_init()
{
	NRF_LOG_INFO("LittleFS initializing");
    NRF_LOG_FLUSH();

	// mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
	if (err) {
        NRF_LOG_WARNING("LittleFS needs to format the storage");
        NRF_LOG_FLUSH();
        int lfs_format_response = lfs_format(&lfs, &cfg);
        int lfs_mount_response = lfs_mount(&lfs, &cfg);
		NRF_LOG_WARNING("LittleFS format (%d) and mount (%d) completed", lfs_format_response, lfs_mount_response);
		user_cfg_set();
#if HAS_DISPLAY
		Adafruit_GFX_setCursor(45,0);
		Adafruit_GFX_print("*F");
		SSD1306_display();
#endif
    }
    NRF_LOG_INFO("LittleFS initialized");
    NRF_LOG_FLUSH();

	// Prepare the static file buffer
	memset(&lfs_file_config, 0, sizeof(struct lfs_file_config));
	lfs_file_config.buffer = lfs_file_buf;
	lfs_file_config.attr_count = 0;

    // read current count  
    lfs_file_opencfg(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT, &lfs_file_config);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
    NRF_LOG_INFO("Read file complete. Boot count: %d", boot_count);

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
    NRF_LOG_INFO("Write file complete. Boot count incremented");

    // storage is not updated until the file is closed
    lfs_file_close(&lfs, &file);
    NRF_LOG_INFO("Close file complete");

	// open the log directory, create directory if needed
	lfs_dir_t directory;
	if( lfs_dir_open(&lfs, &directory, "/FreeSK8Logs") < 0 )
	{
		NRF_LOG_INFO("Creating /FreeSK8Logs");
		// create a directory in root
		lfs_mkdir(&lfs, "/FreeSK8Logs");
		lfs_dir_open(&lfs,&directory,"/FreeSK8Logs");
	}

	// list the contents of the log directory
	NRF_LOG_INFO("Contents of /FreeSK8Logs");

	struct lfs_info entryinfo;
	while(lfs_dir_read(&lfs,&directory,&entryinfo))
	{
		NRF_LOG_INFO("%s %d bytes %s", entryinfo.type == LFS_TYPE_REG ? "FILE" : "DIR ", entryinfo.size, entryinfo.name);
		NRF_LOG_FLUSH();
		// Cleaning 0 bytes files from system at boot, they are of no use
		if( entryinfo.type == LFS_TYPE_REG && entryinfo.size == 0 )
		{
			char filepath[64] = "/FreeSK8Logs/";
			sprintf( filepath + strlen( filepath ), entryinfo.name );
			int remove_response = lfs_remove( &lfs, filepath );
            if (remove_response >= 0)
            {
                NRF_LOG_INFO("rm,OK,%s", filepath);
            }
            else
            {
                NRF_LOG_INFO("rm,FAIL,%s,%d", filepath, remove_response);
            }
			NRF_LOG_FLUSH();
		}
		else if (entryinfo.type == LFS_TYPE_REG)
		{
			++lfs_file_count;
		}
	}

	lfs_dir_close(&lfs,&directory);

	NRF_LOG_INFO("Directory listing complete");
	NRF_LOG_FLUSH();

#if HAS_DISPLAY
	display_file_count();
	Adafruit_GFX_setCursor(0,16);
	sprintf(display_text_buffer,"Log inactive");
	Adafruit_GFX_print(display_text_buffer);
	update_display = true;
#endif
}

/////////////////////////////////////////
// User Configuration
////////////////////////////////////////

void user_cfg_set(void)
{
	NRF_LOG_INFO("Saving User Configuration");
	NRF_LOG_FLUSH();
	lfs_file_opencfg(&lfs, &file, "user_configuration", LFS_O_RDWR | LFS_O_CREAT, &lfs_file_config);
	lfs_file_write(&lfs, &file, &gotchi_cfg_user, sizeof(gotchi_cfg_user));
	lfs_file_close(&lfs, &file);
	NRF_LOG_INFO("User Configuration Saved");
	NRF_LOG_FLUSH();
}

void user_cfg_get(void)
{
	NRF_LOG_INFO("Loading User Configuration");
	NRF_LOG_FLUSH();

	struct lfs_info info;
	lfs_stat(&lfs, "user_configuration", &info);
	if (info.size >= sizeof(gotchi_cfg_user))
	{
		lfs_file_opencfg(&lfs, &file, "user_configuration", LFS_O_RDONLY, &lfs_file_config);
		lfs_file_read(&lfs, &file, &gotchi_cfg_user, sizeof(gotchi_cfg_user));
		lfs_file_close(&lfs, &file);
		NRF_LOG_INFO("User Configuration Loaded");
		NRF_LOG_FLUSH();
	}

	if (gotchi_cfg_user.cfg_version != gotchi_cfg_default.cfg_version)
	{
		NRF_LOG_WARNING("User Configuration Version Mismatch. Restoring Defaults");
		NRF_LOG_FLUSH();
		gotchi_cfg_user = gotchi_cfg_default;
		user_cfg_set();
	}
}

////////////////////////////////////////
// Game that needs it's own source file
////////////////////////////////////////

#if HAS_DISPLAY
// Dont ask
// frame counter
unsigned int frame = 0;
// scrore string buffer
char text[16];

// 'skate_push', 24x23px
const unsigned char skate_push[] =
{0x00, 0x00, 0x70, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x7c, 0xf8, 0x01, 0xff, 0x70, 0x07,
0xff, 0x00, 0x06, 0x3f, 0x00, 0x00, 0x7e, 0x00, 0x00, 0xfd, 0xe0, 0x01, 0xfb, 0xe0, 0x01, 0xf0,
0x00, 0x00, 0xf0, 0x00, 0x06, 0x78, 0x00, 0x7f, 0x3c, 0x00, 0xfe, 0x1c, 0x00, 0xfc, 0x1c, 0x00,
0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x1c, 0x00, 0x0f, 0xff, 0xe0, 0x03, 0xff, 0x80, 0x02,
0x82, 0x80, 0x03, 0x03, 0x00};
// 'skate_push_2', 24x23px
const unsigned char skate_push2[] =
{0x00, 0x01, 0xc0, 0x00, 0x03, 0xe0, 0x00, 0x03, 0xe0, 0x00, 0x7f, 0xe0, 0x01, 0xff, 0xc0, 0x07,
0xff, 0x00, 0x06, 0x3f, 0x00, 0x00, 0x7e, 0x60, 0x00, 0xfd, 0xe0, 0x01, 0xf9, 0x80, 0x01, 0xf0,
0x00, 0x00, 0xf0, 0x00, 0x00, 0x78, 0x00, 0x01, 0xfc, 0x00, 0x01, 0xdc, 0x00, 0x01, 0xdc, 0x00,
0x01, 0xdc, 0x00, 0x01, 0xdc, 0x00, 0x03, 0x9c, 0x00, 0x0f, 0xff, 0xe0, 0x03, 0xff, 0x80, 0x02,
0x82, 0x80, 0x03, 0x03, 0x00};
// 'skate_jump', 24x23px
const unsigned char skate_jump[] =
{0x00, 0x70, 0x00, 0x00, 0x78, 0x00, 0x00, 0xf8, 0xc0, 0x00, 0x73, 0xc0, 0x00, 0x27, 0x00, 0x00,
0xfe, 0x00, 0x01, 0xf8, 0x00, 0x01, 0xf8, 0x00, 0x03, 0xf8, 0x00, 0x03, 0xf8, 0x00, 0x03, 0xf8,
0x00, 0x01, 0xfc, 0x00, 0x00, 0xfe, 0x00, 0x00, 0xc6, 0x00, 0x01, 0xc6, 0x00, 0x01, 0x86, 0x30,
0x01, 0x86, 0x60, 0x03, 0x87, 0xc0, 0x03, 0xfd, 0xc0, 0x1f, 0xc1, 0xc0, 0x03, 0x80, 0xc0, 0x02,
0x80, 0x00, 0x03, 0x80, 0x00};
// 'traffic_cone', 24x24px
const unsigned char traffic_cone[] =
{0x00, 0x20, 0x00, 0x00, 0x20, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00, 0x70, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0xfc, 0x00, 0x01, 0xfc,
0x00, 0x01, 0xfc, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x02, 0x02, 0x00, 0x07, 0xff, 0x00, 0x07, 0xff, 0x40, 0x77, 0xff, 0x70, 0x3b, 0xfe, 0xe0, 0x04,
0x79, 0x80, 0x01, 0xfc, 0x00, 0x00, 0x70, 0x00};

// cloud_1 w:  24  h:  7
const unsigned char cloud_1[] =
{   0x00, 0xF0, 0x00, 0x73, 0x0C, 0x00, 0x88, 0x03,
    0x00, 0x81, 0x04, 0x80, 0x86, 0x00, 0x80, 0x79,
    0x83, 0x00, 0x00, 0x7C, 0x00 };

// skater_tumble, 32x20px
const unsigned char skater_tumble[] =
{0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x70, 0x00,
0x00, 0x00, 0x70, 0xe0, 0x00, 0x00, 0x71, 0xf0, 0x02, 0x00, 0x39, 0xf0, 0x07, 0xf8, 0x1d, 0xf0,
0x07, 0xfc, 0x1e, 0xe0, 0x00, 0xfe, 0x0f, 0x00, 0x00, 0x0e, 0x1f, 0x80, 0x00, 0x07, 0x3f, 0x80,
0x00, 0x37, 0xff, 0xc0, 0x00, 0xff, 0xfe, 0xe0, 0x01, 0xff, 0xfc, 0xe0, 0x03, 0xdf, 0xf8, 0xe0,
0x0f, 0x87, 0xf0, 0xe0, 0x0f, 0x01, 0xe0, 0xe0, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00};

// distance ran
int d, delta;
int cloud_1_y;
int d_jump, d_jump_t;
int d_run;
int d_tumble_t;
int ox;
// Really, don't ask
void play_game(){
	d = 0;
	delta = 0;
	cloud_1_y = 2;
	d_jump = 0;
	d_jump_t = 0;
	d_tumble_t = 0;
	d_run = 0;
	ox = 130;	

	while(true)
	{
		if (!d_run && isButtonPressed) {
			//TODO: why can't I do this?
			//while(isButtonPressed)
			//{
			//	nrf_delay_ms(100);
			//}
			d_run = 1;
		}

		if (d_tumble_t && isButtonPressed) {
			d = 0;
			delta = 0;
			cloud_1_y = 2;
			d_jump = 0;
			d_jump_t = 0;
			d_tumble_t = 0;
			d_run = 0;
			ox = 130;
			while(isButtonPressed)
			{
				nrf_delay_ms(100);
			}
			continue;
		}

		if (++frame>16000) frame = 0;

		// increase distance whilst running
		if (d_run && (++delta > 4)) {
			delta = 0; ++d; 
		}

		// obstacles
		if (d_run) {
			ox -= (frame%2)*(d/100) + 2;
			if (ox < -15) ox += 140 + rand() % 60;
		}

		// jump!
		if (!d_jump_t && isButtonPressed) {
			d_jump_t = 1;
			d_jump=5;

			beep_speaker_blocking(40, 20);

		} else if (d_jump_t) {
			++d_jump_t;

			if (d_jump_t<4) {
				d_jump +=4;
			} else if (d_jump_t<9) {
				d_jump +=2;
			} else if (d_jump_t<13) {
				d_jump +=1;
			} else if (d_jump_t == 16 || d_jump_t == 18) {
				d_jump +=1;
			} else if (d_jump_t == 20 || d_jump_t == 22) {
				d_jump -=1;
			} else if (d_jump_t>38) {
				d_jump = 0;
				d_jump_t = 0;
			} else if (d_jump_t>32) {
				d_jump -=4;
			} else if (d_jump_t>29) {
				d_jump -=2;
			} else if (d_jump_t>25) {
				d_jump -=1;
			}
		}

		// hit detect
		if (!d_tumble_t && ox > -10 && ox <10 && d_jump_t < 5) {
			d_tumble_t = 1;    
		}

		if (d_tumble_t) {
			if (d_tumble_t == 1) {
				beep_speaker_blocking(40,10);
			} else if (d_tumble_t == 6) {
				beep_speaker_blocking(200,20);
			}

			++d_tumble_t;
			if (d_jump > -4) {
				d_jump -= 1;
				ox -= 1;
			} else {
				d_run = 0;
			}
		}

		SSD1306_clearDisplay();

		// hud

		// score
		sprintf(text,"%d",d);
		Adafruit_GFX_setCursor(100, 0);
		Adafruit_GFX_print(text);
		
		// parallax clouds
		Adafruit_GFX_drawBitmap(128 -(d%128),cloud_1_y,cloud_1,24,7,WHITE);

		if (d%128 == 0) {
			cloud_1_y = rand() % 10;
		}

		// terrain
		if (d_jump > 4) {
			Adafruit_GFX_drawLine(0,30,128,30,WHITE);
		} else {
			Adafruit_GFX_drawLine(0,30,3,30,WHITE);
			Adafruit_GFX_drawLine(12,30,127,30,WHITE); 
		}

		// obstacles
		Adafruit_GFX_drawBitmap(ox,8,traffic_cone,24,24,WHITE);


		// dino
		int dy = 9-d_jump;

		// tumbles!
		if (d_tumble_t) {
			Adafruit_GFX_drawBitmap(0,dy,skater_tumble,32,20,WHITE);

		// runs!
		} else {
			//Adafruit_GFX_drawBitmap(0,dy,dino_top,24,18,WHITE);

			// Run, Dino, Run!
			if (d_run && !d_jump) {
				if ((frame%8)/4) {
					Adafruit_GFX_drawBitmap(0,dy,skate_push,24,23,WHITE);
				} else {
					Adafruit_GFX_drawBitmap(0,dy,skate_push2,24,23,WHITE);
				}
			} else if (d_run && d_jump) {
				Adafruit_GFX_drawBitmap(0,dy,skate_jump,24,23,WHITE);
			} else {
				Adafruit_GFX_drawBitmap(0,dy,skate_push2,24,23,WHITE);
			}
		}

		SSD1306_display(); //Game is allowed to update display when it wants
		nrf_delay_ms(16);
	}
}
//You didn't listen when I said don't ask, did you?
#endif


#define SPI_INSTANCE  2 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

//TODO: remove SPI if not in use
void spi_init(void)
{
	nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = 31;
    spi_config.miso_pin = 30;
    spi_config.mosi_pin = 29;
    spi_config.sck_pin  = 28;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));

    NRF_LOG_INFO("SPI example started.");
	while (0)
    {
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

        while (!spi_xfer_done)
        {
            __WFE();
        }

        NRF_LOG_FLUSH();

        bsp_board_led_invert(BSP_BOARD_LED_0);
        nrf_delay_ms(200);
    }
}

char ble_pin[7] = {0};
uint16_t duration_button_pressed = 0;
bool is_pin_displayed = false;
void process_user_input()
{
	// Check if user if pressing the button while we do not have an active connection
	if (isButtonPressed && !is_connection_secure)
	{
		nrf_delay_ms(25);
		if (isButtonPressed) duration_button_pressed += 25;
	}
	// If user held button for 5 seconds we clear all bonds
	if (duration_button_pressed > 5000 && m_conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		NRF_LOG_WARNING("User held button for 5 seconds without a BLE connection");
		duration_button_pressed = 0;
		//TODO: Consider asking for another button press within X milliseconds to confirm clearing
		sd_ble_gap_adv_stop(m_advertising.adv_handle);
		advertising_start(true);
		// Notify user
		Adafruit_GFX_setCursor(64, 0);
		Adafruit_GFX_print("CLEARD");
		update_display = true;
	}
	// If user pressed button for less than 5 seconds display the PIN code
	if (duration_button_pressed > 0 && !isButtonPressed)
	{
		duration_button_pressed = 0;
		if (is_pin_displayed)
		{
			is_pin_displayed = false;
			Adafruit_GFX_setCursor(64, 0);
			Adafruit_GFX_print("      ");
		}
		else
		{
			is_pin_displayed = true;
			Adafruit_GFX_setCursor(64, 0);
			Adafruit_GFX_print(ble_pin);
		}

		update_display = true;
	}
}

int main(void) {

	nrf_gpio_cfg_input(PIN_BUTTON,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(PIN_BUTTON2,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_output(LED_PIN);

	// Turn on LED during boot
	nrf_gpio_pin_set(LED_PIN);

#ifdef NRF52840_XXAA
	nrf_drv_clock_init();

	static const app_usbd_config_t usbd_config = {
			.ev_state_proc = usbd_user_ev_handler
	};

	app_usbd_serial_num_generate();
	app_usbd_init(&usbd_config);
	app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
	app_usbd_class_append(class_cdc_acm);
#endif

	// NRF LOG Init
	log_init();
	NRF_LOG_INFO("Robogotchi Starting");
	NRF_LOG_FLUSH();

	// Initialize PWM for piezo output
	buzzer_init();
	beep_speaker_blocking(75,50); //Play tone blocking, allowing time for OLED to init

	// Init I2C for RTC and OLED
	ret_code_t err_code = twi_master_init();
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("I2C Initialized");
	NRF_LOG_FLUSH();

#if HAS_DISPLAY
	//NOTE: display needs few ms before it will respond from cold boot
	SSD1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS, false);
	Adafruit_GFX_init(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT, SSD1306_drawPixel);

	SSD1306_clearDisplay();
	SSD1306_display();

	char freetitle[] = "FreeSK8";
	Adafruit_GFX_setTextSize(1);
	Adafruit_GFX_setTextColor(1,0);
	Adafruit_GFX_print(freetitle);
	SSD1306_display();

	NRF_LOG_INFO("OLED Initialized");
	NRF_LOG_FLUSH();
#endif

	// Allocate tmTime in memory
	time ( &currentTime );
	tmTime = localtime ( &currentTime );
	// Set RTC to trickle charge super capacitor
	rtc_battery_charge();
	// Get the current time from the RTC
	rtc_get_time();
	// Set the last ESC response time to now
	time_esc_last_responded = currentTime;
	// Set the last GPS response time to now
	time_gps_last_responded = currentTime;

	NRF_LOG_INFO("RTC Initialized");
	NRF_LOG_FLUSH();

#if HAS_DISPLAY
	Adafruit_GFX_setCursor(0,8);
	Adafruit_GFX_print("RTC OK");
	SSD1306_display();

	if(isButtonPressed)
	{
		play_game();
	}
#endif

	// BLE PIN CODE
	uint32_t ble_bondage_safe_word = (NRF_FICR->DEVICEID[0] +  NRF_FICR->DEVICEID[1]) % 999999;
	if (ble_bondage_safe_word < 100000)
	{
		NRF_LOG_WARNING("Computed PIN was 5 digits");
		ble_bondage_safe_word += 100000;
	}
	NRF_LOG_INFO("PIN CODE %d", ble_bondage_safe_word);
	NRF_LOG_FLUSH();
	// Store PIN for Display
	itoa(ble_bondage_safe_word, ble_pin, 10);

	// QSPI & LittleFS filesystem initilization
	qspi_init();
	littlefs_init();
	lfs_free_space_check();

	// Load the user configuration from filesystem
	user_cfg_get();

	// Init GPS after user configuration are loaded
	gps_init();

	// Turn off LED when robogotchi specific init is complete
	nrf_gpio_pin_clear(LED_PIN);

	uart_init();
	app_timer_init();
	nrf_pwr_mgmt_init();
	ble_stack_init();
	gap_params_init();
	passkey_init(ble_bondage_safe_word);
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();
	peer_manager_init();
	command_interface_init(&ble_send_logbuffer, &lfs);

	packet_init(uart_send_buffer, process_packet_vesc, PACKET_VESC);
	packet_init(ble_send_buffer, process_packet_ble, PACKET_BLE);

	app_timer_create(&m_packet_timer, APP_TIMER_MODE_REPEATED, packet_timer_handler);
	app_timer_start(m_packet_timer, APP_TIMER_TICKS(1), NULL);

	app_timer_create(&m_logging_timer, APP_TIMER_MODE_REPEATED, logging_timer_handler);
	app_timer_start(m_logging_timer, APP_TIMER_TICKS(1000), NULL);

	app_timer_create(&m_telemetry_timer, APP_TIMER_MODE_REPEATED, telemetry_timer_handler);
	app_timer_start(m_telemetry_timer, APP_TIMER_TICKS(1000 / gotchi_cfg_user.log_interval_hz), NULL);

#ifdef NRF52840_XXAA
	app_usbd_power_events_enable();
#endif

	advertising_start(false);

	melody_play(MELODY_STARTUP, true); // Play a startup sound

	NRF_LOG_INFO("Robogotchi Ready!");
	NRF_LOG_FLUSH();

	for (;;) {
#ifdef NRF52840_XXAA
		while (app_usbd_event_queue_process()){}
#endif

		// Monitor button press; Do not block for more than 25ms
		process_user_input();

		if (m_uart_error) {
			app_uart_close();
			uart_init();
			packet_reset(PACKET_VESC);
			m_uart_error = false;
		}

		uint8_t byte;
		while (app_uart_get(&byte) == NRF_SUCCESS) {
			time_esc_last_responded = currentTime;
			packet_process_byte(byte, PACKET_VESC);
		}

		while (gps_uart_get(&byte) == NRF_SUCCESS) {
			time_gps_last_responded = currentTime;
			lwgps_process(&hgps, &byte, sizeof(byte));
		}

		if (update_rtc) {
			update_rtc = false;
			rtc_set_time( tmTime->tm_year + 1900, tmTime->tm_mon + 1, tmTime->tm_mday, tmTime->tm_hour, tmTime->tm_min, tmTime->tm_sec );
			rtc_time_has_sync = true;
		}
#if HAS_DISPLAY
		if (update_display) {
			update_display = false;
			SSD1306_display();
		}
#endif

		melody_step();

		sd_app_evt_wait();
	}
}
