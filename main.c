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
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "bsp.h"

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
#include "esb_timeslot.h"
#include "crc.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "command_interface.h"
#include <time.h>

static volatile TELEMETRY_DATA esc_telemetry;
static volatile int esc_rx_cnt = 0;

//Display
#define HAS_DISPLAY 1
#if HAS_DISPLAY
#include "nrf_drv_twi.h"
const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);
static char display_text_buffer[32] = {0};
#include "SSD1306.h"
#include "Adafruit_GFX.h"
#endif

// temporary time tracking intil RTC
struct tm * tmTime;
time_t currentTime;
static volatile char datetimestring[ 64 ] = { 0 };
static volatile bool log_file_active = false;


//RTC
#define RTC_ADDRESS 0x56
void rtc_battery_charge()
{
	ret_code_t ret;
	uint8_t data[] = {0x30, 0x10};
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 2, false);
	APP_ERROR_CHECK(ret);
}
void rtc_system_reset()
{
	ret_code_t ret;
	uint8_t data[] = {0x04, 0x10};
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 2, false);
	APP_ERROR_CHECK(ret);
}
void rtc_clear_control_status()
{
	ret_code_t ret;
	uint8_t data[] = {0x03, 0x00};
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 2, false);
	APP_ERROR_CHECK(ret);
}

void rtc_clear_Control_INT()
{
	ret_code_t ret;
	uint8_t data[] = {0x02, 0x00};
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 2, false);
	APP_ERROR_CHECK(ret);
}

void rtc_clear_PON_BIT()
{
	ret_code_t ret;

	uint8_t data[] = {0x03};
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 1, true);
	APP_ERROR_CHECK(ret);

	uint8_t pdata[1];
	ret = nrf_drv_twi_rx(&m_twi_master, RTC_ADDRESS, pdata, 1);
	APP_ERROR_CHECK(ret);

	pdata[ 0 ] = pdata[ 0 ] & !BIT(5);

	uint8_t data2[] = { 0x03, pdata[ 0 ] };
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data2, 2, false);
	APP_ERROR_CHECK(ret);
}

void rtc_control_status()
{
	NRF_LOG_INFO("checking control status");
	NRF_LOG_FLUSH();
	ret_code_t ret;
	uint8_t data[] = {0x03};
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 1, true);
	APP_ERROR_CHECK(ret);

	uint8_t pdata[1];
	ret = nrf_drv_twi_rx(&m_twi_master, RTC_ADDRESS, pdata, 1);
	APP_ERROR_CHECK(ret);

	NRF_LOG_INFO("control status register = 0x%02x", pdata[0]);
	NRF_LOG_FLUSH();
}
static inline uint8_t bcd_decimal(uint8_t hex)
{
    ASSERT(((hex & 0xF0) >> 4) < 10);  // More significant nybble is valid
    ASSERT((hex & 0x0F) < 10);         // Less significant nybble is valid
    uint8_t dec = ((hex & 0xF0) >> 4) * 10 + (hex & 0x0F);
    return dec;
}     

void rtc_get_time()
{
	//NRF_LOG_INFO("getting time");
	//NRF_LOG_FLUSH();
	ret_code_t ret;
	uint8_t data[] = { 0x08 };
	ret = nrf_drv_twi_tx( &m_twi_master, RTC_ADDRESS, data, 1, true );
	APP_ERROR_CHECK(ret);

	uint8_t pdata[16];
	ret = nrf_drv_twi_rx(&m_twi_master, RTC_ADDRESS, pdata, 7);
	APP_ERROR_CHECK(ret);

/*
	char tmpbf[32]={0};
	for( int i = 0; i < 7; ++i )
	{
		sprintf(tmpbf + strlen(tmpbf), "%02x ", pdata[i] );
	}
	NRF_LOG_INFO("read from RTC %s", tmpbf);
	NRF_LOG_FLUSH();
*/

	tmTime->tm_sec = 	bcd_decimal( pdata[ 0 ] );
	tmTime->tm_min = 	bcd_decimal( pdata[ 1 ] );
	tmTime->tm_hour = bcd_decimal( pdata[ 2 ] );
	tmTime->tm_mday = bcd_decimal( pdata[ 3 ] );
	tmTime->tm_wday = bcd_decimal( pdata[ 4 ] ) - 1;
	tmTime->tm_mon = 	bcd_decimal( pdata[ 5 ] ) - 1;
	tmTime->tm_year = bcd_decimal( pdata[ 6 ] ) + 100; //tm_year starts at 1900, rtc starts at 2000
	currentTime = mktime( tmTime );
}

void ConvertToBinary( uint8_t n, char* output )
{
    int l = sizeof(n) * 8;
    for (int i = l - 1 ; i >= 0; i--) {
        sprintf( output + strlen( output ), "%x", (n & (1 << i)) >> i);
    }
}

void printAllControlPages()
{
	ret_code_t ret;

	uint8_t data[] = {0x00};
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 1, true);
	APP_ERROR_CHECK(ret);

	uint8_t pdata[5];
	ret = nrf_drv_twi_rx(&m_twi_master, RTC_ADDRESS, pdata, 5);
	APP_ERROR_CHECK(ret);

	char binout[24];
	for( int i = 0; i < 5; ++i )
	{
		binout[0]=0;
		ConvertToBinary( pdata[ i ], binout );
		NRF_LOG_INFO( "reg %d> %s", i, binout );
		NRF_LOG_FLUSH();
	}
	NRF_LOG_INFO( "\n" );
	NRF_LOG_FLUSH();
}


int DecimalToBCD( int decimal )
{
   return (((decimal/10) << 4) | (decimal % 10));
}
int BCDToDecimal(int BCD)
{
   return (((BCD>>4)*10) + (BCD & 0xF));
}
void rtc_set_time( int year, int month, int day, int hour, int minute, int second )
{
	ret_code_t ret;
	uint8_t data[ 16 ] = { 0 };
	NRF_LOG_INFO("setting time");


	data[ 0 ] = 0x00;
	data[ 1 ] = 0x00;
	ret = nrf_drv_twi_tx( &m_twi_master, RTC_ADDRESS, data, 2, false );
	if (NRF_SUCCESS != ret)
	{
		NRF_LOG_INFO("stop clock failed");
		NRF_LOG_FLUSH();
		return;
	}

	time_t rawtime;
	struct tm * timeinfo;
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	
	
	timeinfo->tm_sec	= second; //int	seconds after the minute	0-60*
	timeinfo->tm_min	= minute; //int	minutes after the hour	0-59
	timeinfo->tm_hour	= hour; //int	hours since midnight	0-23
	timeinfo->tm_mday	= day; //int	day of the month	1-31
	timeinfo->tm_mon	= month - 1; //int	months since January	0-11
	timeinfo->tm_year	= year - 1900;//int	years since 1900	
	mktime( timeinfo );
	

	NRF_LOG_INFO( "Setting date/time to: %s", asctime (timeinfo) );
	NRF_LOG_FLUSH();

	data[ 0 ] = 0x08;
	data[ 1 ] = DecimalToBCD( timeinfo->tm_sec  );
	data[ 2 ] = DecimalToBCD( timeinfo->tm_min  );
	data[ 3 ] = DecimalToBCD( timeinfo->tm_hour );
	data[ 4 ] = DecimalToBCD( timeinfo->tm_mday );
	data[ 5 ] = DecimalToBCD( timeinfo->tm_wday + 1 );
	data[ 6 ] = DecimalToBCD( timeinfo->tm_mon + 1 );
	data[ 7 ] = DecimalToBCD( timeinfo->tm_year - 100 );
	
	
	ret = nrf_drv_twi_tx( &m_twi_master, RTC_ADDRESS, data, 8, false );
	if (NRF_SUCCESS != ret)
	{
		NRF_LOG_INFO("set timefailed");
		NRF_LOG_FLUSH();
		return;
	}

	rtc_clear_PON_BIT();

	data[0] = 0x00;
	data[1] = 0x01;
	ret = nrf_drv_twi_tx(&m_twi_master, RTC_ADDRESS, data, 2, true);
	if (NRF_SUCCESS != ret)
	{
		NRF_LOG_INFO("start clock failed");
		NRF_LOG_FLUSH();
		return;
	}
}

// Piezo
#define PIN_PIEZO 10
#include "app_pwm.h"
APP_PWM_INSTANCE(PWM1,1);				   // Create the instance "PWM1" using TIMER1.
static volatile bool ready_flag;			// A flag indicating PWM status.
void pwm_ready_callback(uint32_t pwm_id)	// PWM callback function
{
	ready_flag = true;
}
//TODO: Let's make the pizeo tones non blocking
void beep_speaker(int duration_ms, int duty_haha_duty)
{
	while (app_pwm_channel_duty_set(&PWM1, 0, duty_haha_duty) == NRF_ERROR_BUSY){}
	nrf_delay_ms(duration_ms);
	while (app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY){}
}

// Button input
#include "nrf_gpio.h"
#define PIN_BUTTON 9
#define isButtonPressed !nrf_gpio_pin_read(PIN_BUTTON)

//LITTLEFS
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
    .cache_size = 32,
    .lookahead_size = 16,
    .block_cycles = 500,
};

///////////////////

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
#define DEVICE_NAME   					"FreeSK8 Receiver"
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

#define MIN_CONN_INTERVAL			   MSEC_TO_UNITS(15, UNIT_1_25_MS)			 /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL			   MSEC_TO_UNITS(30, UNIT_1_25_MS)			 /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY				   0										   /**< Slave latency. */
#define CONN_SUP_TIMEOUT				MSEC_TO_UNITS(4000, UNIT_10_MS)			 /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)					   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)					  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT	3										   /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF					   0xDEADBEEF								  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#ifdef NRF52840_XXAA
#define UART_TX_BUF_SIZE				16384
#define UART_RX_BUF_SIZE				16384
#else
#define UART_TX_BUF_SIZE				2048
#define UART_RX_BUF_SIZE				8192
#endif

#define PACKET_VESC						0
#define PACKET_BLE						1

#ifdef NRF52840_XXAA
#if MODULE_BUILTIN
#define UART_RX							26
#define UART_TX							25
#define UART_TX_DISABLED				28
#define LED_PIN							27
#elif defined(MODULE_FREESK8)
#define UART_RX							36
#define UART_TX							35
#define UART_TX_DISABLED				19
#define LED_PIN							13
#else
#define UART_RX							11
#define UART_TX							8
#define UART_TX_DISABLED				25
#define LED_PIN							7
#endif
#else
#if MODULE_BUILTIN
#define UART_RX							6
#define UART_TX							7
#define UART_TX_DISABLED				25
#define EN_DEFAULT						1
#define LED_PIN							8
#else
#define UART_RX							7
#define UART_TX							6
#define UART_TX_DISABLED				25
#define EN_DEFAULT						1
#define LED_PIN							8
#endif
#endif


// Private variables
APP_TIMER_DEF(m_packet_timer);
APP_TIMER_DEF(m_nrf_timer);
APP_TIMER_DEF(m_logging_timer);

BLE_FUS_DEF(m_fus, NRF_SDH_BLE_TOTAL_LINK_COUNT);								   /**< BLE FUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);														   /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);															 /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);												 /**< Advertising module instance. */

static uint16_t   m_conn_handle		  = BLE_CONN_HANDLE_INVALID;				 /**< Handle of the current connection. */
static uint16_t   m_ble_fus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;			/**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
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

// Functions
void ble_printf(const char* format, ...);
static void set_enabled(bool en);

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

static void gap_params_init(void)
{
	uint32_t				err_code;
	ble_gap_conn_params_t   gap_conn_params;
	ble_gap_conn_sec_mode_t sec_mode;

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

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

static void start_advertising(void) {
	ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
	sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 8);
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
		start_advertising();
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
	switch (p_ble_evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		nrf_gpio_pin_set(LED_PIN);
		m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
		sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, 8);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		nrf_gpio_pin_clear(LED_PIN);
		m_conn_handle = BLE_CONN_HANDLE_INVALID;
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
		// Pairing not supported
		sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
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
		NRF_LOG_INFO("p_ble_evt->header.evt_id %ld", p_ble_evt->header.evt_id);
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

static void set_enabled(bool en) {
	m_is_enabled = en;

	if (m_is_enabled) {
		app_uart_close();
		m_uart_comm_params.tx_pin_no = UART_TX;
		uart_init();
		nrf_gpio_cfg_default(UART_TX_DISABLED);
	} else {
		app_uart_close();
		m_uart_comm_params.tx_pin_no = UART_TX_DISABLED;
		uart_init();
		nrf_gpio_cfg_default(UART_TX);
	}
}

static void uart_send_buffer(unsigned char *data, unsigned int len) {
	for (int i = 0;i < len;i++) {
		app_uart_put(data[i]);
	}
}

void rfhelp_send_data_crc(uint8_t *data, unsigned int len) {
	uint8_t buffer[len + 2];
	unsigned short crc = crc16((unsigned char*)data, len);
	memcpy(buffer, data, len);
	buffer[len] = (char)(crc >> 8);
	buffer[len + 1] = (char)(crc & 0xFF);
	esb_timeslot_set_next_packet(buffer, len + 2);
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
static void process_packet_vesc(unsigned char *data, unsigned int len) {
	if (data[0] == COMM_GET_VALUES){
		int32_t index = 1;
		esc_telemetry.temp_mos = buffer_get_float16(data,10.0,&index);
		esc_telemetry.temp_motor = buffer_get_float16(data,10.0,&index);
		esc_telemetry.current_motor = buffer_get_float32(data,100.0,&index);
		esc_telemetry.current_in = buffer_get_float32(data,10.0,&index);
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

		//TODO: Rate limit writing telemetry since real time data also requests this packet
		// 		Unless there is a fault!
		if (log_file_active)
		{
			//FileManager.writeToLogFile("${dtNow.toIso8601String().substring(0,21)},values,${telemetryPacket.v_in},${telemetryPacket.temp_motor},${telemetryPacket.temp_mos},${telemetryPacket.duty_now},${telemetryPacket.current_motor},${telemetryPacket.current_in},${telemetryPacket.rpm},${telemetryPacket.tachometer_abs},${telemetryPacket.vesc_id}\n");
            //2020-05-19T13:46:28.8, values, 12.9, -99.9, 29.0, 0.0, 0.0, 0.0, 0.0, 11884, 102
			char values_buffer[ 256 ] = {0};
			// Write fault codes to their own line
			if (esc_telemetry.fault_code != 0) {
				sprintf( values_buffer, "%s,fault,%s,%d,%d\n", datetimestring,  mc_fault_to_string(esc_telemetry.fault_code), esc_telemetry.fault_code, esc_telemetry.vesc_id);
				NRF_LOG_INFO( "File Bytes Written: %d", lfs_file_write(&lfs, &file, values_buffer, strlen(values_buffer)) );
				NRF_LOG_INFO("logline: %s",values_buffer);
			}
			// Write minimum telemetry data set
			sprintf( values_buffer, "%s,values,%0.1f,%0.1f,%0.1f,%0.1f,%0.1f,%0.1f,%0.1f,%d,%d\n", datetimestring, esc_telemetry.v_in, esc_telemetry.temp_motor, esc_telemetry.temp_mos, esc_telemetry.duty_now,esc_telemetry.current_motor,esc_telemetry.current_in, esc_telemetry.rpm, esc_telemetry.tachometer_abs,esc_telemetry.vesc_id);
			NRF_LOG_INFO( "File Bytes Written: %d", lfs_file_write(&lfs, &file, values_buffer, strlen(values_buffer)) );
			NRF_LOG_INFO("logline: %s",values_buffer);
			NRF_LOG_FLUSH();
		}

		++esc_rx_cnt;
#if HAS_DISPLAY
		if (esc_rx_cnt % 2 == 0) {
			Adafruit_GFX_fillCircle(120, 2, 2, WHITE);
			Adafruit_GFX_fillCircle(110, 2, 2, BLACK);
		} else {
			Adafruit_GFX_fillCircle(120, 2, 2, BLACK);
			Adafruit_GFX_fillCircle(110, 2, 2, WHITE);
		}
		SSD1306_display();
#endif
	}

	// Watch telemetry data to trigger logging
	// If we are logging now see if we should stop
	if (log_file_active) {
		if(esc_telemetry.v_in < 14.0) { //TODO: Specify appropriate low voltage detection limit
			log_file_stop();
			beep_speaker(50,50);
			NRF_LOG_INFO("Logging stopped due to power drop");
			NRF_LOG_FLUSH();
			send_status_packet();
		} else if (currentTime - lastTimeBoardMoved > 60) { //TODO: Specify appropriate duration to auto stop logging
			log_file_stop();
			NRF_LOG_INFO("Logging stopped due to inactivity");
			NRF_LOG_FLUSH();
			beep_speaker(50,50);
			send_status_packet();
		} else if (fabs(esc_telemetry.duty_now) > 0.01) { //TODO: Specify appropriate minimum duty cycle to initiate logging
			// We are moving while logging. Keep it up!
			lastTimeBoardMoved = currentTime;
		}
	}
	// We are not logging, see if we should start
	else if (fabs(esc_telemetry.duty_now) > 0.01) { //TODO: Specify appropriate minimum duty cycle to initiate logging
		log_file_start();
		NRF_LOG_INFO("Logging started automatically");
		NRF_LOG_FLUSH();
		send_status_packet();
	}

	// Finish packet processing
	if (data[0] == COMM_EXT_NRF_ESB_SET_CH_ADDR) {
		esb_timeslot_set_ch_addr(data[1], data[2], data[3], data[4]);
	} else if (data[0] == COMM_EXT_NRF_ESB_SEND_DATA) {
		rfhelp_send_data_crc(data + 1, len - 1);
	} else if (data[0] == COMM_EXT_NRF_SET_ENABLED) {
		set_enabled(data[1]);
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

static void esb_timeslot_data_handler(void *p_data, uint16_t length) {
	if (m_other_comm_disable_time == 0) {
		uint8_t buffer[length + 1];
		buffer[0] = COMM_EXT_NRF_ESB_RX_DATA;
		memcpy(buffer + 1, p_data, length);
		CRITICAL_REGION_ENTER();
		packet_send_packet(buffer, length + 1, PACKET_VESC);
		CRITICAL_REGION_EXIT();
	}
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

static void nrf_timer_handler(void *p_context) {
	(void)p_context;

	if (m_other_comm_disable_time == 0) {
		uint8_t buffer[1];
		buffer[0] = COMM_EXT_NRF_PRESENT;
		CRITICAL_REGION_ENTER();
		packet_send_packet(buffer, 1, PACKET_VESC);
		CRITICAL_REGION_EXIT();
	}
}

static void logging_timer_handler(void *p_context) {
	(void)p_context;

	rtc_get_time();

	char dt_string[64] = {0};
	strftime(dt_string, 64, "%Y-%m-%dT%H:%M:%S", tmTime);
	for(int i=0; i<strlen(dt_string)+1; ++i)
	{
		datetimestring[i] = dt_string[i];
	}

	NRF_LOG_INFO("This would be a nice time to perform logging %s", datetimestring);
	NRF_LOG_FLUSH();

	static unsigned char telemetryPacket[] = {0x02, 0x01, 0x04, 0x40, 0x84, 0x03};
	uart_send_buffer(telemetryPacket, 6);
}

void display_file_count(void)
{
	NRF_LOG_INFO("display_file_count");
	NRF_LOG_FLUSH();
#if HAS_DISPLAY
	Adafruit_GFX_setCursor(0,8);
	sprintf(display_text_buffer,"FS ready: %d files   ", lfs_file_count);
	Adafruit_GFX_print(display_text_buffer);
	SSD1306_display();
#endif
}

//TODO: BUG: We need i2c for RTC. Cannot wrap the following in HAS_DISPLAY...
#if HAS_DISPLAY
void i2c_oled_comm_handle(uint8_t hdl_address, uint8_t *hdl_buffer, size_t hdl_buffer_size)
{
	nrf_drv_twi_tx(&m_twi_master, hdl_address, hdl_buffer, hdl_buffer_size, false);
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
	   .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
	   .clear_bus_init	 = true
	};

	ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

	if (NRF_SUCCESS == ret)
	{
		nrf_drv_twi_enable(&m_twi_master);
	}

	return ret;
}
#endif




////////////////QSPI

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

}

void qspiInit()
{
	uint32_t err_code=0;

	err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();

	NRF_LOG_INFO("QSPI initializing");
	NRF_LOG_FLUSH();

	nrf_drv_qspi_config_t config = NRF_DRV_QSPI_DEFAULT_CONFIG;

	err_code = nrf_drv_qspi_init(&config, NULL, NULL);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("QSPI driver initialized");

	configure_memory();

	NRF_LOG_FLUSH();
}

int log_file_stop()
{
	if (log_file_active)
	{
#if HAS_DISPLAY
		Adafruit_GFX_setCursor(0,16);
		sprintf(display_text_buffer,"Logging inactive");
		Adafruit_GFX_print(display_text_buffer);
		SSD1306_display();
#endif
		log_file_active = false;
		//TODO: Experienced a lockup here. No remote connected. BLE connected. lfs_file_close hangs after performing cat
		lfs_unmount(&lfs);
		lfs_mount(&lfs, &cfg);
		//TODO: Testing a re-mount here
		//NOTE: The crash is no longer repeatable with re-mounting and the file saves successfully even though we close
		// the file handle after messing with the state of the lfs object/filesystem. What gives?
		return lfs_file_close(&lfs, &file);
	}
	return -1;
}

void log_file_start()
{
	lastTimeBoardMoved = currentTime;

	if (log_file_active)
	{
		log_file_stop();
	}

	char filename[64];
	strftime( filename, 64, "/FreeSK8Logs/%Y-%m-%dT%H:%M:%S", tmTime );

	NRF_LOG_INFO("Creating log file: %s",filename);
	NRF_LOG_FLUSH();

	//TODO: Experienced a lockup here. No remote connected. BLE connected. lfs_file_open hangs
	lfs_unmount(&lfs);
	lfs_mount(&lfs, &cfg);
	//TODO: Testing a re-mount here like what worked in rm command

	if ( lfs_file_open(&lfs, &file, filename, LFS_O_WRONLY | LFS_O_CREAT) >= 0)
	{
		NRF_LOG_INFO("log_file_active");
		NRF_LOG_FLUSH();
		log_file_active = true;
		++lfs_file_count;
		display_file_count();
		NRF_LOG_INFO("TODO: BUG: We crashed before here?");
		NRF_LOG_FLUSH();
#if HAS_DISPLAY
		Adafruit_GFX_setCursor(0,16);
		sprintf(display_text_buffer,"Logging active  ");
		Adafruit_GFX_print(display_text_buffer);
		SSD1306_display();
#endif
	}
}

void update_status_packet(char * buffer)
{
	//TODO: Possible status variables: isLogging, bytesSaved, fileStartTime, fileCount
	sprintf(buffer, "status,OK,%d,", log_file_active);
}


void littlefsInit()
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
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }
    NRF_LOG_INFO("LittleFS initialized");
    NRF_LOG_FLUSH();

    // read current count  
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
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
		// TODO: Cleaning 0 bytes files from system during debugging
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
	Adafruit_GFX_setCursor(0,8);
	sprintf(display_text_buffer,"FS ready: %d files", lfs_file_count);
	Adafruit_GFX_print(display_text_buffer);
	Adafruit_GFX_setCursor(0,16);
	sprintf(display_text_buffer,"Logging inactive");
	Adafruit_GFX_print(display_text_buffer);
	SSD1306_display();
#endif
}

/////////////////////

void pwm_init(void)  
{
	app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(240L, PIN_PIEZO);
	pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

	APP_ERROR_CHECK(app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback));

	app_pwm_enable(&PWM1);
}

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

			beep_speaker(40, 50);

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
				beep_speaker(40,10);
			} else if (d_tumble_t == 6) {
				beep_speaker(200,90);
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

		SSD1306_display();
		nrf_delay_ms(16);
	}
}
//You didn't listen when I said don't ask, did you?
#endif

int main(void) {

	nrf_gpio_cfg_input(PIN_BUTTON,NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_output(LED_PIN);
	//Flash LED to show sign of life //TODO: remove this
	nrf_gpio_pin_set(LED_PIN);
	nrf_delay_ms(1000);


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


///////////////////Display test

#if HAS_DISPLAY
	ret_code_t err_code = twi_master_init();
	APP_ERROR_CHECK(err_code);

	SSD1306_begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
	Adafruit_GFX_init(SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT, SSD1306_drawPixel);

	SSD1306_clearDisplay();
	SSD1306_display();


	char freetitle[] = "FreeSK8";
	Adafruit_GFX_setTextSize(1);
	Adafruit_GFX_setTextColor(1,0);
	Adafruit_GFX_print(freetitle);
	SSD1306_display();

#endif

////////////////////////////
	// Allocate tmTime
	time ( &currentTime );
	tmTime = localtime ( &currentTime );
	
	rtc_battery_charge();
	
	
	nrf_gpio_pin_clear(LED_PIN);

////////////////////////////

	// Test piezo
	pwm_init();

	//TODO: RREMOVE: Sweep 50 to 100% duty cycle
	for (uint16_t i = 50; i < 100; ++i)
	{
		ready_flag = false;
		/* Set the duty cycle - keep trying until PWM is ready... */
		while (app_pwm_channel_duty_set(&PWM1, 0, i) == NRF_ERROR_BUSY){}
#if HAS_DISPLAY
		Adafruit_GFX_setCursor(0,8);
		sprintf(display_text_buffer,"%d%% ", i);
		Adafruit_GFX_print(display_text_buffer);
		SSD1306_display();
#endif
		/* ... or wait for callback. */
		//while (!ready_flag);
		//APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 0, i));
		nrf_delay_ms(25);
	}
	while (app_pwm_channel_duty_set(&PWM1, 0, 0) == NRF_ERROR_BUSY){}


////////////////////////////
#if HAS_DISPLAY
	if(isButtonPressed)
	{
		play_game();
	}
#endif
////////////////////////////

	//QSPI Testing
	qspiInit();
	littlefsInit();

/////////////////////////////

	uart_init();
	app_timer_init();
	nrf_pwr_mgmt_init();
	ble_stack_init();
	gap_params_init();
	gatt_init();
	services_init();
	advertising_init();
	conn_params_init();
	command_interface_init(&ble_send_logbuffer, &lfs);


	(void)set_enabled;

	packet_init(uart_send_buffer, process_packet_vesc, PACKET_VESC);
	packet_init(ble_send_buffer, process_packet_ble, PACKET_BLE);

	app_timer_create(&m_packet_timer, APP_TIMER_MODE_REPEATED, packet_timer_handler);
	app_timer_start(m_packet_timer, APP_TIMER_TICKS(1), NULL);

	app_timer_create(&m_nrf_timer, APP_TIMER_MODE_REPEATED, nrf_timer_handler);
	app_timer_start(m_nrf_timer, APP_TIMER_TICKS(1000), NULL);

	app_timer_create(&m_logging_timer, APP_TIMER_MODE_REPEATED, logging_timer_handler);
	app_timer_start(m_logging_timer, APP_TIMER_TICKS(1000), NULL);

	esb_timeslot_init(esb_timeslot_data_handler);
	esb_timeslot_sd_start();

#ifdef NRF52840_XXAA
	app_usbd_power_events_enable();
#endif

	start_advertising();

	for (;;) {
#ifdef NRF52840_XXAA
		while (app_usbd_event_queue_process()){}
#endif

		if (m_uart_error) {
			app_uart_close();
			uart_init();
			packet_reset(PACKET_VESC);
			m_uart_error = false;
		}

		uint8_t byte;
		while (app_uart_get(&byte) == NRF_SUCCESS) {
			packet_process_byte(byte, PACKET_VESC);
		}

		sd_app_evt_wait();
	}
}
