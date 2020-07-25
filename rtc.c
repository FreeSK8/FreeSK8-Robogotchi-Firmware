#include "rtc.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include <time.h>

extern const nrf_drv_twi_t m_twi_master;
extern struct tm * tmTime;
extern time_t currentTime;

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
