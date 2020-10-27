#include "command_interface.h"

#include <time.h>
#include <stdio.h>
#include "stdint.h"
#include "string.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_ble_gatt.h"
#include "user_cfg.h"

extern uint16_t lfs_file_count;
extern void display_file_count(void);
extern int log_file_stop();
extern void log_file_start();
extern void update_status_packet(char * buffer);
extern uint16_t m_ble_fus_max_data_len;

extern void user_cfg_set(void);
extern struct gotchi_configuration gotchi_cfg_user;

static lfs_file_t file;
static lfs_dir_t directory;
static int32_t bytes_sent = -1; //file.ctz.size;

void (*m_ble_tx_logbuffer)(unsigned char *data, unsigned int len);
static lfs_t *m_lfs;

static int command_input_index = 0;
static char command_input_buffer[ 128 ] = { 0 };
static unsigned char command_response_buffer[512];

extern bool sync_in_progress;

void command_interface_init(void (*ble_send_logbuffer)(unsigned char *, unsigned int), lfs_t *lfs)
{
    ASSERT(lfs != NULL);
    ASSERT(ble_send_logbuffer != NULL);
    m_lfs = lfs;
    m_ble_tx_logbuffer = ble_send_logbuffer;
}

extern time_t currentTime;
extern struct tm * tmTime;
extern struct lfs_config cfg;
extern volatile bool update_rtc;

void command_interface_process_byte(char incoming)
{
    command_input_buffer[ command_input_index++ ] = incoming;

    if(command_input_index >= sizeof(command_input_buffer))
    {
        command_input_index = 0;
        command_input_buffer[ command_input_index ] = 0;

        return;
    }

    if( incoming == '~' ) //TODO: ~null~ terminate after debugging
    {
        command_input_buffer[ strlen( command_input_buffer ) - 1 ] = 0;

        if( strncmp( &command_input_buffer[ strlen( command_input_buffer ) - 4 ], ",ack", 4 ) == 0 )
        {
            NRF_LOG_INFO("ACK handler >%s<", command_input_buffer);
            NRF_LOG_FLUSH();
            command_interface_continue_transfer( command_input_buffer );
        }
        else if(strncmp(command_input_buffer, "log", 3) == 0)
        {
            if(strncmp(command_input_buffer + 3, "start", 5) == 0)
            {
                NRF_LOG_INFO("command_interface: logstart command received");
                NRF_LOG_FLUSH();
                log_file_start();
            }
            else if(strncmp(command_input_buffer + 3, "stop", 4) == 0)
            {
                NRF_LOG_INFO("command_interface: logstop command received");
                NRF_LOG_FLUSH();
                log_file_stop();
            }

            // Sending status packet to update the client of command result
            update_status_packet((char *)command_response_buffer);
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "settime ", 8) == 0)
        {
            char* timepointer = command_input_buffer + 8;
            int syear, smonth, sday, shour, sminute, ssecond;
            sscanf( timepointer, "%d:%d:%dT%d:%d:%d", &syear, &smonth, &sday, &shour, &sminute, &ssecond );

            NRF_LOG_INFO("command_interface: settime command received: %s", command_input_buffer + 8);
            NRF_LOG_FLUSH();

            // Update time in memory
            tmTime->tm_year = syear - 1900;
            tmTime->tm_mon = smonth - 1;
            tmTime->tm_mday = sday;
            tmTime->tm_hour = shour;
            tmTime->tm_min = sminute;
            tmTime->tm_sec = ssecond;
            currentTime = mktime(tmTime);

            // Update time on RTC
            update_rtc = true;
        }
        else if(strncmp(command_input_buffer, "ls", 2) == 0)
        {
            // open the log directory
            lfs_dir_open(m_lfs, &directory, "/FreeSK8Logs");

            sprintf((char *)command_response_buffer, "ls,/FreeSK8Logs");
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
          
            NRF_LOG_INFO("command_interface: ls (list files) command received: %s", command_response_buffer);
            NRF_LOG_FLUSH();
        }
        else if(strncmp(command_input_buffer, "fc", 2) == 0)
        {
            NRF_LOG_INFO("command_interface: fc (file count) command received");
            NRF_LOG_FLUSH();
            uint16_t file_count = 0;
            uint32_t file_bytes_total = 0;
            // open the log directory
            lfs_dir_open(m_lfs, &directory, "/FreeSK8Logs");
          
            struct lfs_info entryinfo;
            while(lfs_dir_read(m_lfs,&directory,&entryinfo))
            {
                if(entryinfo.type == LFS_TYPE_REG)
                {
                    ++file_count;
                    file_bytes_total += entryinfo.size;
                }
            }
            lfs_dir_close(m_lfs, &directory);

            NRF_LOG_INFO("File Count: %d, Total Size: %d bytes", file_count, file_bytes_total);
            NRF_LOG_FLUSH();
            // returning fc,filecount,totalsize
            sprintf((char *)command_response_buffer, "fc,%d,%ld", file_count, file_bytes_total);
            // send response over BLE
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "cat", 3) == 0 && strlen(command_input_buffer) > 4)
        {
            NRF_LOG_INFO("command_interface: cat (concatenate) command received");
            char filename[19+13+1] = "/FreeSK8Logs/"; //2020-06-20T20:26:40
            strcat(filename,&command_input_buffer[4]);

            NRF_LOG_INFO("filename %s", filename);
            NRF_LOG_FLUSH();
            if(lfs_file_open(m_lfs, &file, filename, LFS_O_RDONLY) >= 0)
            {
                sprintf((char *)command_response_buffer, "cat,%s", filename);
                m_ble_tx_logbuffer(command_response_buffer, (size_t)strlen((const char *)command_response_buffer));
                bytes_sent = 0;
                sync_in_progress = true;
            }
        }
        else if(strncmp(command_input_buffer, "rm ", 3) == 0 && strlen(command_input_buffer) > 3)
        {
            NRF_LOG_INFO("command_interface: rm (remove) command received");

            char filename[32] = {0}; //TODO: Max length is expected to be 19 but undetermined atm
            strcpy(filename, &command_input_buffer[3]);

            char filepath[64] = "/FreeSK8Logs/";
            strcat(filepath, filename);
            NRF_LOG_INFO("filepath %s", filepath);
            NRF_LOG_FLUSH();

            int remove_response = lfs_remove(m_lfs,filepath);
            NRF_LOG_INFO("lfs_remove():remove_response: %d", remove_response);
            NRF_LOG_FLUSH();
            if (remove_response >= 0)
            {
                if (lfs_file_count > 0) --lfs_file_count; //TODO: BUG: Sync with Erase in Mobile app will not actually erase but counter decrements. This makes file_count inaccurate desipte checking response from lfs_remove
                display_file_count();
                sprintf((char *)command_response_buffer, "rm,OK,%s", filename);
            } 
            else 
            {
                sprintf((char *)command_response_buffer, "rm,FAIL,%s,%d", filename, remove_response);
            }

            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "status", 6) == 0)
        {
            NRF_LOG_INFO("command_interface: status command received");
            update_status_packet((char *)command_response_buffer);
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "dfumode", 7) == 0)
        {
            // Set flag for DFU mode and reset device
            NRF_POWER->GPREGRET = 0xB1;
            sd_nvic_SystemReset();
        }
        else if(strncmp(command_input_buffer, "version", 7) == 0)
        {
            sprintf((char *)command_response_buffer, "version,0.3.4,alpha");
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "getcfg", 6) == 0)
        {
            sprintf((char *)command_response_buffer, "getcfg,%d,%0.2f,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%0.1f,%0.1f,%0.1f,%d,%ld",
                gotchi_cfg_user.log_auto_stop_idle_time,
                gotchi_cfg_user.log_auto_stop_low_voltage,
                gotchi_cfg_user.log_auto_start_erpm,
                gotchi_cfg_user.log_interval_hz,
                gotchi_cfg_user.log_auto_erase_when_full,
                gotchi_cfg_user.multi_esc_mode,
                gotchi_cfg_user.multi_esc_ids[0],
                gotchi_cfg_user.multi_esc_ids[1],
                gotchi_cfg_user.multi_esc_ids[2],
                gotchi_cfg_user.multi_esc_ids[3],
                gotchi_cfg_user.gps_baud_rate,
                gotchi_cfg_user.alert_low_voltage,
                gotchi_cfg_user.alert_esc_temp,
                gotchi_cfg_user.alert_motor_temp,
                gotchi_cfg_user.alert_storage_at_capacity,
                gotchi_cfg_user.cfg_version
            );
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "setcfg", 6) == 0)
        {
            struct gotchi_configuration gotchi_cfg;
            gotchi_cfg.log_auto_stop_idle_time = 0;
            gotchi_cfg.log_auto_stop_low_voltage = 0;
            gotchi_cfg.log_auto_start_erpm = 0;
            gotchi_cfg.log_interval_hz = 0;
            gotchi_cfg.log_auto_erase_when_full = 0;
            gotchi_cfg.multi_esc_mode = 0;
            gotchi_cfg.multi_esc_ids[0] = 0;
            gotchi_cfg.multi_esc_ids[1] = 0;
            gotchi_cfg.multi_esc_ids[2] = 0;
            gotchi_cfg.multi_esc_ids[3] = 0;
            gotchi_cfg.gps_baud_rate = 0;
            gotchi_cfg.alert_low_voltage = 0.0;
            gotchi_cfg.alert_esc_temp = 0.0;
            gotchi_cfg.alert_motor_temp = 0.0;
            gotchi_cfg.alert_storage_at_capacity = 0;
            gotchi_cfg.cfg_version = 0;
            char * usr_cfg = command_input_buffer + 7;
            NRF_LOG_INFO("setcfg command received: %s", usr_cfg);
            NRF_LOG_FLUSH();
            
            char *field = strtok(usr_cfg, ","); NRF_LOG_INFO("field: %s", field); NRF_LOG_FLUSH();
            gotchi_cfg.log_auto_stop_idle_time = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.log_auto_stop_low_voltage = atof(field);
            field = strtok(NULL, ",");
            gotchi_cfg.log_auto_start_erpm = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.log_interval_hz = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.log_auto_erase_when_full = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.multi_esc_mode = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.multi_esc_ids[0] = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.multi_esc_ids[1] = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.multi_esc_ids[2] = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.multi_esc_ids[3] = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.gps_baud_rate = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.alert_low_voltage = atof(field);
            field = strtok(NULL, ",");
            gotchi_cfg.alert_esc_temp = atof(field);
            field = strtok(NULL, ",");
            gotchi_cfg.alert_motor_temp = atof(field);
            field = strtok(NULL, ",");
            gotchi_cfg.alert_storage_at_capacity = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.cfg_version = atoi(field);
#ifdef P00PIES
            static char testpoo[128] = {0};
            sprintf(testpoo,"%d,%0.2f,%0.2f,%d,%d,%d,%d,%d,%d,%ld,%ld",
                gotchi_cfg.log_auto_stop_idle_time,
                gotchi_cfg.log_auto_stop_low_voltage,
                gotchi_cfg.log_auto_start_duty_cycle,
                gotchi_cfg.log_interval_hz,
                gotchi_cfg.multi_esc_mode,
                gotchi_cfg.multi_esc_ids[0],
                gotchi_cfg.multi_esc_ids[1],
                gotchi_cfg.multi_esc_ids[2],
                gotchi_cfg.multi_esc_ids[3],
                gotchi_cfg.gps_baud_rate,
                gotchi_cfg.cfg_version);
            NRF_LOG_INFO("Parsed Config >%s<", testpoo);
#endif
            if(gotchi_cfg.cfg_version == gotchi_cfg_user.cfg_version)
            {
                log_file_stop(); // Stop logging before changing settings
                gotchi_cfg_user = gotchi_cfg;
                user_cfg_set();
                sprintf((char *)command_response_buffer, "setcfg,OK");
            }
            else
            {
                sprintf((char *)command_response_buffer, "setcfg,ERR,%ld", gotchi_cfg.cfg_version);
            }

            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }

        memset(command_input_buffer, 0, sizeof(command_input_buffer));
        command_input_index = 0;
    }
}

void command_interface_continue_transfer(char* command)
{
    if(strncmp(command_input_buffer, "ls", 2) == 0)
    {
        struct lfs_info entryinfo;
        int readresp = lfs_dir_read(m_lfs,&directory,&entryinfo);
        if(readresp)
        {
            NRF_LOG_INFO("command_interface_continue_transfer(TRANSFER_MODE_LS) %s,%ld", entryinfo.name,  entryinfo.size);
            NRF_LOG_FLUSH();
            //NRF_LOG_INFO("%s,%d,bytes,%s", entryinfo.type == LFS_TYPE_REG ? "FILE" : "DIR ", entryinfo.size, entryinfo.name);
            //NRF_LOG_FLUSH();
            switch(entryinfo.type)
            {
                case LFS_TYPE_REG:
                {
                    // returning filename,filesize
                    sprintf((char *)command_response_buffer, "ls,FILE,%s,%ld", entryinfo.name,  entryinfo.size);
                    // send response over BLE
                    m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
                    break;
                }
                case LFS_TYPE_DIR:
                {
                    // returning filename,filesize
                    sprintf((char *)command_response_buffer, "ls,DIR,%s,%ld", entryinfo.name,  entryinfo.size);
                    // send response over BLE
                    m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
                    break;
                }
            }
        }
        else
        {
            lfs_dir_close(m_lfs, &directory);
            sprintf((char *)command_response_buffer, "ls,complete");
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
    }
    else if(strncmp(command_input_buffer, "cat", 2) == 0)
    {
        NRF_LOG_INFO("command_interface_continue_transfer(TRANSFER_MODE_CAT)");
        NRF_LOG_FLUSH();

        if(bytes_sent >= file.ctz.size)
        {
            m_ble_tx_logbuffer((unsigned char *)"cat,complete", strlen("cat,complete"));

            NRF_LOG_INFO("finished cat");
            NRF_LOG_FLUSH();

            int close_result = lfs_file_close(m_lfs, &file);

            NRF_LOG_INFO("cat close result: %d", close_result);
            NRF_LOG_FLUSH();

            sync_in_progress = false;
        }
        else if(bytes_sent < file.ctz.size)
        {
            int32_t read_response = lfs_file_read(m_lfs, &file, &command_response_buffer, m_ble_fus_max_data_len);
            //NRF_LOG_INFO("read_response = %d", read_response);
            //NRF_LOG_FLUSH();
            if(read_response > 0)
            {
                m_ble_tx_logbuffer(command_response_buffer, read_response);
                bytes_sent += read_response;
                NRF_LOG_INFO("Sent %d bytes", read_response);
                NRF_LOG_FLUSH();
            }
        }
    }
}
