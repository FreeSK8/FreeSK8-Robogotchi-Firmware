#include "command_interface.h"

#include <time.h>
#include <stdio.h>
#include "stdint.h"
#include "string.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_ble_gatt.h"
#include "user_cfg.h"

extern uint16_t melody_snooze_seconds;
extern uint16_t lfs_file_count;
extern void display_file_count(void);
extern uint8_t lfs_free_space_check(void);
extern int log_file_stop();
extern void log_file_start();
extern void update_status_packet(char * buffer);
extern uint16_t create_fault_packet(char * buffer);
extern uint16_t m_ble_fus_max_data_len;

extern void user_cfg_set(bool restart_telemetry_timer);
extern struct gotchi_configuration gotchi_cfg_user;

extern bool sync_in_progress;
extern struct lfs_config cfg;
extern volatile bool update_rtc;

extern volatile bool log_file_active;

static bool cat_in_progress = false;
static lfs_file_t file_command_interface;
static uint8_t lfs_file_buf[4096]; // Must be cache size
static struct lfs_file_config lfs_file_config;
static lfs_dir_t directory;
static int32_t bytes_sent = -1; //file.ctz.size;

static void (*m_ble_tx_logbuffer)(unsigned char *data, unsigned int len);
static lfs_t *m_lfs;

static int command_input_index = 0;
static char command_input_buffer[ 128 ] = { 0 };
static unsigned char command_response_buffer[256];

static void (*m_update_time)(int syear, int smonth, int sday, int shour, int sminute, int ssecond);

void command_interface_init(void (*ble_send_logbuffer)(unsigned char *, unsigned int), lfs_t *lfs, void (*update_time)(int, int, int, int, int, int))
{
    ASSERT(lfs != NULL);
    ASSERT(ble_send_logbuffer != NULL);
    m_lfs = lfs;
    m_ble_tx_logbuffer = ble_send_logbuffer;
    m_update_time = update_time;

    // Prepare the static file buffer
	memset(&lfs_file_config, 0, sizeof(struct lfs_file_config));
	lfs_file_config.buffer = lfs_file_buf;
	lfs_file_config.attr_count = 0;
}

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
            if (sync_in_progress)
            {
                command_interface_continue_transfer( command_input_buffer );
            }
            else
            {
                NRF_LOG_WARNING("ACK received while !sync_in_progress");
                NRF_LOG_FLUSH();
            }
        }
        else if( strncmp(&command_input_buffer[strlen(command_input_buffer)-4], "nack", 4) == 0)
        {
            NRF_LOG_INFO("NACK handler >%s<", command_input_buffer);
            NRF_LOG_FLUSH();
            if (sync_in_progress)
            {
                if (cat_in_progress)
                {
                    // For a cat command we want to seek to the byte the client last received
                    if(strncmp(command_input_buffer, "cat", 3) == 0)
                    {
                        // Set the file position to what we've received on the client side
                        lfs_file_seek(m_lfs, &file_command_interface, atoi(command_input_buffer+4), LFS_SEEK_SET);
                    }
                }
                // Continue to send data to the client
                command_interface_continue_transfer(command_input_buffer);
            }
            else
            {
                NRF_LOG_WARNING("NACK received while !sync_in_progress");
                NRF_LOG_FLUSH();
            }
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
            //TODO: guarding settime by log_file_active to prevent time traveling. Needs better solution
            if (!log_file_active)
            {
                char* timepointer = command_input_buffer + 8;
                int syear, smonth, sday, shour, sminute, ssecond;
                sscanf( timepointer, "%d:%d:%dT%d:%d:%d", &syear, &smonth, &sday, &shour, &sminute, &ssecond );

                NRF_LOG_INFO("command_interface: settime command received: %s", command_input_buffer + 8);
                NRF_LOG_FLUSH();

                // Update time in memory
                m_update_time(syear, smonth, sday, shour, sminute, ssecond);

                // Update time on RTC
                update_rtc = true;
            }
            else
            {
                NRF_LOG_WARNING("command_interface: set time command ignored while logging");
                NRF_LOG_FLUSH();
            }
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
            if (cat_in_progress)
            {
                lfs_file_close(m_lfs, &file_command_interface);
            }
            if(lfs_file_opencfg(m_lfs, &file_command_interface, filename, LFS_O_RDONLY, &lfs_file_config) >= 0)
            {
                lfs_file_seek(m_lfs, &file_command_interface, 0, LFS_SEEK_SET); //TODO: Testing rewind on open, necessary?
                sprintf((char *)command_response_buffer, "cat,%s", filename);
                m_ble_tx_logbuffer(command_response_buffer, (size_t)strlen((const char *)command_response_buffer));
                bytes_sent = 0;
                cat_in_progress = true;
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
                lfs_free_space_check();
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
        else if(strncmp(command_input_buffer, "faults", 6) == 0)
        {
            NRF_LOG_INFO("command_interface: faults command received");
            uint16_t packet_size = create_fault_packet((char *)command_response_buffer);
            m_ble_tx_logbuffer(command_response_buffer, packet_size);
        }
        else if(strncmp(command_input_buffer, "dfumode", 7) == 0)
        {
            // Set flag for DFU mode and reset device
            NRF_POWER->GPREGRET = 0xB1;
            sd_nvic_SystemReset();
        }
        else if(strncmp(command_input_buffer, "version", 7) == 0)
        {
            sprintf((char *)command_response_buffer, "version,0.10.2,beta");
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "getcfg", 6) == 0)
        {
            sprintf((char *)command_response_buffer, "getcfg,%ld,%d,%0.2f,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%0.1f,%0.1f,%0.1f,%d,%d,%d",
                gotchi_cfg_user.cfg_version,
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
                gotchi_cfg_user.timezone_hour_offset,
                gotchi_cfg_user.timezone_minute_offset
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
            gotchi_cfg.timezone_hour_offset = 0;
            gotchi_cfg.timezone_minute_offset = 0;
            gotchi_cfg.cfg_version = 0;
            char * usr_cfg = command_input_buffer + 7;
            NRF_LOG_INFO("setcfg command received: %s", usr_cfg);
            NRF_LOG_FLUSH();
            
            char *field = strtok(usr_cfg, ","); NRF_LOG_INFO("field: %s", field); NRF_LOG_FLUSH();
            gotchi_cfg.cfg_version = atoi(field);
            field = strtok(NULL, ",");
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
            gotchi_cfg.timezone_hour_offset = atoi(field);
            field = strtok(NULL, ",");
            gotchi_cfg.timezone_minute_offset = atoi(field);

            // Check if parsed version matches expectations
            if(gotchi_cfg.cfg_version == gotchi_cfg_user.cfg_version)
            {
                bool restart_telemetry_timer = gotchi_cfg.log_interval_hz != gotchi_cfg_user.log_interval_hz;
                log_file_stop(); // Stop logging before changing settings
                gotchi_cfg_user = gotchi_cfg;
                user_cfg_set(restart_telemetry_timer);
                sprintf((char *)command_response_buffer, "setcfg,OK");
            }
            else
            {
                sprintf((char *)command_response_buffer, "setcfg,ERR,%ld", gotchi_cfg.cfg_version);
            }

            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }
        else if(strncmp(command_input_buffer, "syncstart", 9) == 0)
        {
            // Sync process is starting
            NRF_LOG_INFO("syncstart command received");
            NRF_LOG_FLUSH();
            if (!sync_in_progress)
            {
                sync_in_progress = true;
                NRF_LOG_INFO("sync_in_progress is now true");
                NRF_LOG_FLUSH();
            } else {
                NRF_LOG_INFO("sync_in_progress was already true");
                NRF_LOG_FLUSH();
            }
        }
        else if(strncmp(command_input_buffer, "syncstop", 8) == 0)
        {
            // Sync process was aborted by the user
            NRF_LOG_INFO("syncstop command received");
            NRF_LOG_FLUSH();
            if (sync_in_progress)
            {
                sync_in_progress = false;
            } else {
                NRF_LOG_INFO("sync_in_progress was false");
                NRF_LOG_FLUSH();
            }
            if (cat_in_progress)
            {
                int close_result = lfs_file_close(m_lfs, &file_command_interface);
                NRF_LOG_INFO("lfs close result: %d", close_result);
                NRF_LOG_FLUSH();
                cat_in_progress = false;
            }
        }
        else if(strncmp(command_input_buffer, "snooze,", 7) == 0)
        {
            // Set melody snooze duration
            NRF_LOG_INFO("snooze command received: %s seconds", command_input_buffer+7);
            NRF_LOG_FLUSH();
            melody_snooze_seconds = atoi(command_input_buffer+7);
        }
        else if(strncmp(command_input_buffer, "roboid", 6) == 0)
        {
            // Return robogotchi unique ID
            NRF_LOG_INFO("robogotchi ID command received");
            NRF_LOG_FLUSH();
            sprintf((char *)command_response_buffer, "roboid,%02lx%02lx", NRF_FICR->DEVICEID[0], NRF_FICR->DEVICEID[1]);
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
    else if(strncmp(command_input_buffer, "cat", 3) == 0)
    {
        if (!cat_in_progress)
        {
            NRF_LOG_WARNING("CAT ACK/NACK received while !cat_in_progress");
            NRF_LOG_FLUSH();
            return;
        }
        NRF_LOG_INFO("command_interface_continue_transfer(TRANSFER_MODE_CAT)");
        NRF_LOG_FLUSH();

        if(bytes_sent >= file_command_interface.ctz.size)
        {
            m_ble_tx_logbuffer((unsigned char *)"cat,complete", strlen("cat,complete"));

            NRF_LOG_INFO("sending cat,complete");
            NRF_LOG_FLUSH();

            //NOTE: Leaving file open in case the client needs to ACK/NACK
            //NOTE: File will be closed upon request of the next file or syncstop
            /*
            int close_result = lfs_file_close(m_lfs, &file_command_interface);
            NRF_LOG_INFO("cat close result: %d", close_result);
            NRF_LOG_FLUSH();
            cat_in_progress = false;
            */
        }
        else if(bytes_sent < file_command_interface.ctz.size)
        {
            int32_t read_response = lfs_file_read(m_lfs, &file_command_interface, &command_response_buffer, m_ble_fus_max_data_len);
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
