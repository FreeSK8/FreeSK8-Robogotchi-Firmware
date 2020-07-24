#include "command_interface.h"

#include <time.h>
#include <stdio.h>
#include "stdint.h"
#include "string.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_ble_gatt.h"

extern uint16_t lfs_file_count;
extern void display_file_count(void);
extern int log_file_stop();
extern void log_file_start();
extern void update_status_packet(char * buffer);


static lfs_file_t file;
static lfs_dir_t directory;
static int32_t bytes_sent = -1; //file.ctz.size;

void (*m_ble_tx_logbuffer)(unsigned char *data, unsigned int len);
static lfs_t *m_lfs;

static int command_input_index = 0;
static char command_input_buffer[ 128 ] = { 0 };
static unsigned char command_response_buffer[ 244 ];

void command_interface_init(void (*ble_send_logbuffer)(unsigned char *, unsigned int), lfs_t *lfs)
{
    ASSERT(lfs != NULL);
    ASSERT(ble_send_logbuffer != NULL);
    m_lfs = lfs;
    m_ble_tx_logbuffer = ble_send_logbuffer;
}

extern struct tm * tmTime;
extern struct lfs_config cfg;
extern void rtc_set_time( int year, int month, int day, int hour, int minute, int second );

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

            rtc_set_time( syear - 1900, smonth, sday, shour, sminute, ssecond );
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

            //TODO: testing un-mount to fix random hangs. Might not be necessary
            lfs_unmount( m_lfs );
            lfs_mount(m_lfs, &cfg);

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
            NVIC_SystemReset();
        }
        else if(strncmp(command_input_buffer, "version", 7) == 0)
        {
            sprintf((char *)command_response_buffer, "version,0.0.1,alpha");
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
                    //NRF_LOG_INFO((const char *)command_response_buffer); //TODO: This looks like dookie but sends correct
                    //NRF_LOG_FLUSH();
                    // send response over BLE
                    m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
                    break;
                }
                case LFS_TYPE_DIR:
                {
                    // returning filename,filesize
                    sprintf((char *)command_response_buffer, "ls,DIR,%s,%ld", entryinfo.name,  entryinfo.size);
                    //NRF_LOG_INFO((const char *)command_response_buffer); //TODO: This looks like dookie but sends correct
                    //NRF_LOG_FLUSH();
                    // send response over BLE
                    m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
                    break;
                }
            }
        }
        else
        {
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
        }
        else if(bytes_sent < file.ctz.size)
        {
            //TODO: BUG: command_response_buffer is 244 bytes but we should limit to MTU
            int32_t read_response = lfs_file_read(m_lfs, &file, &command_response_buffer, sizeof(command_response_buffer));
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
