#include "command_interface.h"

#include <time.h>
#include <stdio.h>
#include "stdint.h"
#include "string.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_ble_gatt.h"

extern int log_file_stop();
extern void log_file_start();

static lfs_file_t file;
static lfs_dir_t directory;
static int32_t bytes_sent = -1; //file.ctz.size;

void (*m_ble_tx_logbuffer)(unsigned char *data, unsigned int len);
static lfs_t *m_lfs;

static int command_input_index = 0;
static char command_input_buffer[ 128 ] = { 0 };
static unsigned char command_response_buffer[ 128 ];

void command_interface_init(void (*ble_send_logbuffer)(unsigned char *, unsigned int), lfs_t *lfs)
{
    ASSERT(lfs != NULL);
    ASSERT(ble_send_logbuffer != NULL);
    m_lfs = lfs;
    m_ble_tx_logbuffer = ble_send_logbuffer;
}

enum{
    TRANSFER_MODE_IDLE,
    TRANSFER_MODE_LS,
    TRANSFER_MODE_CAT,
};

extern time_t currentTime;
extern struct tm * tmTime;
extern struct lfs_config cfg;

static volatile uint8_t transferMode = TRANSFER_MODE_IDLE;

void command_interface_process_byte(char incoming)
{
    command_input_buffer[ command_input_index++ ] = incoming;
    command_input_buffer[ command_input_index ] = 0x00;
    
    if(command_input_index >= sizeof(command_input_buffer))
    {
        command_input_index = 0;
        command_input_buffer[ command_input_index ] = 0;

        return;
    }

    if(incoming == '~' && strlen(command_input_buffer) >0) //TODO: ~null~ terminate after debugging
    {
        if(strncmp(command_input_buffer, "log", 3) == 0)
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

        }
        else if(strncmp(command_input_buffer, "settime ", 8) == 0)
        {
            char* timepointer = command_input_buffer + 8;
            int syear, smonth, sday, shour, sminute, ssecond;
            sscanf( timepointer, "%d:%d:%dT%d:%d:%d", &syear, &smonth, &sday, &shour, &sminute, &ssecond );

            NRF_LOG_INFO("command_interface: settime command received: %s", command_input_buffer + 8);
            NRF_LOG_FLUSH();

            tmTime->tm_year = syear - 1900;
            tmTime->tm_mon = smonth - 1;
            tmTime->tm_mday = sday;
            tmTime->tm_hour = shour;
            tmTime->tm_min = sminute;
            tmTime->tm_sec = ssecond;

            currentTime = mktime(tmTime);
        }
        else if(strncmp(command_input_buffer, "ls", 2) == 0)
        {
            // open the log directory
            lfs_dir_open(m_lfs, &directory, "/FreeSK8Logs");

            sprintf((char *)command_response_buffer, "ls,/FreeSK8Logs");
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
          
            NRF_LOG_INFO("command_interface: ls (list files) command received: %s", command_response_buffer);
            NRF_LOG_FLUSH();

            transferMode = TRANSFER_MODE_LS;
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
        else if(strncmp(command_input_buffer, "cat ", 4) == 0 && strlen(command_input_buffer) > 4)
        {
            NRF_LOG_INFO("command_interface: cat (concatenate) command received");
            char filename[19+13+1] = "/FreeSK8Logs/"; //2020-06-20T20:26:40
            strcpy(&filename[13], &command_input_buffer[4]);
            filename[strlen(filename)-1] = 0x0; //TODO: remove once commands are null terminated
            NRF_LOG_INFO("filename %s", filename);
            NRF_LOG_FLUSH();
            if(lfs_file_open(m_lfs, &file, filename, LFS_O_RDONLY) >= 0)
            {
                sprintf((char *)command_response_buffer, "cat,%s", filename);
                m_ble_tx_logbuffer(command_response_buffer, (size_t)strlen((const char *)command_response_buffer));

                NRF_LOG_INFO("Sent header");
                NRF_LOG_FLUSH();
                bytes_sent = 0;
                transferMode = TRANSFER_MODE_CAT;
            }
        }
        else if(strncmp(command_input_buffer, "rm ", 3) == 0 && strlen(command_input_buffer) > 3)
        {
            NRF_LOG_INFO("command_interface: rm (remove) command received");

            char filename[32] = {0}; //TODO: Max length is expected to be 19 but undetermined atm
            strcpy(filename, &command_input_buffer[3]);
            filename[strlen(filename)-1] = 0x0; //TODO: remove once commands are null terminated

            char filepath[64] = "/FreeSK8Logs/";
            strcat(filepath, filename);
            NRF_LOG_INFO("filepath %s", filepath);
            NRF_LOG_FLUSH();

            lfs_unmount( m_lfs );
            int err = lfs_mount(m_lfs, &cfg);

            int remove_response = lfs_remove(m_lfs,filepath);
            NRF_LOG_INFO("lfs_remove():remove_response: %d", remove_response);
            NRF_LOG_FLUSH();
            if (remove_response >= 0)
            {
                sprintf((char *)command_response_buffer, "rm,OK,%s", filename);
            } 
            else 
            {
                sprintf((char *)command_response_buffer, "rm,FAIL,%s,%d", filename, remove_response);
            }

            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }

        command_input_index = 0;
        command_input_buffer[ command_input_index ] = 0;
    }
}

void command_interface_continue_transfer()
{
    switch(transferMode)
    {
        case TRANSFER_MODE_IDLE:
            return;
        case TRANSFER_MODE_LS:
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
                transferMode = TRANSFER_MODE_IDLE;
            }
        }
        break;
        case TRANSFER_MODE_CAT:
        {
            NRF_LOG_INFO("command_interface_continue_transfer(TRANSFER_MODE_CAT)");
            NRF_LOG_FLUSH();

            if(bytes_sent >= file.ctz.size)
            {
                transferMode = TRANSFER_MODE_IDLE;
                m_ble_tx_logbuffer((unsigned char *)"cat,complete", strlen("cat,complete"));

                NRF_LOG_INFO("finished cat");
                NRF_LOG_FLUSH();
            }
            else if(bytes_sent < file.ctz.size)
            {
                int32_t read_response = lfs_file_read(m_lfs, &file, &command_response_buffer, sizeof(command_response_buffer));
                NRF_LOG_INFO("read_response = %d", read_response);
                NRF_LOG_FLUSH();
                if(read_response > 0)
                {
                    m_ble_tx_logbuffer(command_response_buffer, read_response);
                    bytes_sent += read_response;
                    NRF_LOG_INFO("Sent %d bytes", read_response);
                    NRF_LOG_FLUSH();
                }
            }

        }
        break;
    }
}