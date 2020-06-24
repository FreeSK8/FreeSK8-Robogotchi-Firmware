#include "command_interface.h"

#include <stdio.h>
#include "stdint.h"
#include "string.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_ble_gatt.h"

#define MAX_COMMAND_LEN 32

static lfs_file_t file;

static int32_t bytes_sent = -1; //file.ctz.size;

void (*m_ble_tx_logbuffer)(unsigned char *data, unsigned int len);
lfs_t *m_lfs;

struct command_struct{
    char command[MAX_COMMAND_LEN];
    void (*action)(char*);
    struct command_struct *next;
} ;

struct command_struct CommandList[10];

static int command_input_index = 0;
static char command_input_buffer[ MAX_COMMAND_LEN + 1 ] = { 0 };
static unsigned char command_response_buffer[ 128 ];

void command_interface_init(void (*ble_send_logbuffer)(unsigned char *, unsigned int), lfs_t *lfs)
{
    ASSERT(lfs != NULL);
    ASSERT(ble_send_logbuffer != NULL);
    m_lfs = lfs;
    m_ble_tx_logbuffer = ble_send_logbuffer;
}


void command_interface_process_byte(char incoming)
{
    command_input_buffer[ command_input_index++ ] = incoming;
    
    if(command_input_index >= MAX_COMMAND_LEN)
    {
        command_input_index = 0;
        command_input_buffer[ command_input_index ] = 0;
        
        return;
    }

    if(incoming == '~' && strlen(command_input_buffer) > 0) //TODO: ~null~ terminate after debugging
    {
        if(strncmp(command_input_buffer, "ls", 2) == 0)
        {
            NRF_LOG_INFO("command_interface: ls (list files) command received");
            // open the log directory
            lfs_dir_t directory;
            lfs_dir_open(m_lfs, &directory, "/FreeSK8Logs");
          
            struct lfs_info entryinfo;
            while(lfs_dir_read(m_lfs,&directory,&entryinfo))
            {
                if(entryinfo.type == LFS_TYPE_REG)
                {
                    // returning filename,filesize
                    sprintf((char *)command_response_buffer, "ls,%s,%ld", entryinfo.name,  entryinfo.size);
                    //NRF_LOG_INFO((const char *)command_response_buffer); //TODO: This looks like dookie but sends correct
                    //NRF_LOG_FLUSH();
                    // send response over BLE
                    m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
                }
                
                NRF_LOG_INFO("%s,%d,bytes,%s", entryinfo.type == LFS_TYPE_REG ? "FILE" : "DIR ", entryinfo.size, entryinfo.name);
                NRF_LOG_FLUSH();
            }
        }
        else if(strncmp(command_input_buffer, "fc", 2) == 0)
        {
            NRF_LOG_INFO("command_interface: fc (file count) command received");
            uint16_t file_count = 0;
            uint32_t file_bytes_total = 0;
            // open the log directory
            lfs_dir_t directory;
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
            sprintf((char *)command_response_buffer, "%d,%ld", file_count, file_bytes_total);
            // send response over BLE
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }

        else if(strncmp(command_input_buffer, "cat ", 4) == 0 && strlen(command_input_buffer) > 4)
        {
            NRF_LOG_INFO("command_interface: cat (concatenate) command received");
            char filename[23+13] = "/FreeSK8Logs/";
            strcpy(&filename[13], &command_input_buffer[4]);
            filename[strlen(filename)-1] = 0x0; //TODO: remove once commands are null terminated
            NRF_LOG_INFO("filename %s", filename);
            NRF_LOG_FLUSH();
            if(lfs_file_open(m_lfs, &file, filename, LFS_O_RDONLY) >= 0)
            {
                sprintf((char *)command_response_buffer, "cat,%s,", filename);
                m_ble_tx_logbuffer(command_response_buffer, (size_t)strlen(command_response_buffer));

                NRF_LOG_INFO("Sent header");
                NRF_LOG_FLUSH();
                bytes_sent = 0;
            } 
        }

        else if(strncmp(command_input_buffer, "rm ", 3) == 0 && strlen(command_input_buffer) > 3)
        {
            NRF_LOG_INFO("command_interface: rm (remove) command received");
            char filename[23+13] = "/FreeSK8Logs/";
            strcpy(&filename[13], &command_input_buffer[3]);
            filename[strlen(filename)-1] = 0x0; //TODO: remove once commands are null terminated
            NRF_LOG_INFO("filename %s", filename);
            NRF_LOG_FLUSH();
            if (lfs_remove(m_lfs,filename) >= 0)
            {
                sprintf((char *)command_response_buffer, "rm,OK");
            } else {
                sprintf((char *)command_response_buffer, "rm,FAIL");
            }
            
            m_ble_tx_logbuffer(command_response_buffer, strlen((const char *)command_response_buffer));
        }

        command_input_index = 0;
        command_input_buffer[ command_input_index ] = 0;
    }
}


void continueFileTransfer()
{
    if( bytes_sent < 0 ) return;
    if(bytes_sent < file.ctz.size)
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

    if(bytes_sent >= file.ctz.size)
    {
        bytes_sent = -1;
        //NRF_LOG_INFO("break");
        //NRF_LOG_FLUSH();

        //sprintf((char *)command_response_buffer, "cat,%s,", filename);
        m_ble_tx_logbuffer("cat,complete", strlen("cat,complete"));

        NRF_LOG_INFO("finished cat");
        NRF_LOG_FLUSH();
    }
}