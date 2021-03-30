#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H
#include "lfs.h"


enum{
    TRANSFER_MODE_IDLE,
    TRANSFER_MODE_LS,
    TRANSFER_MODE_CAT,
    TRANSFER_MODE_CAT_BEGIN
};


void command_interface_process_byte( char incoming );
void command_interface_init(
    void (*ble_send_logbuffer)(unsigned char *, unsigned int),
    lfs_t *lfs,
    void (*update_time)(int, int, int, int, int, int)
);
void command_interface_continue_transfer();


#endif