#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H
#include "lfs.h"

void command_interface_process_byte( char incoming );
void command_interface_init(void (*ble_send_logbuffer)(unsigned char *, unsigned int), lfs_t *lfs );
void continueFileTransfer();


#endif