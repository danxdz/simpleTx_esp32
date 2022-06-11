#pragma once

#include "usb_host_hid_bridge.h"

//#define DAEMON_TASK_LOOP_DELAY  3 // ticks
//#define CLASS_TASK_LOOP_DELAY   3 // ticks
#define DAEMON_TASK_COREID      0
#define CLASS_TASK_COREID       0

extern int32_t usb_input_ch[];

void setUsbBridge(void);
