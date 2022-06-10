/****************************************************************************************************************************
  usb_host_hid_bridge.h
  For ESP32 S series boards

  ESP32 USB Host HID Brigde is a library for the ESP32/Arduino platform
  Built by Jeff Leung https://github.com/badjeff/ESP32-USB-Host-HID-Bridge
  Licensed under MIT license
  
  Version: 1.0.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   Jeff Leung   06/07/2022 Initial coding
 *****************************************************************************************************************************/

#ifndef ESP32_USB_HID_HOST_BRIDGE_H /* include guards */
#define ESP32_USB_HID_HOST_BRIDGE_H

#include <Arduino.h>
#include <functional>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"

// config
#define DAEMON_TASK_PRIORITY         1
#if !defined(DAEMON_TASK_COREID)
     #define DAEMON_TASK_COREID      0
#endif
#if !defined(DAEMON_TASK_LOOP_DELAY)
     #define DAEMON_TASK_LOOP_DELAY  100
#endif

#define CLASS_TASK_PRIORITY           1
#if !defined(CLASS_TASK_COREID)
     #define CLASS_TASK_COREID        0
#endif
#if !defined(CLASS_TASK_LOOP_DELAY)
     #define CLASS_TASK_LOOP_DELAY    15
#endif

#define CLIENT_NUM_EVENT_MSG    5  // usb_host_client_config_t.max_num_event_msg

#define INTV_XFER_CTRL          10  // tick offset between each usb_host_transfer_submit_control() call
#define DELAY_POST_XFER_CTRL    0  // vTaskDelay(n) after a usb_host_transfer_submit_control() call

#define INTV_XFER               8  // tick offset between each usb_host_transfer_submit() call
#define DELAY_POST_XFER         0  // vTaskDelay(n) after a usb_host_transfer_submit() call

typedef std::function<void(const usb_config_desc_t *config_desc)> OnConfigDescriptorReceived;
typedef std::function<void(usb_device_info_t *dev_info)> OnDeviceInfoReceived;
typedef std::function<void(usb_transfer_t *transfer)> OnHidReportDescriptorReceived;
typedef std::function<void(usb_transfer_t *transfer)> OnReportReceived;

class UsbHostHidBridge
{

public:
    UsbHostHidBridge();
    ~UsbHostHidBridge();
    void begin();
    void end();

    void setOnConfigDescriptorReceived( OnConfigDescriptorReceived _configDescCb );
    void setOnDeviceInfoReceived( OnDeviceInfoReceived _deviceInfoCb );
    void setOnHidReportDescriptorReceived( OnHidReportDescriptorReceived _hidReportDescCb );
    void setOnReportReceived( OnReportReceived _hidReportCb );

    bool hostInstalled;
    // SemaphoreHandle_t _signaling_sem;

    OnConfigDescriptorReceived _configDescCb;
    OnDeviceInfoReceived _deviceInfoCb;
    OnHidReportDescriptorReceived _hidReportDescCb;
    OnReportReceived _hidReportCb;

protected:

};

#endif
