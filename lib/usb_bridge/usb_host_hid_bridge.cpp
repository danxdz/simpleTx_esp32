/****************************************************************************************************************************
  usb_host_hid_bridge.cpp
  For ESP32 S series boards

  ESP32 USB Host HID Brigde is a library for the ESP32/Arduino platform
  Built by Jeff Leung https://github.com/badjeff/ESP32-USB-Host-HID-Bridge
  Licensed under MIT license
  
  Version: 1.0.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   Jeff Leung   06/07/2022 Initial coding
 *****************************************************************************************************************************/

#include <Arduino.h>
#include "usb_host_hid_bridge.h"
#include "uart.h"

#if !CONFIG_DISABLE_HAL_LOCKS
    SemaphoreHandle_t _mtx_lock;
#endif
#if !CONFIG_DISABLE_HAL_LOCKS
#define HUHHB_MUTEX_LOCK()    do {} while (xSemaphoreTake(_mtx_lock, portMAX_DELAY) != pdPASS)
#define HUHHB_MUTEX_UNLOCK()  xSemaphoreGive(_mtx_lock)
#else
#define HUHHB_MUTEX_LOCK()    
#define HUHHB_MUTEX_UNLOCK()  
#endif

// bit mask for async tasks
#define ACTION_OPEN_DEV             0x01
#define ACTION_GET_DEV_INFO         0x02
#define ACTION_GET_DEV_DESC         0x04
#define ACTION_GET_CONFIG_DESC      0x08
#define ACTION_GET_STR_DESC         0x10
#define ACTION_CLOSE_DEV            0x20
#define ACTION_EXIT                 0x40
#define ACTION_CLAIM_INTF           0x0100
#define ACTION_TRANSFER_CONTROL     0x0200
#define ACTION_TRANSFER             0x0400

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
    uint16_t bMaxPacketSize0;
    usb_ep_desc_t *ep_in;
    usb_ep_desc_t *ep_out;
    SemaphoreHandle_t transfer_done;
    usb_transfer_status_t transfer_status;
} class_driver_t;

TaskHandle_t _daemon_task_hdl;
TaskHandle_t _class_driver_task_hdl;

static const char *TAG_DAEMON = "DAEMON";
static const char *TAG_CLASS = "CLASS";

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            if (driver_obj->dev_addr == 0) {
                driver_obj->dev_addr = event_msg->new_dev.address;
                //Open the device next
                driver_obj->actions |= ACTION_OPEN_DEV;
            }
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            if (driver_obj->dev_hdl != NULL) {
                //Cancel any other actions and close the device next
                driver_obj->actions = ACTION_CLOSE_DEV;
            }
            break;
        default:
            //Should never occur
            abort();
    }
}

static void action_open_dev(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(TAG_CLASS, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    
    //Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));

    // ESP_LOGI(TAG_CLASS, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    // ESP_LOGI(TAG_CLASS, "\tbConfigurationValue %d", dev_info.bConfigurationValue);

    //Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));

    driver_obj->bMaxPacketSize0 = dev_desc->bMaxPacketSize0; // shall be used in action_transfer_control()
    // ESP_LOGI(TAG_CLASS, "\tidVendor 0x%04x", dev_desc->idVendor);
    // ESP_LOGI(TAG_CLASS, "\tidProduct 0x%04x", dev_desc->idProduct);
    // usb_print_device_descriptor(dev_desc);

    //Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(class_driver_t *driver_obj, UsbHostHidBridge *bdg)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    if (bdg->_configDescCb != NULL) {
        bdg->_configDescCb(config_desc);
    }
    //Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t *driver_obj, UsbHostHidBridge *bdg)
{
    assert(driver_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    if (bdg->_deviceInfoCb != NULL) {
        bdg->_deviceInfoCb(&dev_info);
    }
    //Claim the interface next
    driver_obj->actions &= ~ACTION_GET_STR_DESC;
    driver_obj->actions |= ACTION_CLAIM_INTF;
}

static void action_claim_interface(class_driver_t *driver_obj, UsbHostHidBridge *bdg)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));

    bool hidIntfClaimed = false;
    int offset = 0;
    for (size_t n = 0; n < config_desc->bNumInterfaces; n++)
    {
        const usb_intf_desc_t *intf = usb_parse_interface_descriptor(config_desc, n, 0, &offset);
        ESP_LOGI(TAG_CLASS, "Parsed intf->bInterfaceNumber: 0x%02x \n", intf->bInterfaceNumber);
        dbout.printf("Parsed intf->bInterfaceNumber: 0x%02x \n", intf->bInterfaceNumber);
        dbout.printf("Detected HID intf->bInterfaceClass: 0x%02x \n", intf->bInterfaceClass);

        if (intf->bInterfaceClass == 0x03) // HID - https://www.usb.org/defined-class-codes
        {
            ESP_LOGI(TAG_CLASS, "Detected HID intf->bInterfaceClass: 0x%02x \n", intf->bInterfaceClass);
            dbout.printf("Detected endpoints: 0x%02x \n", intf->bNumEndpoints);

            const usb_ep_desc_t *ep_in = NULL;
            const usb_ep_desc_t *ep_out = NULL;
            const usb_ep_desc_t *ep = NULL;
            for (size_t i = 0; i < intf->bNumEndpoints; i++) {
                int _offset = 0;
                ep = usb_parse_endpoint_descriptor_by_index(intf, i, config_desc->wTotalLength, &_offset);
                ESP_LOGI(TAG_CLASS, "\t > Detected EP num: %d/%d, len: %d, ", i + 1, intf->bNumEndpoints, config_desc->wTotalLength);
                ESP_LOGI(TAG_CLASS, "\t   address: 0x%02x, mps: %d, dir: %s", ep->bEndpointAddress, ep->wMaxPacketSize, (ep->bEndpointAddress & 0x80) ? "IN" : "OUT");
                dbout.printf("\t > Detected EP num: %d/%d, len: %d, ", i + 1, intf->bNumEndpoints, config_desc->wTotalLength);
                dbout.printf("\t   address: 0x%02x, mps: %d, dir: %s", ep->bEndpointAddress, ep->wMaxPacketSize, (ep->bEndpointAddress & 0x80) ? "IN" : "OUT");
                
                if (ep) {
                    if (ep->bmAttributes != USB_TRANSFER_TYPE_INTR) {
                        // only support INTERRUPT > IN Report in action_transfer() for now
                        continue;
                    }
                    if (ep->bEndpointAddress & 0x80) {
                        ep_in = ep;
                        driver_obj->ep_in = (usb_ep_desc_t *)ep_in;
                    } else {
                        ep_out = ep;
                        driver_obj->ep_out = (usb_ep_desc_t *)ep_out;
                    }
                } else {
                    ESP_LOGW("", "error to parse endpoint by index; EP num: %d/%d, len: %d", i + 1, intf->bNumEndpoints, config_desc->wTotalLength);
                }
            }
            esp_err_t err = usb_host_interface_claim(driver_obj->client_hdl, driver_obj->dev_hdl, n, 0);
            if (err) {
                ESP_LOGI("", "interface claim status: %d", err);
            } else {
                ESP_LOGI(TAG_CLASS, "Claimed HID intf->bInterfaceNumber: 0x%02x \n", intf->bInterfaceNumber);
                hidIntfClaimed = true;
            }
        }
    }

    //Get the HID's descriptors next
    driver_obj->actions &= ~ACTION_CLAIM_INTF;
    if (hidIntfClaimed)
    {
        driver_obj->actions |= ACTION_TRANSFER_CONTROL;
    }
}

static void transfer_cb(usb_transfer_t *transfer)
{
    //This is function is called from within usb_host_client_handle_events(). Don't block and try to keep it short
    //struct class_driver_control *class_driver_obj = (struct class_driver_control *)transfer->context;
    class_driver_t *driver_obj = (class_driver_t *)transfer->context;
    driver_obj->transfer_status = transfer->status;
    xSemaphoreGive(driver_obj->transfer_done);
}

static esp_err_t wait_for_transfer_done(usb_transfer_t *transfer)
{
    class_driver_t *driver_obj = (class_driver_t *)transfer->context;
    BaseType_t received = xSemaphoreTake(driver_obj->transfer_done, pdMS_TO_TICKS(transfer->timeout_ms));
    // BaseType_t received = xSemaphoreTake(driver_obj->transfer_done, portMAX_DELAY);
    if (received != pdTRUE) {
        xSemaphoreGive(driver_obj->transfer_done);
        return ESP_ERR_TIMEOUT;
    }
    xSemaphoreGive(driver_obj->transfer_done);
    return (driver_obj->transfer_status == USB_TRANSFER_STATUS_COMPLETED) ? ESP_OK : ESP_FAIL;
}

static void action_transfer_control(class_driver_t *driver_obj, UsbHostHidBridge *bdg)
{
    assert(driver_obj->dev_hdl != NULL);
    static uint16_t mps = driver_obj->bMaxPacketSize0;
    static uint16_t tps = usb_round_up_to_mps(1024, mps);
    static usb_transfer_t *transfer;
    if (!transfer) {
        usb_host_transfer_alloc(tps, 0, &transfer);
    }
    static TickType_t lastSendTime = 0;
    if (xTaskGetTickCount() - lastSendTime > INTV_XFER_CTRL)
    {
        usb_setup_packet_t stp;

        // 0x81,        // bmRequestType: Dir: D2H, Type: Standard, Recipient: Interface
        // 0x06,        // bRequest (Get Descriptor)
        // 0x00,        // wValue[0:7]  Desc Index: 0
        // 0x22,        // wValue[8:15] Desc Type: (HID Report)
        // 0x00, 0x00,  // wIndex Language ID: 0x00
        // 0x40, 0x00,  // wLength = 64
        stp.bmRequestType = USB_BM_REQUEST_TYPE_DIR_IN | USB_BM_REQUEST_TYPE_TYPE_STANDARD | USB_BM_REQUEST_TYPE_RECIP_INTERFACE;
        stp.bRequest = USB_B_REQUEST_GET_DESCRIPTOR;
        stp.wValue = 0x2200;
        stp.wIndex = 0;
        stp.wLength = tps - 8;
        transfer->num_bytes = tps;

        memcpy(transfer->data_buffer, &stp, USB_SETUP_PACKET_SIZE);
        transfer->bEndpointAddress = 0x00;
        ESP_LOGI("", "transfer->bEndpointAddress: 0x%02X \n", transfer->bEndpointAddress);

        transfer->device_handle = driver_obj->dev_hdl;
        transfer->callback = transfer_cb;
        transfer->context = (void *)driver_obj;
        transfer->timeout_ms = 5000;

        BaseType_t received = xSemaphoreTake(driver_obj->transfer_done, INTV_XFER_CTRL + pdMS_TO_TICKS(transfer->timeout_ms));
        if (received == pdTRUE) {
            esp_err_t result = usb_host_transfer_submit_control(driver_obj->client_hdl, transfer);
            if (result != ESP_OK) {
                ESP_LOGW("", "attempting control %s\n", esp_err_to_name(result));
                transfer_cb(transfer);
            } else {
                usb_host_client_handle_events(driver_obj->client_hdl, INTV_XFER_CTRL + pdMS_TO_TICKS(transfer->timeout_ms));
                wait_for_transfer_done(transfer);
                if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
                    ESP_LOGW("", "Transfer control failed - Status %d \n", transfer->status);
                }
                if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
                    if (transfer->actual_num_bytes > 0 && bdg->_hidReportDescCb != NULL) {
                        bdg->_hidReportDescCb(transfer);
                    }
                    driver_obj->actions |= ACTION_TRANSFER;
                }
            }
            driver_obj->actions &= ~ACTION_TRANSFER_CONTROL;
            #if defined(DELAY_POST_XFER_CTRL) && DELAY_POST_XFER_CTRL > 0
                vTaskDelay(DELAY_POST_XFER_CTRL); //Add a short delay to let the tasks run
            #endif
        }
        lastSendTime = xTaskGetTickCount();
    }
}

static void action_transfer(class_driver_t *driver_obj, UsbHostHidBridge *bdg)
{
    assert(driver_obj->dev_hdl != NULL);
    static uint16_t mps = driver_obj->ep_in->wMaxPacketSize;
    // static uint16_t tps = usb_round_up_to_mps(mps, mps);
    static usb_transfer_t *transfer;
    if (!transfer) {
        usb_host_transfer_alloc(mps, 0, &transfer);
        memset(transfer->data_buffer, 0x00, mps);
    }
    static TickType_t lastSendTime = 0;
    if (xTaskGetTickCount() - lastSendTime > INTV_XFER)
    {
        transfer->num_bytes = mps;
        memset(transfer->data_buffer, 0x00, mps);
        transfer->bEndpointAddress = driver_obj->ep_in->bEndpointAddress;
        // ESP_LOGI("", "transfer->bEndpointAddress: 0x%02X \n", transfer->bEndpointAddress);
        //dbout.printf("transfer->bEndpointAddress: 0x%02X \n", transfer->bEndpointAddress);

        transfer->device_handle = driver_obj->dev_hdl;
        transfer->callback = transfer_cb;
        transfer->context = (void *)driver_obj;
        transfer->timeout_ms = 5000;

        BaseType_t received = xSemaphoreTake(driver_obj->transfer_done, INTV_XFER + pdMS_TO_TICKS(transfer->timeout_ms));
        if (received == pdTRUE) {
            esp_err_t result = usb_host_transfer_submit(transfer);
            if (result != ESP_OK) {
                ESP_LOGW("", "attempting %s\n", esp_err_to_name(result));
                dbout.printf("attempting %s\n", esp_err_to_name(result));
                transfer_cb(transfer);
            } else {
                usb_host_client_handle_events(driver_obj->client_hdl, INTV_XFER + pdMS_TO_TICKS(transfer->timeout_ms));
                wait_for_transfer_done(transfer);
                if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
                    ESP_LOGW("", "Transfer failed - Status %d \n", transfer->status);
                    dbout.printf("", "Transfer failed - Status %d \n", transfer->status);
                }
                if (transfer->status == USB_TRANSFER_STATUS_COMPLETED) {
                    if (transfer->actual_num_bytes > 0 && bdg->_hidReportCb != NULL) {
                        bdg->_hidReportCb(transfer);
                    }
                }
            }
            // driver_obj->actions &= ~ACTION_TRANSFER; // break during development
            #if defined(DELAY_POST_XFER) && DELAY_POST_XFER > 0
                vTaskDelay(DELAY_POST_XFER); //Add a short delay to let the tasks run
            #endif
        }
        lastSendTime = xTaskGetTickCount();
    }
}

static void aciton_close_dev(class_driver_t *driver_obj)
{
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    
    int offset = 0;
    for (size_t n = 0; n < config_desc->bNumInterfaces; n++)
    {
        const usb_intf_desc_t *intf = usb_parse_interface_descriptor(config_desc, n, 0, &offset);
        ESP_LOGI(TAG_CLASS, "\nReleasing intf->bInterfaceNumber: 0x%02x \n", intf->bInterfaceNumber);
        if (intf->bInterfaceClass == 0x03) // HID - https://www.usb.org/defined-class-codes
        {
            ESP_LOGI(TAG_CLASS, "\nReleasing HID intf->bInterfaceClass: 0x%02x \n", intf->bInterfaceClass);
            const usb_ep_desc_t *ep_in = NULL;
            const usb_ep_desc_t *ep_out = NULL;
            const usb_ep_desc_t *ep = NULL;
            for (size_t i = 0; i < intf->bNumEndpoints; i++) {
                int _offset = 0;
                ep = usb_parse_endpoint_descriptor_by_index(intf, i, config_desc->wTotalLength, &_offset);
                if (ep) {
                    if (ep->bEndpointAddress & 0x80) {
                        ep_in = ep;
                    } else {
                        ep_out = ep;
                    }
                    ESP_LOGI(TAG_CLASS, "\t > Halting EP num: %d/%d, len: %d, ", i + 1, intf->bNumEndpoints, config_desc->wTotalLength);
                    ESP_LOGI(TAG_CLASS, "\t   address: 0x%02x, EP max size: %d, dir: %s\n", ep->bEndpointAddress, ep->wMaxPacketSize, (ep->bEndpointAddress & 0x80) ? "IN" : "OUT");
                    ESP_ERROR_CHECK(usb_host_endpoint_halt(driver_obj->dev_hdl, ep->bEndpointAddress));
                    ESP_ERROR_CHECK(usb_host_endpoint_flush(driver_obj->dev_hdl, ep->bEndpointAddress));
                }
            }
            ESP_ERROR_CHECK(usb_host_interface_release(driver_obj->client_hdl, driver_obj->dev_hdl, n));
        }
    }

    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    //We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions &= ~ACTION_TRANSFER;
    driver_obj->actions |= ACTION_EXIT;
}

static void usb_class_driver_task(void *pvParameters)
{
    UsbHostHidBridge *bdg = (UsbHostHidBridge *)pvParameters;
    // SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)bdg->_signaling_sem;
    class_driver_t driver_obj = {0};

    //Wait until daemon task has installed USB Host Library
    // xSemaphoreTake(signaling_sem, portMAX_DELAY);
    while (!bdg->hostInstalled) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG_CLASS, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,    //Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));

    driver_obj.transfer_done = xSemaphoreCreateCounting( 1, 1 );

    while (1) {
        if (driver_obj.actions == 0) {
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        } else {
            if (driver_obj.actions & ACTION_OPEN_DEV) {
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO) {
                action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC) {
                action_get_dev_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC) {
                action_get_config_desc(&driver_obj, bdg);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC) {
                action_get_str_desc(&driver_obj, bdg);
            }
            if (driver_obj.actions & ACTION_CLAIM_INTF) {
                action_claim_interface(&driver_obj, bdg);
            }
            if (driver_obj.actions & ACTION_TRANSFER_CONTROL) {
                action_transfer_control(&driver_obj, bdg);
            }
            if (driver_obj.actions & ACTION_TRANSFER) {
                action_transfer(&driver_obj, bdg);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV) {
                aciton_close_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_EXIT) {
                break;
            }
        }
        vTaskDelay(CLASS_TASK_LOOP_DELAY);
    } // end main loop

    vSemaphoreDelete(driver_obj.transfer_done);

    ESP_LOGI(TAG_CLASS, "Deregistering Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    //Wait to be deleted
    // xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

static void usb_host_lib_daemon_task(void *pvParameters)
{
    UsbHostHidBridge *bdg = (UsbHostHidBridge *)pvParameters;
    // SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)bdg->_signaling_sem;

    ESP_LOGI(TAG_DAEMON, "Installing USB Host Library");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    //Signal to the class driver task that the host library is installed
    // xSemaphoreGive(signaling_sem);
    bdg->hostInstalled = true;
    vTaskDelay(DAEMON_TASK_LOOP_DELAY); //Short delay to let client task spin up

    bool has_clients = true;
    bool has_devices = true;
    while (has_clients || has_devices ) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            has_devices = false;
        }
        vTaskDelay(DAEMON_TASK_LOOP_DELAY);
    } // end main loop
    ESP_LOGI(TAG_DAEMON, "No more clients and devices");

    //Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    //Wait to be deleted
    // xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

UsbHostHidBridge::UsbHostHidBridge() :
    hostInstalled( false ),
    _configDescCb( NULL ),
    _deviceInfoCb( NULL ),
    _hidReportDescCb( NULL ),
    _hidReportCb( NULL )
{
}

UsbHostHidBridge::~UsbHostHidBridge()
{
}

void UsbHostHidBridge::begin()
{
    // this->_signaling_sem = xSemaphoreCreateBinary();

    //Create usb host lib daemon task
    xTaskCreatePinnedToCore(
        usb_host_lib_daemon_task,           /* Task function. */
        "usb_host_lib_daemon_task",         /* name of task. */
        4096,                               /* Stack size of task */
        (void *)this,                       /* parameter of the task */
        DAEMON_TASK_PRIORITY,               /* priority of the task */
        &_daemon_task_hdl,                  /* Task handle to keep track of created task */
        DAEMON_TASK_COREID);                /* pin task to core 0 */
    vTaskDelay(500); //Add a short delay to let the tasks run

    //Create usb class driver task
    xTaskCreatePinnedToCore(
        usb_class_driver_task,              /* Task function. */
        "usb_class_driver_task",            /* name of task. */
        4096,                               /* Stack size of task */
        (void *)this,                       /* parameter of the task */
        CLASS_TASK_PRIORITY,                /* priority of the task */
        &_class_driver_task_hdl,            /* Task handle to keep track of created task */
        CLASS_TASK_COREID);                 /* pin task to core 0 */
    vTaskDelay(500); //Add a short delay to let the tasks run
}

void UsbHostHidBridge::end()
{
    // //Wait for the tasks to complete
    // for (int i = 0; i < 2; i++) {
    //     xSemaphoreTake(this->_signaling_sem, portMAX_DELAY);
    // }
    // Delete the tasks
    vTaskDelete(_class_driver_task_hdl);
    vTaskDelete(_daemon_task_hdl);
}

void UsbHostHidBridge::setOnConfigDescriptorReceived( OnConfigDescriptorReceived _configDescCb )
{
    this->_configDescCb = _configDescCb;
}

void UsbHostHidBridge::setOnDeviceInfoReceived( OnDeviceInfoReceived _deviceInfoCb )
{
    this->_deviceInfoCb = _deviceInfoCb;
}

void UsbHostHidBridge::setOnHidReportDescriptorReceived( OnHidReportDescriptorReceived _hidReportDescCb )
{
    this->_hidReportDescCb = _hidReportDescCb;
}

void UsbHostHidBridge::setOnReportReceived( OnReportReceived _hidReportCb )
{
    this->_hidReportCb = _hidReportCb;
}
