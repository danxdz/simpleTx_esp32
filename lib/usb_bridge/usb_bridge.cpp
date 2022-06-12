
#include "usb_host_hid_bridge.h"
#include "uart.h"



UsbHostHidBridge hidBridge;
int32_t usb_input_ch[] = { 0,0,0,0, 0,0,0,0 };


void config_desc_cb(const usb_config_desc_t *config_desc);
void device_info_cb(usb_device_info_t *dev_info);
void hid_report_descriptor_cb(usb_transfer_t *transfer);
void hid_report_cb(usb_transfer_t *transfer);

void setUsbBridge(void)
{
    delay(2000); // await monitor port wakeup
  
    hidBridge.setOnConfigDescriptorReceived( config_desc_cb );
    hidBridge.setOnDeviceInfoReceived( device_info_cb );
    hidBridge.setOnHidReportDescriptorReceived( hid_report_descriptor_cb );
    hidBridge.setOnReportReceived( hid_report_cb );
    hidBridge.begin();
}


void config_desc_cb(const usb_config_desc_t *config_desc) {
    usb_print_config_descriptor(config_desc, NULL);
    dbout.println("config_desc_cb ***************************");
}

void device_info_cb(usb_device_info_t *dev_info) {
    if (dev_info->str_desc_manufacturer) usb_print_string_descriptor(dev_info->str_desc_manufacturer);
    if (dev_info->str_desc_product)      usb_print_string_descriptor(dev_info->str_desc_product);
    if (dev_info->str_desc_serial_num)   usb_print_string_descriptor(dev_info->str_desc_serial_num);
    dbout.println("device_info_cb ***************************");

}

void hid_report_descriptor_cb(usb_transfer_t *transfer) {
    //>>>>> for HID Report Descriptor
    // Explanation: https://electronics.stackexchange.com/questions/68141/
    // USB Descriptor and Request Parser: https://eleccelerator.com/usbdescreqparser/#
    //<<<<<
    dbout.printf("\nstatus %d, actual number of bytes transferred %d\n", transfer->status, transfer->actual_num_bytes);
    for(int i=0; i < transfer->actual_num_bytes; i++) {
        if (i == USB_SETUP_PACKET_SIZE) {
            dbout.printf("\n\n>>> Goto https://eleccelerator.com/usbdescreqparser/ \n");
            dbout.printf(">>> Copy & paste below HEX and parser as... USB HID Report Descriptor\n\n");
        }
        dbout.printf("%02X ", transfer->data_buffer[i]);
    }
    dbout.printf("\n\n");
    // Serial.printf("HID Report Descriptor\n");
    uint8_t *const data = (uint8_t *const)(transfer->data_buffer + USB_SETUP_PACKET_SIZE);
    size_t len = transfer->actual_num_bytes - USB_SETUP_PACKET_SIZE;
    // Serial.printf("> size: %ld bytes\n", len);
    bool isGamepad = false;
    bool isVenDef  = false;
    if (len >= 5) {
        uint8_t gamepadUsagePage[] = { 0x05, 0x01, 0x09, 0x05 };
        uint8_t vdrDefUsagePage[] = { 0x06, 0x00, 0xFF, 0x09, 0x01 };
        isGamepad = memcmp(data, gamepadUsagePage, sizeof(gamepadUsagePage)) == 0;
        isVenDef  = memcmp(data, vdrDefUsagePage, sizeof(vdrDefUsagePage)) == 0;
    }
    dbout.printf(">>> best guess: %s\n", isGamepad ? "HID Gamepad" : isVenDef ? "Vendor Defined" : "Unkown");
}


void hid_report_cb(usb_transfer_t *transfer) {
    //
    // check HID Report Descriptor for usage
    //
    unsigned char *const data = (unsigned char *const)(transfer->data_buffer);
    //for (int i=0; i<transfer->actual_num_bytes && i<11; i++) {
        // Serial.printf("%d ", data[i]);
        // Serial.printf("%02X ", data[i]);
      //  for (int b = 8; b != -1; b--) dbout.printf("%d", (data[i] & (1 << b)) >> b );
      //  dbout.print(" ");
    //}
    //dbout.print("\n");
    usb_input_ch[0] = data[3] * 16; //map(data[0], 0, 255, 0, 4096);
    usb_input_ch[1] = data[1] * 16; //map(data[1], 0, 255, 0, 4096);
    usb_input_ch[2] = data[2] * 16; //map(data[2], 0, 255, 0, 4096);
    usb_input_ch[3] = data[0] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[4] = data[4] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[5] = data[5] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[6] = data[6] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[7] = data[7] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[8] = data[8] * 16; //map(data[0], 0, 255, 0, 4096);
    usb_input_ch[9] = data[9] * 16; //map(data[1], 0, 255, 0, 4096);
    usb_input_ch[10] = data[10] * 16; //map(data[2], 0, 255, 0, 4096);
    usb_input_ch[11] = data[11] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[12] = data[12] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[13] = data[13] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[14] = data[14] * 16; //map(data[3], 0, 255, 0, 4096);
    usb_input_ch[15] = data[15] * 16; //map(data[3], 0, 255, 0, 4096);

    for (int i=0; i<16; i++) dbout.printf("%d ", data[i]);
      dbout.println("");


}

