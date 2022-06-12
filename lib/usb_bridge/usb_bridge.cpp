
#include "usb_host_hid_bridge.h"
#include "uart.h"


uint32_t get_offset_bits(uint8_t *data, uint32_t offset, uint32_t count);

UsbHostHidBridge hidBridge;
int32_t usb_input_ch[] = { 0,0,0,0, 0,0,0,0 };


void config_desc_cb(const usb_config_desc_t *config_desc);
void device_info_cb(usb_device_info_t *dev_info);
void hid_report_descriptor_cb(usb_transfer_t *transfer);
void hid_report_cb(usb_transfer_t *transfer);

void setUsbBridge(void)
{
    hidBridge.onConfigDescriptorReceived = config_desc_cb;
    hidBridge.onDeviceInfoReceived = device_info_cb;
    hidBridge.onHidReportDescriptorReceived = hid_report_descriptor_cb;
    hidBridge.onReportReceived = hid_report_cb;
    hidBridge.begin();
}


void config_desc_cb(const usb_config_desc_t *config_desc) {
    usb_print_config_descriptor(config_desc, NULL);
}

void device_info_cb(usb_device_info_t *dev_info) {
    if (dev_info->str_desc_manufacturer) usb_print_string_descriptor(dev_info->str_desc_manufacturer);
    if (dev_info->str_desc_product)      usb_print_string_descriptor(dev_info->str_desc_product);
    if (dev_info->str_desc_serial_num)   usb_print_string_descriptor(dev_info->str_desc_serial_num);
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
    // dbout.printf("HID Report Descriptor\n");
    uint8_t *const data = (uint8_t *const)(transfer->data_buffer + USB_SETUP_PACKET_SIZE);
    size_t len = transfer->actual_num_bytes - USB_SETUP_PACKET_SIZE;
    // dbout.printf("> size: %ld bytes\n", len);
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
    uint8_t *data = (uint8_t *)(transfer->data_buffer);

    //for (int i=0; i<transfer->actual_num_bytes && i<11; i++) {
        // dbout.printf("%d ", data[i]);
        // dbout.printf("%02X ", data[i]);
        //for (int b=0; b<8; b++) dbout.printf("%d", (data[i] & (1 << b)) >> b );
        //dbout.printf(" ");
   // }
    //dbout.printf("\n");


    usb_input_ch[0] = data[0]*16;
    usb_input_ch[1] = data[1]*16;
    usb_input_ch[2] = data[2]*16;
    usb_input_ch[3] = data[3]*16;
    usb_input_ch[4] = data[4]*16;
    usb_input_ch[5] = data[5]*16;
    usb_input_ch[6] = data[6]*16;
    usb_input_ch[7] = data[7]*16;
    for (int i=0; i<8; i++) dbout.printf("%d ", usb_input_ch[i]);
    dbout.printf("\n");


    /*  usb_input_ch[0] = get_offset_bits(data, 8,  8)*16;
     usb_input_ch[1] = get_offset_bits(data, 16, 8)*16;
     usb_input_ch[2] = get_offset_bits(data, 24, 8)*16;
     usb_input_ch[3] = get_offset_bits(data, 32, 8)*16;
     usb_input_ch[4] = get_offset_bits(data, 40, 8)*16;
     usb_input_ch[5] = get_offset_bits(data, 48, 8)*16;
     usb_input_ch[6] = get_offset_bits(data, 56, 8)*16;
     usb_input_ch[7] = get_offset_bits(data, 64, 8)*16;
     for (int i=0; i<8; i++) dbout.printf("%d ", usb_input_ch[i]);
         dbout.printf("\n"); */
}

uint32_t get_offset_bits(uint8_t *data, uint32_t offset, uint32_t count) {
	int shft;
	uint32_t ret, byte;
	byte = offset / 8;
	shft = offset & 7;
	ret = (((uint32_t)data[byte + 0]) << (shft + 24));
	if (count + shft > 8)
		ret |= (((uint32_t)data[byte + 1]) << (shft + 16));
	if (count + shft > 16)
		ret |= (((uint32_t)data[byte + 2]) << (shft +  8));
	if (count + shft > 24)
		ret |= (((uint32_t)data[byte + 3]) << (shft +  0));
	if (count + shft > 32)
		ret |= (((uint32_t)data[byte + 4]) << (shft -  8));
	return ret >> (32 - count);
}
