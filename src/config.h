/*
 * This file is part of Simple TX
 *
 * Simple TX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Simple TX is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/* 
 =======================================================================================================
 Simple TX CONFIG OPTIONS (comment out unneeded options)
 =======================================================================================================
 */
int Aileron_value = 0;        // values read from the pot 
int Elevator_value = 0; 
int Throttle_value=0;
int Rudder_value = 0; 

int Arm = 0;        // switch values read from the digital pin
int FlightMode = 0; 

 // Define RC input Offset
int Aileron_OFFSET = 0;        // values read from the pot 
int Elevator_OFFSET  = 0; 
int Throttle_OFFSET =0;
int Rudder_OFFSET  = 0; 

static HardwareSerial elrs(1);
static HardwareSerial db_out(0);

#define _tr_noop(x) x

static const char * const crsf_opts[] = {
  _tr_noop("Bit Rate"), "400K", "1.87M", "2.25M", NULL,
  NULL
};

static void add_param(uint8_t *buffer, uint8_t num_bytes);
static void elrsWrite (uint8_t crsfPacket[],uint8_t size,int32_t add_delay); 

void sync_crsf(int32_t add);
void serialEvent();

#define CRSF_MAX_PARAMS  55   // one extra required, max observed is 47 in Diversity Nano RX
#define CRSF_MAX_DEVICES       4
#define CRSF_MAX_NAME_LEN      16
#define CRSF_MAX_STRING_BYTES  2500     // max observed is 2010 in Nano RX
#define CRSF_STRING_BYTES_AVAIL(current)  (CRSF_MAX_STRING_BYTES-((char *)(current)-mp->strings))


//NUM_TRIM_ELEMS + NUM_BOX_ELEMS + NUM_BAR_ELEMS + NUM_TOGGLE_ELEMS
#ifndef NUM_ELEMS
    #define NUM_ELEMS (6 + 8 + 8 + 4 + 1)
#endif

struct crsfconfig_page {
    char strings[CRSF_MAX_STRING_BYTES];

};
struct crsfdevice_page {
    char strings[CRSF_MAX_STRING_BYTES];
};

struct pagemem {
    union {
        //struct main_page main_page;
        struct crsfconfig_page crsfconfig_page;
        struct crsfdevice_page crsfdevice_page;
    } u;
    uint8_t modal_page;
};




struct pagemem pagemem;
static uint16_t current_selected = 0;
static uint8_t number_of_devices;    // total known



enum data_type {
    UINT8          = 0,
    INT8           = 1,
    UINT16         = 2,
    INT16          = 3,
    FLOAT          = 8,
    TEXT_SELECTION = 9,
    STRING         = 10,
    FOLDER         = 11,
    INFO           = 12,
    COMMAND        = 13,
    OUT_OF_RANGE   = 127,
};


typedef struct {
    // common fields
    uint8_t device;            // device index of device parameter belongs to
    uint8_t id;                // Parameter number (starting from 1)
    uint8_t parent;            // Parent folder parameter number of the parent folder, 0 means root
    enum data_type type;  // (Parameter type definitions and hidden bit)
    uint8_t hidden;            // set if hidden
    char *name;           // Null-terminated string
    char *value;          // size depending on data type

    // field presence depends on type
    char *default_value;  // size depending on data type. Not present for COMMAND.
    int32_t min_value;        // not sent for string type
    int32_t max_value;        // not sent for string type
    int32_t step;             // Step size ( type float only otherwise this entry is not sent )
    uint8_t timeout;           // COMMAND timeout (100ms/count)
    uint8_t changed;           // flag if set needed when edit element is de-selected
    char *max_str;        // Longest choice length for text select
    union {
        uint8_t point;             // Decimal point ( type float only otherwise this entry is not sent )
        uint8_t text_sel;          // current value index for TEXT_SELECTION type
        uint8_t string_max_len;    // String max length ( for string type only )
        uint8_t status;            // Status for COMMANDs
    } u;
    union {
        char *info;
        char *unit;         // Unit ( Null-terminated string / not sent for type string and folder )
    } s;
} crsf_param_t;


typedef struct {
    uint8_t address;
    uint8_t number_of_params;
    uint8_t params_version;
    uint32_t serial_number;
    uint32_t hardware_id;
    uint32_t firmware_id;
    char name[CRSF_MAX_NAME_LEN];
} crsf_device_t;



typedef struct {
        // common fields
    uint8_t device;            // device index of device parameter belongs to
    uint8_t id;                // Parameter number (starting from 1)
    uint8_t parent;            // Parent folder parameter number of the parent folder, 0 means root
    enum data_type type;  // (Parameter type definitions and hidden bit)
    uint8_t hidden;            // set if hidden
    char *name;           // Null-terminated string
    char *value;          // size depending on data type
    
    char *submenuItems[20];
    int submenuType;

    // field presence depends on type
    char *default_value;  // size depending on data type. Not present for COMMAND.
    int32_t min_value;        // not sent for string type
    int32_t max_value;        // not sent for string type
    int32_t step;             // Step size ( type float only otherwise this entry is not sent )
    uint8_t timeout;           // COMMAND timeout (100ms/count)
    uint8_t changed;           // flag if set needed when edit element is de-selected
    char *max_str;        // Longest choice length for text select
    union {
        uint8_t point;             // Decimal point ( type float only otherwise this entry is not sent )
        uint8_t text_sel;          // current value index for TEXT_SELECTION type
        uint8_t string_max_len;    // String max length ( for string type only )
        uint8_t status;            // Status for COMMANDs
    } u;
    union {
        char *info;
        char *unit;         // Unit ( Null-terminated string / not sent for type string and folder )
    } s;
} menu_items;


extern menu_items mItems[55];
extern menu_items smItems[55];

typedef enum {
    MODULE_UNKNOWN,
    MODULE_ELRS,
    MODULE_OTHER,
} module_type_t;

static uint8_t params_loaded;     // if not zero, number received so far for current device

//setup menus
int selected = 0;
int subSelected = -1;
int entered = -1; 
bool menu_loaded = false;
uint8_t menu_item_id = 0;
uint8_t submenu_item_id = 0;

menu_items get_all_params_from_buffer(menu_items *mItemP, uint8_t *buffer ) ;
void get_divided_submenu_options(menu_items *mItemP );

extern crsf_device_t crsf_devices[CRSF_MAX_DEVICES];
uint8_t protocol_module_is_elrs();

static char *next_string;
#define TEMPSTRINGLENGTH 400 //This is the max dialog size (80 characters * 5 lines)
                             //We could reduce this to ~240 on the 128x64 screens
                             //But only after all sprintf are replaced with snprintf
                             //Maybe move this to target_defs.h
extern char tempstring[TEMPSTRINGLENGTH];

static const char *hdr_str_cb(const void *data);
crsf_param_t *current_param(int absrow);


static void crsfdevice_init();
void bt_handle(uint8_t value);
