#include "Arduino.h"
#define NUM_PROTO_OPTS 8

#define NUM_VIRT_CHANNELS 10
#define VIRT_NAME_LEN 10
#define NUM_OUT_CHANNELS 16
#define NUM_TX_INPUTS (INP_LAST - 1)
#define NUM_INPUTS (NUM_TX_INPUTS)
#define NUM_SOURCES (NUM_INPUTS + NUM_CHANNELS + MAX_PPM_IN_CHANNELS)

#define NUM_CHANNELS (NUM_OUT_CHANNELS + NUM_VIRT_CHANNELS)
#define MAX_PPM_IN_CHANNELS 8
enum {
    PROTO_OPTS_BITRATE,
    LAST_PROTO_OPT,
};
enum {
    INP_NONE,
    INP_CAP,//#include "capabilities.h"
    INP_LAST,
};

typedef enum {
    MIXER_ADVANCED = 0x01,
    MIXER_STANDARD = 0x02,
} MixerMode;
enum Protocols {
    PROTOCOL_NONE = 0,
    #include "protocol/protocol.h"
    PROTOCOL_COUNT,
};
enum TxPower {
    TXPOWER_100uW,
    TXPOWER_300uW,
    TXPOWER_1mW,
    TXPOWER_3mW,
    TXPOWER_10mW,
    TXPOWER_30mW,
    TXPOWER_100mW,
    TXPOWER_150mW,
    TXPOWER_LAST,
};
enum SwashType {
    SWASH_TYPE_NONE,
    SWASH_TYPE_120,
    SWASH_TYPE_120X,
    SWASH_TYPE_140,
    SWASH_TYPE_90,
    SWASH_TYPE_LAST,
};

enum Radio {
    CYRF6936,
    A7105,
    CC2500,
    NRF24L01,
    MULTIMOD,
    R9M,
    TX_MODULE_LAST,
};
enum ModelType {
    MODELTYPE_HELI,
    MODELTYPE_PLANE,
    MODELTYPE_MULTI,
};
struct Model {
    uint32_t fixed_id;
    enum ModelType type;
    enum Protocols protocol;
    int16_t proto_opts[NUM_PROTO_OPTS];
    uint8_t num_channels;
    uint8_t num_ppmin_channels;
    uint16_t ppmin_centerpw;
    uint16_t ppmin_deltapw;
    uint8_t train_sw;
    enum Radio radio;
    enum TxPower tx_power;
    enum SwashType swash_type;
    uint8_t swash_invert;
    uint8_t swashmix[3];
    char name[24];
    char icon[24];
    char virtname[NUM_VIRT_CHANNELS][VIRT_NAME_LEN];
    uint8_t templates[NUM_CHANNELS];
    uint8_t safety[NUM_SOURCES+1];
    MixerMode mixer_mode;
    int8_t ppm_map[MAX_PPM_IN_CHANNELS];
    uint8_t ppmin_mode;
    uint8_t padding_1[1];
#if HAS_PERMANENT_TIMER
    u32 permanent_timer;
#endif
#if HAS_VIDEO
    u8 videosrc;
    u8 videoch;
    s8 video_contrast;
    s8 video_brightness;
#endif
};
extern struct Model Model;
struct Model Model;
