#include "config.h"
#include "crsf.h"
#include "menus.h"
#include "uart.h"
#include "rx_params.h"

bool powerChangeHasRun = false;

int Aileron_value = 0;        // values read from the pot 
int Elevator_value = 0; 
int Throttle_value=0;
int Rudder_value = 0; 

int Arm = 0;        // switch values read from the digital pin
int FlightMode = 0; 

uint32_t tickTime = 0;
uint16_t rates[] = { 0, 25, 50, 100, 150, 200 };


void crsfdevice_init() {
    next_param = 1;
    next_chunk = 0;
    recv_param_ptr = recv_param_buffer;
    params_loaded = 0;
    //CBUF_Init(send_buf);
}

void  update_packet_rate(uint32_t currentMicros) {
    
    if (currentMicros > tickTime ) {      
        dbout.printf("tick %u\n", rxConected );
    
       //for (size_t i = 0;crsf_devices[i].address; i++) dbout.printf("device address: 0x%x\n",crsf_devices[i].address);
        
        
        uint8_t tmp =  LinkStatistics.rf_Mode;
        if (MODULE_IS_ELRS) {            
            if ( (int)local_info.good_pkts != (int)rates[tmp] ) {
                dbout.printf("get elrs tx module info\n");
                CRSF_get_elrs_info(crsfCmdPacket,ELRS_ADDRESS);
                elrsWrite(crsfCmdPacket,8,0);
            }
            if (rxConected == 0) {
                crsf_devices[1].address = 0;
                CRSF_ping_devices();
                dbout.printf("no rx found - broadcasting ping..\n");
            } else {
                if (crsf_devices[1].address == 0) {

                    CRSF_ping_devices();
              
                }
                else {
                    if (rx_params_loaded < crsf_devices[1].number_of_params) {
                        dbout.printf("read rx info\n");
                        next_param = 1;
                        next_chunk = 0;
                        CRSF_read_param(crsfCmdPacket,next_param, next_chunk, ELRS_RX_ADDRESS);
                        elrsWrite(crsfCmdPacket,8,200000);
                    }
                }
            }
        } else {
            local_info.good_pkts = 0;
            dbout.printf("no module found\n");
        }
        tickTime = currentMicros + 1000000;
        rxConected = 0;
    }
}

const char * hdr_str_cb(const void *data) {
    
    (void)data;
     //   dbout.printf("call params: %u: %i\n",count_params_loaded(), device_idx);

    if (count_params_loaded(0) != crsf_devices[device_idx].number_of_params) {
        dbout.printf("not all params: %u: %i\n",count_params_loaded(0), device_idx);
    
        snprintf(tempstring, sizeof tempstring, "%s %s", crsf_devices[device_idx].name, "LOADING");
    
    } else if (protocol_module_is_elrs()) {
        dbout.printf("idx_elrs: %i\n",device_idx);

        snprintf(tempstring, sizeof tempstring, "%s  %d/%d  %c",
                 crsf_devices[device_idx].name, elrs_info.bad_pkts, elrs_info.good_pkts,
                 (elrs_info.flags & 1) ? 'C' : '-');
    } else  {
        dbout.printf("tx module not detected\n");
        //return crsf_devices[device_idx].name;
        snprintf(tempstring, sizeof tempstring, "%s  %d/%d  %c",
                 crsf_devices[device_idx].name, elrs_info.bad_pkts, elrs_info.good_pkts,
                 (elrs_info.flags & 1) ? 'C' : '-');
    }
    return tempstring;
}

void bt_handle(uint8_t value) {
  dbout.println("bt_handle");
  
  powerChangeHasRun=true;   

  clickCurrentMicros = crsfTime + 500000;//0.5sec
  dbout.printf("times: %u:%u\n", clickCurrentMicros/1000, crsfTime/1000);
  //powerChangeHasRun=true;
  
  //CRSF_read_param(crsfCmdPacket,1,next_chunk);
  //elrsWrite(crsfCmdPacket,8,0);
  
  //buildElrsPingPacket(crsfCmdPacket);
  //dbout.println(CRSF_send_model_id(2));
  
  //set modelId
  //CRSF_sendId(crsfSetIdPacket,0);
  //elrsWrite(crsfSetIdPacket,LinkStatisticsFrameLength);

  //turn on rx wifi, even if missmatch modelId
  //buildElrsPacket(crsfCmdPacket,16,1);

  CRSF_read_param(crsfCmdPacket,1,next_chunk,ELRS_ADDRESS);
  elrsWrite(crsfCmdPacket,8,200000);
  //serialEvent();
}

