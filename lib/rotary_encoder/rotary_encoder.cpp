#include <Arduino.h>
#include "rotary_encoder.h"
#include "uart.h"

uint8_t encoderInit() {

	ESP32Encoder::useInternalWeakPullResistors=UP;
	encoder.attachSingleEdge(12, 4);
    //set starting count value after attaching
	encoder.setCount(0);

	db_out.println("Encoder Start = " + String((int32_t)encoder.getCount()));
}

uint8_t getRE_POS() {
    
    //delay(100);
    int readEncoder= encoder.getCount()/2;
    int out=0;
    if ( readEncoder > rotary_encoder_last_pos ) {
        db_out.printf("up :%i\n",readEncoder);
        out =  1;
    }
    else if (readEncoder < rotary_encoder_last_pos) {
        db_out.printf("down :%i\n",readEncoder);
        out = 2;
  } 
	//db_out.printf("Encoder count:%i:%i \n",readEncoder,rotary_encoder_last_pos);
    rotary_encoder_last_pos = readEncoder;

return out;
}