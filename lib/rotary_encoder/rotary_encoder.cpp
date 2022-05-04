#include <Arduino.h>
#include "rotary_encoder.h"
#include "uart.h"

int rotary_encoder_last_pos = 0;

uint8_t encoderInit() {

	ESP32Encoder::useInternalWeakPullResistors=UP;
	encoder.attachSingleEdge(12, 4);
    //set starting count value after attaching
	encoder.setCount(0);

	dbout.println("Encoder Start = " + String((int32_t)encoder.getCount()));
}

uint8_t get_encoder_pos() {
    
    //delay(100);
    int readEncoder= encoder.getCount()/2;
    int out=0;
    if ( readEncoder > rotary_encoder_last_pos ) {
        dbout.printf("up :%i\n",readEncoder);
        out =  1;
    }
    else if (readEncoder < rotary_encoder_last_pos) {
        dbout.printf("down :%i\n",readEncoder);
        out = 2;
  } 
	//dbout.printf("Encoder count:%i:%i \n",readEncoder,rotary_encoder_last_pos);
    rotary_encoder_last_pos = readEncoder;

return out;
}