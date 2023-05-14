#pragma once

#ifndef ADS_H /* include guards */
#define ADS_H

#include "crsf.h"

class ADSreader
{

public:
  void init();
  void readInputs(rc_input_t *rc_input);

};

#endif