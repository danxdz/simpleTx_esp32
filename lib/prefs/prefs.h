#pragma once

#include "crsf.h"

typedef struct {
  int aileronMax = 0;
    int aileronMin = 0;
    int aileronCenter = 0;
    int elevatorMax = 0;
    int elevatorMin = 0;
    int elevatorCenter = 0;
    int throttleMax = 0;
    int throttleMin = 0;
    int throttleCenter = 0;
    int rudderMax = 0;
    int rudderMin = 0;
    int rudderCenter = 0;
} stick_calibration_t;

#define STICK_HIGH 15000
#define STICK_LOW 0
#define STICK_CENTER 7500

class Prefs
{



public:
    void init(rc_input_t *rc_input);
    bool validateStickCalibration(rc_input_t *rc_input);
    boolean calibrateStickValues(rc_input_t *rc_input);
    void readStickCalibrationPrefs(stick_calibration_t *stick_calibration);
    void saveSticks(stick_calibration_t *stick_calibration);

};
