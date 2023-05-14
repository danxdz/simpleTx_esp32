#include "prefs.h"
#include <Preferences.h>
#include "uart.h"
#include "crsf.h"
#include "ads.h"

void Prefs::init(rc_input_t *rc_input)
{
    Preferences preferences;
    preferences.begin("elrs", false);
    unsigned int inputCalibration = preferences.getUInt("calibration", 0);

    while (inputCalibration == 0)
    {
        dbout.printf("calibration not found\n");
        boolean cal = calibrateStickValues(rc_input);
        if (cal)
        {
            boolean isValid = validateStickCalibration(rc_input);
            dbout.printf("calibration valid: %u\n", isValid);
            if (isValid)
            {
                inputCalibration++;
                preferences.putUInt("calibration", inputCalibration);
            }
        }
    }

    // Print the counter to Serial Monitor
    dbout.printf("Current calibration saved: %u\n", inputCalibration);

    // Store the counter to the Preferences

    // Close the Preferences
    preferences.end();
}

boolean Prefs::calibrateStickValues(rc_input_t *rc_input)
{
    ADSreader ads_reader;

    Preferences preferences;
    preferences.begin("elrs", false);

    stick_calibration_t sticks;
    stick_calibration_t oldSticks;

    readStickCalibrationPrefs(&sticks);

    dbout.printf("Starting calibration\n");
    dbout.printf("Center sticks\n");
    delay(2000);
    // Realizar leituras contínuas do ADC para detectar os valores máximos e mínimos dos sticks
    const unsigned long calibrationDuration = 5000; // Duração da calibração em milissegundos
    const unsigned long calibrationEndTime = millis() + calibrationDuration;

    oldSticks.aileronCenter = rc_input->aileron;
    oldSticks.elevatorCenter = rc_input->elevator;
    oldSticks.throttleCenter = rc_input->throttle;
    oldSticks.rudderCenter = rc_input->rudder;

    delay(500);


    dbout.printf("Center values: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", oldSticks.aileronCenter, oldSticks.elevatorCenter, oldSticks.throttleCenter, oldSticks.rudderCenter);

    dbout.printf("Move sticks to their maximum and minimum positions\n");

    // Atualizar os valores máximos e mínimos dos sticks

    while (millis() < calibrationEndTime)
    {
        ads_reader.readInputs(rc_input);

        // Ler os valores dos sticks do ADC
        int aileron = rc_input->aileron;
        int throttle = rc_input->throttle;
        int rudder = rc_input->rudder;
        int elevator = rc_input->elevator;

        // Atualizar os valores máximos e mínimos dos sticks
        sticks.aileronMax = max(sticks.aileronMax, aileron);
        sticks.elevatorMax = max(sticks.elevatorMax, elevator);
        sticks.throttleMax = max(sticks.throttleMax, throttle);
        sticks.rudderMax = max(sticks.rudderMax, rudder);

        sticks.aileronMin = min(sticks.aileronMin, aileron);
        sticks.elevatorMin = min(sticks.elevatorMin, elevator);
        sticks.throttleMin = min(sticks.throttleMin, throttle);
        sticks.rudderMin = min(sticks.rudderMin, rudder);

        // Exibir os valores lidos durante a calibração
        if (!debugEnabled)
            dbout.printf("Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", aileron, elevator, throttle, rudder);
    }
    delay(1000);
    dbout.printf("Calibration finished\n");
    dbout.printf("Max values: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", sticks.aileronMax, sticks.elevatorMax, sticks.throttleMax, sticks.rudderMax);
    dbout.printf("Min values: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", sticks.aileronMin, sticks.elevatorMin, sticks.throttleMin, sticks.rudderMin);
    dbout.printf("Center values: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", oldSticks.aileronCenter, oldSticks.elevatorCenter, oldSticks.throttleCenter, oldSticks.rudderCenter);

    saveSticks(&sticks);

    return true;
}

void Prefs::saveSticks(stick_calibration_t *stick_calibration)
{
    Preferences preferences;
    preferences.begin("elrs", false);

    preferences.putInt("aileron_max", stick_calibration->aileronMax);
    preferences.putInt("elevator_max", stick_calibration->elevatorMax);
    preferences.putInt("throttle_max", stick_calibration->throttleMax);
    preferences.putInt("rudder_max", stick_calibration->rudderMax);

    preferences.putInt("aileron_min", stick_calibration->aileronMin);
    preferences.putInt("elevator_min", stick_calibration->elevatorMin);
    preferences.putInt("throttle_min", stick_calibration->throttleMin);
    preferences.putInt("rudder_min", stick_calibration->rudderMin);

    preferences.putInt("aileron_center", stick_calibration->aileronCenter);
    preferences.putInt("elevator_center", stick_calibration->elevatorCenter);
    preferences.putInt("throttle_center", stick_calibration->throttleCenter);
    preferences.putInt("rudder_center", stick_calibration->rudderCenter);

    preferences.end();
}


bool Prefs::validateStickCalibration(rc_input_t *rc_input)
{
    Preferences preferences;
    preferences.begin("elrs", false);
    stick_calibration_t sticks;

    readStickCalibrationPrefs(&sticks);

    int aileron = rc_input->aileron;
    int elevator = rc_input->elevator;
    int throttle = rc_input->throttle;
    int rudder = rc_input->rudder;

    // Calcular as diferenças entre os valores máximos e mínimos para cada canal
    int diffAileron = sticks.aileronMax - sticks.aileronMin;
    int diffElevator = sticks.elevatorMax - sticks.elevatorMin;
    int diffThrottle = sticks.throttleMax - sticks.throttleMin;
    int diffRudder = sticks.rudderMax - sticks.rudderMin;

    // Definir um limite mínimo para considerar os sticks calibrados
    double minDiff = 2000;

    // Calcular a tolerância para cada canal (50% da diferença, mas não menor que o limite mínimo)
    int toleranceAileron = max(diffAileron * 0.5, minDiff);
    int toleranceElevator = max(diffElevator * 0.5, minDiff);
    int toleranceThrottle = max(diffThrottle * 0.5, minDiff);
    int toleranceRudder = max(diffRudder * 0.5, minDiff);
    dbout.printf("Tolerance: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", toleranceAileron, toleranceElevator, toleranceThrottle, toleranceRudder);

    // Verificar se os valores atuais estão dentro dos limites da calibração
    bool isAileronValid = abs(aileron - sticks.aileronCenter) >= toleranceAileron;
    bool isElevatorValid = abs(elevator - sticks.elevatorCenter) >= toleranceElevator;
    bool isThrottleValid = abs(throttle - sticks.throttleCenter) >= toleranceThrottle;
    bool isRudderValid = abs(rudder - sticks.rudderCenter) >= toleranceRudder;

    // Verificar se a diferença entre os valores máximos e mínimos é maior que o limite mínimo
    bool isAileronCalibrated = diffAileron > minDiff;
    bool isElevatorCalibrated = diffElevator > minDiff;
    bool isThrottleCalibrated = diffThrottle > minDiff;
    bool isRudderCalibrated = diffRudder > minDiff;

    // Exibir os resultados da validação
    dbout.printf("Aileron valid: %s\n", isAileronValid ? "true" : "false");
    dbout.printf("Elevator valid: %s\n", isElevatorValid ? "true" : "false");
    dbout.printf("Throttle valid: %s\n", isThrottleValid ? "true" : "false");
    dbout.printf("Rudder valid: %s\n", isRudderValid ? "true" : "false");

    dbout.printf("Aileron calibrated: %s\n", isAileronCalibrated ? "true" : "false");
    dbout.printf("Elevator calibrated: %s\n", isElevatorCalibrated ? "true" : "false");
    dbout.printf("Throttle calibrated: %s\n", isThrottleCalibrated ? "true" : "false");
    dbout.printf("Rudder calibrated: %s\n", isRudderCalibrated ? "true" : "false");

    // Fechar as preferências
    preferences.end();

    // Verificar se todos os canais estão válidos e calibrados
    return isAileronValid && isElevatorValid && isThrottleValid && isRudderValid && isAileronCalibrated && isElevatorCalibrated && isThrottleCalibrated && isRudderCalibrated;
}













// Read saved sticks center, max and min values from preferences
void Prefs::readStickCalibrationPrefs(stick_calibration_t *stickCalibration)
{
    Preferences preferences;
    preferences.begin("elrs", false);

    // Ler os valores das preferências
    stickCalibration->aileronMax = preferences.getInt("aileron_max", 0);
    stickCalibration->elevatorMax = preferences.getInt("elevator_max", 0);
    stickCalibration->throttleMax = preferences.getInt("throttle_max", 0);
    stickCalibration->rudderMax = preferences.getInt("rudder_max", 0);

    stickCalibration->aileronMin = preferences.getInt("aileron_min", 0);
    stickCalibration->elevatorMin = preferences.getInt("elevator_min", 0);
    stickCalibration->throttleMin = preferences.getInt("throttle_min", 0);
    stickCalibration->rudderMin = preferences.getInt("rudder_min", 0);

    stickCalibration->aileronCenter = preferences.getInt("aileron_center", 0);
    stickCalibration->elevatorCenter = preferences.getInt("elevator_center", 0);
    stickCalibration->throttleCenter = preferences.getInt("throttle_center", 0);
    stickCalibration->rudderCenter = preferences.getInt("rudder_center", 0);



    stickCalibration->aileronMin = (stickCalibration->aileronMin == 0) ? STICK_CENTER : stickCalibration->aileronMin;
    stickCalibration->elevatorMin = (stickCalibration->elevatorMin == 0) ? STICK_CENTER : stickCalibration->elevatorMin;
    stickCalibration->throttleMin = (stickCalibration->throttleMin == 0) ? STICK_CENTER : stickCalibration->throttleMin;
    stickCalibration->rudderMin = (stickCalibration->rudderMin == 0) ? STICK_CENTER : stickCalibration->rudderMin;
    
    stickCalibration->aileronCenter = (stickCalibration->aileronCenter == 0) ? STICK_CENTER : stickCalibration->aileronCenter;
    stickCalibration->elevatorCenter = (stickCalibration->elevatorCenter == 0) ? STICK_CENTER : stickCalibration->elevatorCenter;
    stickCalibration->throttleCenter = (stickCalibration->throttleCenter == 0) ? STICK_CENTER : stickCalibration->throttleCenter;
    stickCalibration->rudderCenter = (stickCalibration->rudderCenter == 0) ? STICK_CENTER : stickCalibration->rudderCenter;

    // Fechar as preferências
    preferences.end();

    // Exibir os valores lidos
    if (debugEnabled)
    {
        dbout.printf("Saved Max values: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", stickCalibration->aileronMax, stickCalibration->elevatorMax, stickCalibration->throttleMax, stickCalibration->rudderMax);
        dbout.printf("Saved Min values: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", stickCalibration->aileronMin, stickCalibration->elevatorMin, stickCalibration->throttleMin, stickCalibration->rudderMin);
        dbout.printf("Saved Center values: Aileron: %d, Elevator: %d, Throttle: %d, Rudder: %d\n", stickCalibration->aileronCenter, stickCalibration->elevatorCenter, stickCalibration->throttleCenter, stickCalibration->rudderCenter);
    }
}