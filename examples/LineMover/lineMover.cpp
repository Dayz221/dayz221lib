#include <Arduino.h>
#include "dayz221lib.h"

Motor leftMotor(0);
Motor rightMotor(1);
NikiMotors motors(&leftMotor, &rightMotor);

int lineSensorsPins[4] = {8, 9, 11, 12};
LineSensors sensors(4, lineSensorsPins);

float weights[4] = {-1, -0.5, 0.5, 1};

void setup() {

}

Timer timer;
void loop() {
    static bool vals[4];
    float sum = 0;
    float weightedSum = 0;
    float err = 0;

    if (timer.get(50)) {
        sensors.getArray(vals);
        for (int i = 0; i < 4; i++) {
            sum += (int)vals[i];
            weightedSum += weights[i]*(int)vals[i];
        }
        if (sum != 0.0) err = weightedSum/sum;
        motors.move((int)(100*(1-err)), (int)(100*(1+err)));
    }
}