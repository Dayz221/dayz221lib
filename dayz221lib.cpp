#include "dayz221lib.h"


Motor::Motor(int directionPin, int pwmPin, float k = 1.0f) {
    this->directionPin = directionPin;
    this->pwmPin = pwmPin;
    this->k = k;
    pinMode(directionPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
}

Motor::Motor(bool isRight = false, float k = 1.0f) {
    if (isRight) {
        this->directionPin = right_dir;
        this->pwmPin = right_pwm;
    } else {
        this->directionPin = left_dir;
        this->pwmPin = left_pwm;
    }
    this->k = k;
    pinMode(directionPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
}

void Motor::setK(float k) {
    this->k = k;
}

void Motor::setSpeed(int speed) {
    speed = max(-255, min(255, speed));

    if (speed > 0) {
        digitalWrite(directionPin, LOW);
        analogWrite(pwmPin, (int)((float)speed*k));
    } else if (speed < 0) {
        digitalWrite(directionPin, HIGH);
        analogWrite(pwmPin, (int)((float)(-speed)*k));
    } else {
        digitalWrite(directionPin, LOW);
        digitalWrite(pwmPin, LOW);
    }
}


NikiMotors::NikiMotors(Motor* leftMotor, Motor* rightMotor, int initialSpeed = 100, uint32_t timeOfOneRotate = 1000) {
    this->leftMotor = leftMotor;
    this->rightMotor = rightMotor;
    this->initialSpeed = initialSpeed;

    this->setTimeOfOneRotate(timeOfOneRotate);
}

void NikiMotors::setTimeOfOneRotate(uint32_t time) {
    this->timeOfOneRotate = time;
}

void NikiMotors::move(int leftSpeed, int rightSpeed) {
    leftMotor->setSpeed(leftSpeed);
    rightMotor->setSpeed(rightSpeed);
}

void NikiMotors::move(int speed) {
    leftMotor->setSpeed(speed);
    rightMotor->setSpeed(speed);
}

void NikiMotors::stop() {
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
}

void NikiMotors::rotate(int degs, int speed = 0) {
    this->stop();
    if (speed == 0) speed = initialSpeed;
    uint32_t time_of_rotate = ((float)degs/360.0)*((float)initialSpeed/speed)*timeOfOneRotate;
    uint32_t timer = millis();
    this->move(-speed, speed);
    while (millis() - timer < time_of_rotate);
    this->stop();
}


Sonic::Sonic(int echo, int trig) {
    this->echo = echo;
    this->trig = trig;
    pinMode(echo, INPUT);
    pinMode(trig, OUTPUT);
}

float Sonic::getDist() {
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    float dst = (float)pulseIn(echo, HIGH)/58.2;
    if (dst < 400) return dst;
    else return 0;
}


LineSensor::LineSensor(int pin) {
    this->pin = pin;
    pinMode(pin, INPUT);
}

bool LineSensor::get() {
    return !digitalRead(pin);
}


LineSensors::LineSensors(int count, int* pins) {
    this->count = count;
    this->sensors = new LineSensor*[count];
    for (int i = 0; i < count; i++) {
        sensors[i] = new LineSensor(pins[i]);
    }
}

LineSensors::LineSensors(const int leftSensor, const int rightSensor) {
    this->count = 2;
    this->sensors = new LineSensor*[2];
    sensors[0] = new LineSensor(leftSensor);
    sensors[1] = new LineSensor(rightSensor);
}

int LineSensors::getBin() {
    int num = 0;
    for (int i = 0; i < count; i++) {
        num += (*sensors[i]).get() << i;
    }
    return num;
}

void LineSensors::getArray(bool* arr) {
    for (int i = 0; i < count; i++) {
        arr[i] = (*sensors[i]).get();
    }
}

float LineSensors::getError() {
    float delta = 1.0/(count-1);
    int cur = this->getBin();
    float answ = 0;
    switch (cur) {
        case 0b00000001: answ = -1; break;
        case 0b00000011: answ = -1 + delta; break;
        case 0b00000010: answ = -1 + delta*2; break;
        case 0b00000110: answ = -1 + delta*3; break;
        case 0b00000100: answ = -1 + delta*4; break;
        case 0b00001100: answ = -1 + delta*5; break;
        case 0b00001000: answ = -1 + delta*6; break;
        case 0b00011000: answ = -1 + delta*7; break;
        case 0b00010000: answ = -1 + delta*8; break;
        case 0b00110000: answ = -1 + delta*9; break;
        case 0b00100000: answ = -1 + delta*10; break;
        case 0b01100000: answ = -1 + delta*11; break;
        case 0b01000000: answ = -1 + delta*12; break;
        case 0b11000000: answ = -1 + delta*13; break;
        case 0b10000000: answ = -1 + delta*14; break;
        default: answ = -2; break;
    };
    return answ;
}

void LineSensors::getError4() {
    bool sensor_state[4];
    int error = 0;

    this->getArray(sensor_state);

    int count_lines = 0;
    for(int i = 0; i < 4; i++){
        count_lines += ((sensor_state[i])? 1:0);
    }
    if(count_lines >= 3){
      return 5; // CROSSROAD;
    }

     
    if((sensor_state[0]==0)&&(sensor_state[1]==0)&&(sensor_state[2]==0)&&(sensor_state[3]==1))
        error=3;
    else if((sensor_state[0]==0)&&(sensor_state[1]==0)&&(sensor_state[2]==1)&&(sensor_state[3]==1))
        error=2;
    else if((sensor_state[0]==0)&&(sensor_state[1]==0)&&(sensor_state[2]==1)&&(sensor_state[3]==0))
        error=1;
    else if((sensor_state[0]==0)&&(sensor_state[1]==1)&&(sensor_state[2]==1)&&(sensor_state[3]==0))
        error=0;
    else if((sensor_state[0]==0)&&(sensor_state[1]==1)&&(sensor_state[2]==0)&&(sensor_state[3]==0))
        error=-1;
    else if((sensor_state[0]==1)&&(sensor_state[1]==1)&&(sensor_state[2]==0)&&(sensor_state[3]==0))
        error=-2;
    else if((sensor_state[0]==1)&&(sensor_state[1]==0)&&(sensor_state[2]==0)&&(sensor_state[3]==0))
        error=-3;
    else if((sensor_state[0]==0)&&(sensor_state[1]==0)&&(sensor_state[2]==0)&&(sensor_state[3]==0)) {
        if (last_err<=-3) error=-4;
        else error=4;
    }

    last_err = error;
    
    return error;
}

void LineSensors::debug() {
    Serial.print("Sensors: ");
    bool arr[count];
    this->getArray(arr);
    for (auto a : arr) {
        Serial.print(a);
        Serial.print(" ");
    }
    Serial.println();
    Serial.print("Error: ");
    Serial.println(this->getError());
    Serial.println("");
}

LineFollower::LineFollower(NikiMotors* motors, LineSensors* sensors) {
    this->motors = motors;
    this->sensors = sensors;
}

void LineFollower::follow(int speed, float k = 1.0) {
    float err = sensors->getError();
    if (err == -2) motors->stop();
    else motors->move((int)((float)speed*(1-err*k)), (int)((float)speed*(1+err*k)));
}

void LineFollower::followUntilCrossroad(int speed) {
    int initial_motor_speed = spe;
    int delta_power[4] = {8, 18, 32, 38};
    int error = this->sensors->getError4();
     
    while(error != 5){
        if (error > 0) {
            this->motors->move(speed+delta_power[error-1], speed-delta_power[error-1]);
        } else if (error < 0) {
            this->motors->move(speed-delta_power[(-error)-1], speed+delta_power[(-error)-1]);
        } else {
            this->motors->move(speed);
        }

        error = getError();
    }
}

void LineFollower::stop() {
    motors->stop();
}


PID::PID(float* input, float* output, float* setpoint, float kp, float ki, float kd, float dt, float minOut, float maxOut) {
    this->input = input;
    this->output = output;
    this->setpoint = setpoint;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->minOut = minOut;
    this->maxOut = maxOut;
}

void PID::compute() {
    float _input = (*input);
    float _setpoint = (*setpoint);
    float p = _setpoint - _input;
    float i = max(minOut, min(maxOut, prev_i + p*(dt/1000)*ki));
    float d = (p-prev_err)/(dt/1000);
    (*output) = max(minOut, min(maxOut, p*kp + i + d*kd));
    prev_i = i;
    prev_err = p;
}


bool Timer::get(uint32_t delta) {
    if (millis() - last >= delta) {
        last = millis();
        return true;
    }
    return false;
}