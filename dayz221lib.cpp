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
    (*leftMotor).setSpeed(leftSpeed);
    (*rightMotor).setSpeed(rightSpeed);
}

void NikiMotors::move(int speed) {
    (*leftMotor).setSpeed(speed);
    (*rightMotor).setSpeed(speed);
}

void NikiMotors::stop() {
    (*leftMotor).setSpeed(0);
    (*rightMotor).setSpeed(0);
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
        sensors[i] = &LineSensor(pins[i]);
    }
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