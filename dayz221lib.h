#include <Arduino.h>


#define left_pwm 7
#define left_dir 6
#define right_pwm 4
#define right_dir 5


class Motor {
private:
    int directionPin, pwmPin;
    float k;
public:
    Motor(int directionPin, int pwmPin, float k = 1.0);
    Motor(bool isRight = false, float k = 1.0);
    void setK(float k);
    void setSpeed(int speed);
};


class NikiMotors {
private:
    Motor* leftMotor;
    Motor* rightMotor;
    uint32_t timeOfOneRotate;
    int initialSpeed;
public:
    NikiMotors(Motor* leftMotor, Motor* rightMotor, int initialSpeed = 100, uint32_t timeOfOneRotate = 1000);
    void setTimeOfOneRotate(uint32_t time);
    void rotate(int degs, int speed = 0);
    void move(int leftSpeed, int rightSpeed);
    void move(int speed);
    void stop();
};


class Sonic {
private:
    int echo, trig;
public:
    Sonic(int echo, int trig);
    float getDist();
};


class LineSensor {
private:
    int pin;
public:
    LineSensor(int pin);
    bool get();
};


class LineSensors {
private:
    int count;
    LineSensor** sensors;

public:
    LineSensors(int count, int* pins);
    int getBin();
    void getArray(bool* arr);
};