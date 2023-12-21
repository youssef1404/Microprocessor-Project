#include <REG51.H>
//#include <AT89X51.H>


float pTerm, iTerm, dTerm;
int error;
int previousError;

float kp = 11; //11
float ki = 0;
float kd = 11; //11

float output;
int integral, derivative;

sbit irSensor1 = P2^5;  // IR sensor pins
sbit irSensor2 = P2^6;
sbit irSensor3 = P2^7;

sbit motor1Forward = P1^5;
sbit motor1Backward = P1^4;
sbit motor1pwmPin = P3^2;
sbit motor2Forward = P1^3;
sbit motor2Backward = P1^2;
sbit motor2pwmPin = P3^1;

int motor1newSpeed;
int motor2newSpeed;
int motor2Speed = 40; //Default 70
int motor1Speed = 40; //Default 120


int irReadings[3];

void delay(unsigned int time) {
    unsigned int i, j;
    for (i = 0; i < time; i++)
        for (j = 0; j < 1275; j++);
}

void readIRSensors() {
    // Read the IR sensors and put the readings in irReadings array
    irReadings[0] = irSensor1;
    irReadings[1] = irSensor2;
    irReadings[2] = irSensor3;
}

void calculateError() {
    // Determine an error based on the readings
    if (irReadings[0] && irReadings[1] && !irReadings[2]) {
        error = 2;
    } else if (irReadings[0] && !irReadings[1] && !irReadings[2]) {
        error = 1;
    } else if (irReadings[0] && !irReadings[1] && irReadings[2]) {
        error = 0;
    } else if (!irReadings[0] && !irReadings[1] && irReadings[2]) {
        error = -1;
    } else if (!irReadings[0] && irReadings[1] && irReadings[2]) {
        error = -2;
    } else if (irReadings[0] && irReadings[1] && irReadings[2]) {
        if (previousError == -2) {
            error = -3;
        } else {
            error = 3;
        }
    } else if (!irReadings[0] && !irReadings[1] && !irReadings[2]) {
        error = 0;
    }
}

void pidCalculations() {
    pTerm = kp * error;
    integral += error;
    iTerm = ki * integral;
    derivative = error - previousError;
    dTerm = kd * derivative;
    output = pTerm + iTerm + dTerm;
    previousError = error;
}

void changeMotorSpeed() {
    // Change motor speed of both motors accordingly
    motor2newSpeed = motor2Speed + output;
    motor1newSpeed = motor1Speed - output;
    // Constrain the new speed of motors to be between the range 0-255
    motor2newSpeed = (motor2newSpeed > 255) ? 255 : motor2newSpeed;
    motor1newSpeed = (motor1newSpeed > 255) ? 255 : motor1newSpeed;
    // Set new speed, and run motors in the forward direction
    motor2pwmPin = motor2newSpeed;
    motor1pwmPin = motor1newSpeed;
    motor2Forward = 1;
    motor2Backward = 0;
    motor1Forward = 1;
    motor1Backward = 0;
}

void main() {
    while (1) {
        readIRSensors();
        calculateError();
        pidCalculations();
        changeMotorSpeed();
        delay(100);  // Adjust the delay as needed
    }
}
