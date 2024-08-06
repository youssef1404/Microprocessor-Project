#include <REG51.H>
//#include <AT89X51.H>

typedef unsigned short int   uint8_t;

uint8_t period, ON_Period, OFF_Period;
uint8_t ONperiod, OFFperiod;
float pTerm, iTerm, dTerm;
uint8_t error;
uint8_t previousError;

float kp = 11; //11
float ki = 0;
float kd = 11; //11

float output;
uint8_t integral, derivative;

sbit irSensor1 = P2^5;  // IR sensor pins
sbit irSensor2 = P2^6;
sbit irSensor3 = P2^7;

sbit motor1Forward = P1^5;
sbit motor1Backward = P1^4;
sbit motor1pwmPin = P3^2;
sbit motor2Forward = P1^3;
sbit motor2Backward = P1^2;
sbit motor2pwmPin = P3^1;

uint8_t motor1newSpeed;
uint8_t motor2newSpeed;
uint8_t motor2Speed = 40; //Default 70
uint8_t motor1Speed = 40; //Default 120


uint8_t irReadings[3];

void setPWM1(uint8_t motor1newSpeed); 
void setPWM2(uint8_t motor2newSpeed); 

void delay(uint8_t time) {
    uint8_t i, j;
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
   
		setPWM1(motor1newSpeed);
		setPWM2(motor2newSpeed);
    
    motor2Forward = 1;
    motor2Backward = 0;
    motor1Forward = 1;
    motor1Backward = 0;
		
		
}

void setPWM1(uint8_t motor1newSpeed) {
		period = 65535 - 256;  // FFFF - period
		ONperiod = 65535 - motor1newSpeed ;
		OFFperiod = period + ONperiod ; // 65535 - 256(period) + ontime
}

void setPWM2(uint8_t motor2newSpeed) {
		period = 65535 - 256;  // FFFF - period
		ONperiod = 65535 - motor2newSpeed ;
		OFFperiod = period + ONperiod ; // 65535 - 256(period) + ontime
}

void setTimer() {
		TMOD = 0x01;
		if(motor1newSpeed > 1)
		{
			TH0 = (ON_Period >> 8);
			TL0 = ON_Period;
		}	
		else
		{
			TH0 = (OFF_Period >> 8);
			TL0 = OFF_Period;
		}	
	
		if(motor2newSpeed > 1)
		{
			TH0 = (ON_Period >> 8);
			TL0 = ON_Period;
		}	
		else
		{
			TH0 = (OFF_Period >> 8);
			TL0 = OFF_Period;
		}	

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
