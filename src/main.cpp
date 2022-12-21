#include <Arduino.h>


enum microstep{
FULL      = 1,
HALF      = 2,
QUARTER   = 4,
EIGHTH    = 8,
SIXTEENTH = 16
};

enum commandType{
FORWARD   = 'W',
BACKWARD  = 'S',
LEFT      = 'A',
RIGHT     = 'D',
TURN_R    = 'E',
TURN_L    = 'Q',
STOP      = 'X'
};

class continuousStepper{
	int16_t speed=0;
	uint16_t accel=0;
	int16_t targetSpeed=0;
	long prevStepTime_us=0;
	long prevAccelTime_ms=0;
	const uint16_t maxSpeed=100;
	microstep stepsize;
	bool invert;
	bool stepState=0;
public:
	uint8_t EN,DIR,STP,SLP,RST,MS1,MS2,MS3;
	continuousStepper(	uint8_t dir,
						uint8_t stp,
						uint8_t slp,
						uint8_t rst,
						uint8_t ms3,
						uint8_t ms2,
						uint8_t ms1,
						uint8_t en, 
						bool Invert=false) : DIR(dir) ,STP(stp) ,SLP(slp) ,RST(rst), MS3(ms3), MS2(ms2), MS1(ms1), EN(en), invert(Invert){};

	void init(microstep stepSize=FULL) {
		// set up pins
		pinMode(EN,OUTPUT);
		pinMode(DIR,OUTPUT);
		pinMode(STP,OUTPUT);
		pinMode(SLP,OUTPUT);
		pinMode(RST,OUTPUT);
		pinMode(MS1,OUTPUT);
		pinMode(MS2,OUTPUT);
		pinMode(MS3,OUTPUT);
		// disable driver
		off();
		// set microstepping
		stepsize=stepSize;
		switch (stepSize) {
		case FULL: 
			digitalWrite(MS1,LOW);
			digitalWrite(MS2,LOW);
			digitalWrite(MS3,LOW); break;
		case HALF: 
			digitalWrite(MS1,HIGH);
			digitalWrite(MS2,LOW);
			digitalWrite(MS3,LOW); break;
		case QUARTER: 
			digitalWrite(MS1,LOW);
			digitalWrite(MS2,HIGH);
			digitalWrite(MS3,LOW); break;
		case EIGHTH: 
			digitalWrite(MS1,HIGH);
			digitalWrite(MS2,HIGH);
			digitalWrite(MS3,LOW); break;
		case SIXTEENTH: 
			digitalWrite(MS1,HIGH);
			digitalWrite(MS2,HIGH);
			digitalWrite(MS3,HIGH); break;
		default: 
			digitalWrite(MS1,LOW);
			digitalWrite(MS2,LOW);
			digitalWrite(MS3,LOW);
		}
	}

	void on() {
		digitalWrite(SLP,HIGH); // disable sleep
		digitalWrite(RST,HIGH); // disable reset
		digitalWrite(EN,LOW); // enable driver
	}

	void off() {
		digitalWrite(SLP,LOW); // disable sleep
		digitalWrite(RST,LOW); // disable reset
		digitalWrite(EN,HIGH); // enable driver
	}

	void stop() {targetSpeed=0;};

	// accel in step/s^2
	void accel(uint16_t Accel) {
		accel=Accel;
	}
	void speed(int16_t Speed) {
		targetSpeed=((float)(maxSpeed*stepsize)/255)*Speed;
	};
	int16_t getSpeed() {return speed;};
	void run() {
		// calculate new speed using accel parameter
		if (millis()-prevAccelTime_ms>1000/accel) {
			if (targetSpeed>speed)
				speed+=accel/1000;
				if (speed>targetSpeed) {
					speed=targetSpeed;
				}
			else if (targetSpeed<speed) {
				speed-=accel/1000;
				if (speed<targetSpeed) {
					speed=targetSpeed;
				}
			}
		}
		// run at commanded speed (temporary, no accel)
		// speed=targetSpeed;

		// set direction based on speed
		if (speed>0) {
			if (invert) {
				digitalWrite(DIR,HIGH);
			} else {
				digitalWrite(DIR,LOW);
			}
			} else if (speed<0) {
			if (invert) {
				digitalWrite(DIR,LOW);
			} else {
				digitalWrite(DIR,HIGH);
			}
		}
		// only toggle step pin after every half period at the set speed
		if (micros()-prevStepTime_us>1000000/(speed*2)) {
			prevStepTime_us=micros();
			~stepState;
			digitalWrite(STP,stepState);
		}
	};

};

continuousStepper leftMotor(A0,A1,A2,A3,A4,A5,A6,A7,false);
continuousStepper rightMotor(2,3,4,5,6,7,8,9,false);

void setup() {
	Serial.begin(9600);
	leftMotor.init(FULL);
	leftMotor.on();
	rightMotor.init(FULL);
	rightMotor.on();
}

void receiveCommands () {
	while (Serial.available()) {
		String command = Serial.readStringUntil('/n');
		byte type=(byte)command.begin();
		command.remove(0);
		byte value=command.toInt();
		switch(type) {
		case FORWARD:
			leftMotor.speed(value);
			rightMotor.speed(-value); break;
		case BACKWARD:
			leftMotor.speed(-value);
			rightMotor.speed(value); break;
		case LEFT:
			leftMotor.speed(value);
			rightMotor.speed(value); break;
		case RIGHT:
			leftMotor.speed(-value);
			rightMotor.speed(-value); break;
		case TURN_R:
			leftMotor.speed(value);
			rightMotor.speed(0); break;
		case TURN_L:
			leftMotor.speed(0);
			rightMotor.speed(value); break;
		default:
			leftMotor.stop();
			rightMotor.stop();
		}
	}
}
long prevPrint=0;

void loop() {
	receiveCommands();
	leftMotor.run();
	rightMotor.run();
	if (millis()-prevPrint>100) {
		Serial.print(">leftMotor:")
	}
}