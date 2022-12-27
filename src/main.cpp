#include <Arduino.h>

// set orientation of barrels (true if screws turn inwards to drive forward)
// (inwards meaning clockwise on left, anticlockwise on right)
#define THREADS_INWARDS true
// absolute max speed
#define MAX_SPEED 500
// absolute max accel
#define MAX_ACCEL 500

#define BAUD 9600

enum microstep{
	FULL      = 1,
	HALF      = 2,
	QUARTER   = 4,
	EIGHTH    = 8,
	SIXTEENTH = 16
};

enum commandType{
	FORWARD   = 'W', // drive forward
	BACKWARD  = 'S', // drive back
	LEFT      = 'A', // roll left
	RIGHT     = 'D', // roll right
	PIVOT_R   = 'E', // pivot to the right while stationary
	PIVOT_L   = 'Q', // pivot to the left while stationary
	TURN_R    = 'L', // turn left while moving
	TURN_L    = 'J', // turn right while moving
	L_FORWARD = '[', // set left side speed (tank controls)
	R_FORWARD = ']', // set right side speed (tank controls)
	STOP      = 'X', // any unrecognised command will stop
	SET_SPEED = ':', // change max speed
	SET_ACCEL = '@', // change accel
	SET_MICROSTEP = 'M' // change microstep value
};



class continuousStepper{
	int16_t speed=0;
	uint16_t accel=50;
	int16_t targetSpeed=0;
	unsigned long prevStepTime_us=0;
	unsigned long prevAccelTime_ms=0;
	int16_t maxSpeed=200;
	microstep stepsize;
	bool invert;
	bool stepState=0;
	uint8_t EN,DIR,STP,SLP,RST,MS1,MS2,MS3;
	unsigned long prevSpeedMeasureTime=0;
	int16_t stepCount=0;
	int direction=1;
public:
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
		setMicrostep(stepSize);
	}

	void on() {
		digitalWrite(SLP,HIGH); // disable sleep
		digitalWrite(RST,HIGH); // disable reset
		digitalWrite(EN,LOW); // enable driver
	}

	void off() {
		stop();
		speed=0;
		digitalWrite(SLP,LOW); // disable sleep
		digitalWrite(RST,LOW); // disable reset
		digitalWrite(EN,HIGH); // enable driver
	}

	void stop() {targetSpeed=0;};

	// accel in step/s^2
	int16_t setAccel(uint16_t Accel) {
		accel=constrain(Accel,0,MAX_ACCEL);
		return (int16_t)accel;
	}

	int16_t setMicrostep(int16_t microsteps) {
		// Serial.print("mc ");Serial.print(microsteps);
		switch(microsteps) {
			case 1: stepsize=FULL;break;
			case 2: stepsize=HALF;break;
			case 4: stepsize=QUARTER;break;
			case 8: stepsize=EIGHTH;break;
			case 16: stepsize=SIXTEENTH;break;
			default: stepsize=QUARTER;
		}
		switch (stepsize) {
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
		return (int16_t)stepsize;
	}
	int16_t setMaxSpeed(int16_t stepsPerSecond){
		maxSpeed=constrain(stepsPerSecond,0,MAX_SPEED); // 
		return maxSpeed;
	}
	int16_t setSpeed(int16_t Speed) {
		Speed=constrain(Speed,-255,255);
		targetSpeed=map(Speed,-255,255,-maxSpeed,maxSpeed);
		// targetSpeed=((float)(maxSpeed*stepsize)/255)*Speed;
		// Serial.println(targetSpeed);
		return Speed;
	};
	int16_t getAverageSpeed() { // calculated using steps/time since last call of this function
		uint16_t averagespeed = (stepCount/2)/((millis()-prevSpeedMeasureTime)/1000);
		stepCount=0;
		prevSpeedMeasureTime=millis();
		return averagespeed;
	}
	int16_t getSpeed() {return speed;};
	int16_t getTargetSpeed() {return targetSpeed*stepsize;};

	void run() {
		// calculate new speed using accel parameter
		if (millis()-prevAccelTime_ms>1000/accel) {
			prevAccelTime_ms=millis();
			if (speed<(targetSpeed*stepsize)) {
				speed+=stepsize;
				if (speed>(targetSpeed*stepsize)) {
					speed=targetSpeed*stepsize;
				}
			}
			else if (speed>(targetSpeed*stepsize)) {
				speed-=stepsize;
				if (speed<(targetSpeed*stepsize)) {
					speed=targetSpeed*stepsize;
				}
			}
			// set direction based on speed
			if (speed>0) {
				direction=1;
				if (invert) {
					digitalWrite(DIR,HIGH);
				} else {
					digitalWrite(DIR,LOW);
				}
			} else if (speed<0) {
				direction=-1;
				if (invert) {
					digitalWrite(DIR,LOW);
				} else {
					digitalWrite(DIR,HIGH);
				}
			}
		}
		

		
		// only toggle step pin after every half period at the set speed
		if (speed!=0) {
			if (micros()-prevStepTime_us>(unsigned long)1000000/(abs(speed)*2)) {
				prevStepTime_us=micros();
				stepState=!stepState;
				digitalWrite(STP,stepState);
				stepCount+=direction;
			}
		}

		// power saving
		if (speed==0 && targetSpeed==0) {
			off();
		} else {
			on();
		}
	};

};

// Invert motors such that positive speed turns inwards (clockwise on left, anticlockwise on right)
continuousStepper leftMotor(A0,A1,A2,A3,A4,A5,A6,A7,false);
continuousStepper rightMotor(2,3,4,5,6,7,8,9,true);

void setup() {
	Serial.begin(BAUD);
	pinMode(13,INPUT); // used to detect power to motors, prevent sudden current
	leftMotor.init(QUARTER);
	leftMotor.on();
	rightMotor.init(QUARTER);
	rightMotor.on();
}

int16_t speedL=0;
int16_t speedR=0;
int16_t offsetL=0;
int16_t offsetR=0;

void stop(bool print=true) {
	speedL=0;
	speedR=0;
	offsetL=0;
	offsetR=0;
	leftMotor.off();
	rightMotor.off();
	if (print) 
		Serial.print("STOP ");
}

void receiveCommands () {
	if (Serial.available()) {
		String command = Serial.readStringUntil('\n');
		command.toUpperCase();
		byte type=(byte)command[0];
		// Serial.println(type);
		String val = command.substring(1,command.length());
		int16_t value=val.toInt();
		switch(type) {
		case FORWARD: // rotate inwards to drive forward
			value=constrain(value,-255,255);
			Serial.print("FORWARD ");
			speedL=value;
			speedR=value; break;
		case BACKWARD: // rotate outwards to drive backward
			value=constrain(value,-255,255);
			Serial.print("BACKWARD ");
			speedL=-value;
			speedR=-value; break;
		case LEFT: // turn both anticlockwise to roll left
			value=constrain(value,-255,255);
			Serial.print("LEFT ");
			speedL=-value;
			speedR=value; break;
		case RIGHT: // turn both clockwise to roll right
			value=constrain(value,-255,255);
			Serial.print("RIGHT ");
			speedL=value;
			speedR=-value; break;
		case PIVOT_R: // turn by reducing speed on right side 
			value=constrain(value,-255,255);
			Serial.print("PIVOT_R ");
			speedR=-value; 
			speedL=0;break;
		case PIVOT_L: // turn by reducing speed on left side 
			value=constrain(value,-255,255);
			Serial.print("PIVOT_L ");
			speedL=-value; 
			speedR=0; break;
		case TURN_R: // turn by reducing speed on right side 
			value=constrain(value,-255,255);
			Serial.print("TURN_R ");
			offsetR=-value;
			break;
		case TURN_L: // turn by reducing speed on left side 
			value=constrain(value,-255,255);
			Serial.print("TURN_L ");
			offsetL=-value; break;
		case L_FORWARD: // set speed of left side
			value=constrain(value,-255,255);
			Serial.print("L_FORWARD ");
			speedL=value; break;
		case R_FORWARD: // set speed of right side
			value=constrain(value,-255,255);
			Serial.print("R_FORWARD ");
			speedR=value; break;
		case SET_SPEED: // set speed of right side
			stop(); 
			Serial.print("SET_SPEED ");
			value=leftMotor.setMaxSpeed(value);
			value=rightMotor.setMaxSpeed(value); break;
		case SET_ACCEL: // set accel of right side
			stop(); 
			Serial.print("SET_ACCEL ");
			value=leftMotor.setAccel((uint16_t)value);
			value=rightMotor.setAccel((uint16_t)value); break;
		case SET_MICROSTEP: // set speed of right side
			stop();
			Serial.print("SET_MICROSTEP ");
			value=leftMotor.setMicrostep(value);
			value=rightMotor.setMicrostep(value); break;
		default:
			stop();
			value=0;
		}
		// Apply speed with offsets
		leftMotor.setSpeed(speedL+offsetL);
		rightMotor.setSpeed(speedR+offsetR);

		Serial.println(value);
	}
}
long prevPrint=0;
long prevSec=0;

void loop() {
	if (digitalRead(13)==0) { // if no power available, stop motors and set speed to zero
		stop(false);
	} 
	receiveCommands();
	leftMotor.run();
	rightMotor.run();
	if (millis()-prevPrint>1000) {
		prevPrint=millis();
		// Serial.print(">leftMotor:");Serial.println(leftMotor.getSpeed());
		// Serial.print(">rightMotor:");Serial.println(rightMotor.getSpeed());
		// Serial.print(">leftMotorTarget:");Serial.println(leftMotor.getTargetSpeed());
		// Serial.print(">rightMotorTarget:");Serial.println(rightMotor.getTargetSpeed());
	}

	if (millis()-prevSec>2000) {
		prevSec=millis();
		Serial.print("T L:");Serial.print(leftMotor.getTargetSpeed());Serial.print(" R:");Serial.print(rightMotor.getTargetSpeed());
		Serial.print(" C L:");Serial.print(leftMotor.getSpeed());Serial.print(" R:");Serial.print(rightMotor.getSpeed());
		Serial.print(" A L:");Serial.print(leftMotor.getAverageSpeed());Serial.print(" R:");Serial.print(rightMotor.getAverageSpeed());
		Serial.println();
	}

}