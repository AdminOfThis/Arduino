#import <LowPower.h>
#import <Servo.h>
#import <Math.h>

#define PIN_HAND 9
#define PIN_TOP 8
#define PIN_SWITCH 3

#define POS_HAND_LOW 0
#define POS_HAND_HIGH 172

#define POS_TOP_LOW 180
#define POS_TOP_HIGH 40

#define MAX_RAGE 7


//Need to declare all functions
void r1f1();
void r2f1();
void r3f1();
void r4f1();
void r5f1();
void r6f1();
void r7f1();

void (*functions[][10])() = { {r1f1}, {r2f1}, {r3f1}, {r4f1}, {r5f1}, {r6f1}, {r7f1}};

int numFunctions[7] = {1,1,1,1,1,1,1};

Servo servoHand;
Servo servoTop;

long lastEvent;

double rage = 0;

bool active = LOW;

void setup() {
	
	Serial.begin(115200);
	Serial.println("=== UselessBox v0.1 ===");
	
	randomSeed(A0); //Intialize random function with noise
	
	servoHand.attach(PIN_HAND);
	servoTop.attach(PIN_TOP);
	
	servoHand.write(POS_HAND_LOW);
	servoTop.write(POS_TOP_HIGH); //Open top for 5 s to attach
	delay(3000);
	servoTop.write(POS_TOP_LOW);
	
	pinMode(PIN_SWITCH, INPUT_PULLUP);
	delay(10);

	attachInterrupt(digitalPinToInterrupt(PIN_SWITCH), interrupt, FALLING);
	
	lastEvent = millis();

}

void loop() {
	
	long timeSinceLast = millis() -lastEvent;
	
	double newRage = rage-(timeSinceLast*timeSinceLast)*.0000001;
	rage = max(0, newRage);
		
	//Check if Switch is up
	if(!digitalRead(PIN_SWITCH)) {
		active = HIGH;
		}
	
	// Do action
	if(active) {
		
		double newRage = 1+rage*rage*random(30,50)/25.0;
		rage = min(MAX_RAGE, newRage);
		
		
		detachInterrupt(interrupt);
		
		//Serial.print("Rage: ");
		//Serial.println(rage);
		
		//Choose action
		doRandomFunction();
		
		lastEvent = millis();
		active = LOW;
	}
	
	
	
	//Check if done, then go to sleep
	if(!active) {	
		Serial.println("Going to sleep");
		servoHand.write(POS_HAND_LOW);
		servoTop.write(POS_TOP_LOW);
		delay(500);
		if(digitalRead(PIN_SWITCH)) {
			attachInterrupt(digitalPinToInterrupt(PIN_SWITCH), interrupt, FALLING);
			LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
		}
	}

}

void interrupt() {
	Serial.println("Interrupted");
}

//Chooses a random Action based on the rage level
void doRandomFunction() {
	
	//Chooses a rage level to pick a function from
	int chosenRage = random(0, round(rage-1));
	
	//Chooses a function based on how many there are
	int chosenFunction = random(0, numFunctions[chosenRage]-1);
	
	//Calls the function by ragelevel and indice
	functions[chosenRage][chosenFunction]();
	
	
}

void r1f1() {
	servoTop.write(POS_TOP_HIGH);
	delay(50);
	servoHand.write(POS_HAND_HIGH);
	while(!digitalRead(PIN_SWITCH)) {
		delay(10);
	}
	servoHand.write(POS_HAND_LOW);
	delay(50);
	servoTop.write(POS_TOP_LOW);
	delay(500);
}

void r2f1()  {
	int x = random(3,5);
	for(int i=0;i<x;i++) {
		servoTop.write(POS_TOP_HIGH);
		delay(220);
		servoTop.write(POS_TOP_LOW);
		delay(250);
	}
	delay(random(0, 3000));
	servoTop.write(POS_TOP_HIGH);
	delay(50);
	servoHand.write(POS_HAND_HIGH);
	while(!digitalRead(PIN_SWITCH)) {
		delay(10);
	}
	servoHand.write(POS_HAND_LOW);
	delay(50);
	servoTop.write(POS_TOP_LOW);
	delay(500);
}
void r3f1()  {
	servoTop.write(POS_TOP_HIGH);
	for(int i = POS_HAND_LOW;i<=POS_HAND_HIGH;i++) {
		servoHand.write(i);
		delay(10);
	}
	delay(50);
	servoHand.write(POS_HAND_LOW);
	delay(50);
	servoTop.write(POS_TOP_LOW);
	delay(500);
}
void r4f1()  {
	servoTop.write(POS_TOP_HIGH);
	delay(50);
	servoHand.write(POS_HAND_HIGH-40);
	delay(random(500, 4000));
	servoHand.write(POS_HAND_HIGH);
	while(!digitalRead(PIN_SWITCH)) {
		delay(10);
	}
	servoHand.write(POS_HAND_LOW);
	delay(50);
	servoTop.write(POS_TOP_LOW);
	delay(500);
}
void r5f1()  {
	for(int i=0;i<random(1,5);i++) {
		int pos = POS_TOP_HIGH+random(10,60);
		servoTop.write(pos);
		delay(random(500, 2000));
		int speed = random(5,20);
		for(int j = pos;j<=POS_TOP_LOW;j++) {
			servoTop.write(j);
			delay(speed);
		}
		delay(random(0,1500));
	}
	servoTop.write(POS_TOP_HIGH);
	delay(50);
	servoHand.write(POS_HAND_HIGH);
	while(!digitalRead(PIN_SWITCH)) {
		delay(10);
	}
	servoHand.write(POS_HAND_LOW);
	delay(50);
	servoTop.write(POS_TOP_LOW);
	delay(500);
}
void r6f1()  {
	int x = random(5,10);
	for(int i=0;i<x;i++) {
		servoTop.write(POS_TOP_HIGH);
		delay(50);
		servoTop.write(POS_TOP_LOW);
		delay(100);
	}
	delay(random(0, 3000));
	servoTop.write(POS_TOP_HIGH);
	delay(50);
	servoHand.write(POS_HAND_HIGH);
	while(!digitalRead(PIN_SWITCH)) {
		delay(10);
	}
	servoHand.write(POS_HAND_LOW);
	delay(50);
	servoTop.write(POS_TOP_LOW);
	delay(500);
}
void r7f1()  {
	for(int i=POS_TOP_LOW;i>POS_TOP_HIGH;i--) {
		servoTop.write(i);
		delay(25);
	}
	delay(random(0,3000));
	for(int i=POS_HAND_LOW;i<=POS_HAND_HIGH-40;i++) {
		servoHand.write(i);
		delay(25);
	}
	delay(random(0,3000));
	servoHand.write(POS_HAND_HIGH);
	while(!digitalRead(PIN_SWITCH)) {
		delay(10);
	}
	servoHand.write(POS_HAND_LOW);
	delay(50);
	servoTop.write(POS_TOP_LOW);
	delay(500);
	
}
