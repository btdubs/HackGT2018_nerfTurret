#include <Servo.h>

#define SHOOTING_STATE 0
#define REV_STATE 1
#define MOVING_STATE 2
#define WAIT_STATE 3
#define STOPPED 4


//Pin Definitions
Servo pitchServo; //Servo for tilting mechanism
const int pitchPin = 3;
Servo yeetServo;  //Servo for shooting mechanism
const int yeetPin = 6;
Servo yawServo;  //Servo for turntable mechanism
const int yawPin= 9;
const int errorLEDPin = 2;
const int statLED0 = 16;
const int statLED1 = 14;
const int relayPin = 15;  //pin to control motor contactor

//Servo Positions
const int triggerRelease = 0;  //not shooting
const int triggerDepress = 180;  //shooting

const int maxYaw = 180;
const int minYaw = 0;

const int maxPitch = 180;
const int minPitch = 0;

//Time Delays
const int revTime = 1000;  //Time for shooter motor to start up
const int fireRestPeriod = 500; //Time between shots

void setup() {
  // put your setup code here, to run once:
  pitchServo.attach(pitchPin);
  yeetServo.attach(yeetPin);
  yawServo.attach(yawPin);

  pinMode(relayPin, OUTPUT);
  pinMode(errorLEDPin, OUTPUT);
  pinMode(statLED0, OUTPUT);
  pinMode(statLED1, OUTPUT);
  Serial.begin(9600);
}

//Serial Messages
int pitchTarget;
int yawTarget;
bool targetSeen;
bool fireReady;

bool isTimedOut = false;
int lastMessageTime = millis();
int state = 1;
int startRevTime = millis();

void loop() {
  // put your main code here, to run repeatedly:

  bool gotMessage = getMessage();

  //if we haven't received a message from the NUC in a while, stop driving
  if(gotMessage) {
    isTimedOut = false;
    lastMessageTime = millis();
  } 
  else if ((lastMessageTime + 1000) < millis()) {
    isTimedOut = true;
    state = STOPPED;
  } else{}
  
  switch(state){
    case SHOOTING_STATE:  //presses and depresses the trigger
      yeetServo.write(triggerDepress);
      delay(100);
      yeetServo.write(triggerRelease);
      delay(fireRestPeriod);
      if(!fireReady){
        state = MOVING_STATE;
      }
      digitalWrite(errorLEDPin,0);
      digitalWrite(statLED0, 1);
      digitalWrite(statLED1, 1);
      break;
    case REV_STATE:  //starts rev of shooter motor
      startRevTime = millis();
      state = WAIT_STATE;
      digitalWrite(relayPin,1);
      digitalWrite(errorLEDPin, 0);
      digitalWrite(statLED0, 0);
      digitalWrite(statLED1, 1);
      break;
    case WAIT_STATE:  //waits for minimum time to rev motor
      if((millis()-startRevTime) > revTime && fireReady){
        state = SHOOTING_STATE;
      }
      digitalWrite(errorLEDPin, 0);
      digitalWrite(statLED0, 1);
      digitalWrite(statLED1, 0);
      break;
    case MOVING_STATE: //aims at target based on given target angles
      digitalWrite(relayPin,0);
      yawServo.write(yawTarget);
      pitchServo.write(pitchTarget);
      if(targetSeen){
        state = REV_STATE;
      }
      digitalWrite(errorLEDPin, 0);
      digitalWrite(statLED0, 0);
      digitalWrite(statLED1, 0);
      break;
    case STOPPED:
      //Serial.println("error");
      digitalWrite(errorLEDPin, 1);
      digitalWrite(statLED0, 0);
      digitalWrite(statLED1, 0);
      break;
  }

}

bool getMessage()
{
  bool gotMessage = false;
  while(Serial.available())
  {
    if(Serial.read() == '$')
    { 
      gotMessage = true;
      yawTarget = Serial.parseInt();
      pitchTarget = Serial.parseInt();
      targetSeen = Serial.parseInt() == 1;
      fireReady = Serial.parseInt() == 1;
      yawTarget = min(maxYaw, max(yawTarget, minYaw));
      pitchTarget = min(maxPitch, max(pitchTarget, minPitch));
    }
  }
  return gotMessage;
}

