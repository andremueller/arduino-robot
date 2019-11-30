
// #include <Arduino.h>
// #include <WProgram.h>

/*
 * $Id$
 */
#define ENABLE_IRREMOTE 1

#include <AFMotor.h>
//#include <NewPing.h>
#include <IRremote.h>

#define APP_VERSION "0.1"
#define APP_REVISION "$Rev$"

// Motor driver
AF_DCMotor motorL(1), motorR(2);

// PINs
#define LED_PIN                 13
#define HC_SR04_TRIGGER_PIN     10
#define HC_SR04_ECHO_PIN        9
#define MAX_DISTANCE            200 // cm

#define IR_RECV_PIN       A0

// Fuer Kathrein Fernbedienung
#define IR_KEY_RIGHT           1969
#define IR_KEY_UP              1968
#define IR_KEY_LEFT            4019
#define IR_KEY_DOWN            4018
#define IR_KEY_OK              4020
#define IR_KEY_ON_OFF          1945

//NewPing sonar(HC_SR04_TRIGGER_PIN, HC_SR04_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
volatile unsigned int distance = 0;     // distance sensors [cm]

// Infrared receiver
IRrecv irrecv(IR_RECV_PIN);
//decode_results results;

volatile uint8_t currentState = 0;

void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Roboter " __FILE__ " " APP_VERSION " " APP_REVISION);
  
  // set input pins to 20 kOhms to Vcc
  /*
  pinMode(ON_OFF_SWITCH_PIN, INPUT);
  pinMode(BUMPER_PIN, INPUT);
  digitalWrite(ON_OFF_SWITCH_PIN, HIGH);
  digitalWrite(BUMPER_PIN, HIGH);
  */
  
    // setup motor driver
 
  motorL.run(RELEASE);
  motorR.run(RELEASE);
  irrecv.blink13(0);
  irrecv.enableIRIn();

  delay(100);

      motorL.run(FORWARD);
        motorR.run(FORWARD);
        motorL.setSpeed(255);
        motorR.setSpeed(255);

  delay(1000);
  motorL.run(RELEASE);
  motorR.run(RELEASE);
}


void loop()
{
  decode_results results;
  if (irrecv.decode(&results))
  {
        motorL.run(FORWARD);
        motorR.run(FORWARD);
        motorL.setSpeed(255);
        motorL.setSpeed(255);
        switch(results.value)
        {
          case IR_KEY_UP:
            motorL.run(FORWARD);
            motorR.run(FORWARD);
            motorL.setSpeed(255);
            motorL.setSpeed(255);
            break;
          case IR_KEY_DOWN:
          
            motorL.run(RELEASE);
            motorR.run(RELEASE);
            break;
          }
        Serial.print("IR : ");
        Serial.println(results.value, DEC);
        irrecv.resume();
  }
}

