// #include <Arduino.h>
// #include <WProgram.h>

/*
 * $Id$
 */
#define ENABLE_IRREMOTE 1
#define ENABLE_SERVO    1
#define ENABLE_PING     0

#include <AFMotor.h>
#include <NewPing.h>

#if ENABLE_SERVO==1
//#include <Servo.h>
#endif

#include <SimpleSoftwareServo.h>

#if ENABLE_IRREMOTE == 1
  #include <IRremote.h>  
#endif

#define APP_NAME "Robot " __FILE__
#define APP_VERSION "0.1"
#define APP_REVISION "$Rev$"
#define APP_DATE __DATE__

// Motor driver
AF_DCMotor motorL(1), motorR(2);

SimpleSoftwareServo servo1;

#define COMMAND_TIME           500     // milliseconds
#define SERVO_TIME             500
#define CYCLE_TIME             20

#define DIR_STOP               0
#define DIR_FORWARD            1
#define DIR_BACKWARD           2
#define DIR_LEFT               3
#define DIR_RIGHT              4


#define MODE_MANUAL            0
#define MODE_AUTONOMOUS        1

volatile uint8_t direction = DIR_STOP;

// PINs
#define LED_PIN                 13
#define HC_SR04_TRIGGER_PIN     A3
#define HC_SR04_ECHO_PIN        A2

#define MAX_DISTANCE            300 // sonar maximum cm
#define FAR_AWAY                999
#define SERVO1_PIN              10

#define SERVO_MAX               (140-20)
#define SERVO_LEFT              SERVO_MAX
#define SERVO_CENTER            70
#define SERVO_RIGHT             20

#define IR_RECV_PIN             A0
#define ON_OFF_SWITCH_PIN       A1
#define BLUE_LED_PIN            A5


// Kathrein Fernbedienung
#define IR_MASK                0x07FF
#define IR_KEY_RIGHT           0x07B1
#define IR_KEY_UP              0x07B0
#define IR_KEY_LEFT            0x07B3
#define IR_KEY_DOWN            0x07B2
#define IR_KEY_OK              0x07B4
#define IR_KEY_ON_OFF          0x0799
#define IR_KEY_START           0x07BB // 0x0798 // 
#define IR_KEY_STOP            0x07BF // 0x0797 // 
#define IR_KEY_PAUSE           0x07BD
#define IR_KEY_PPLUS           0x07A2
#define IR_KEY_PMINUS          0x07A3
#define IR_KEY_BLUE            0x07AC

// Fuer 

#define NUM_PINGS            5

NewPing sonar(HC_SR04_TRIGGER_PIN, HC_SR04_ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
volatile unsigned int distance = 0;     // distance sensors [cm]

// Infrared receiver
#if ENABLE_IRREMOTE==1
  IRrecv irrecv(IR_RECV_PIN);
  decode_results results;
#endif

volatile uint8_t currentState = 0;
volatile uint8_t mode = MODE_MANUAL;
volatile unsigned long lastCommandTime = 0;
volatile int16_t angle = SERVO_CENTER;
volatile bool blueLedOn = false;
volatile unsigned long lastSwitchTime = 0;

void isort(int *a, int n){
// *a is an array pointer function
  for (int i = 1; i < n; ++i)
  {
    int j = a[i];
    int k;
    for (k = i - 1; (k >= 0) && (j < a[k]); k--)
    {
      a[k + 1] = a[k];
    }
    a[k + 1] = j;
  }
}

unsigned int getDistance()
{
  int d[NUM_PINGS];
  for (uint8_t i=0; i<NUM_PINGS; ++i)
  {
    delay(5);
    d[i] = sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
    if(d[i] == 0)
      d[i] = FAR_AWAY;
  }
  isort(d, NUM_PINGS);
  return d[NUM_PINGS/2];
}

void updateDistance()
{
  distance = getDistance();
}

void updateServo()
{
   servo1.write(angle);
   unsigned long t = millis();
   while (millis() - t <= SERVO_TIME)
   {
     SimpleSoftwareServo::refresh();
     delay(10);
   }
}

void lookAround(uint16_t dist[5])
{
  
  int16_t i;
  uint8_t idx = 0;
  Serial.print("dist: ");
  for (i=SERVO_LEFT; i>=SERVO_RIGHT; i-=25)
  {
    angle = i;
    updateServo();
    updateDistance();
    Serial.print(distance, DEC);
    Serial.print(" ");
    if (dist != NULL)
      dist[idx] = distance;
    ++idx;
  }
  Serial.println();
  angle = SERVO_CENTER;
  updateServo();
  delay(1000);
}

void updateDirection(uint16_t key)
{
  bool cmd = false;
  switch(key)
  {
    case IR_KEY_UP:
      direction = DIR_FORWARD;
      cmd = true;
      break;
      
   case IR_KEY_DOWN:
     direction = DIR_BACKWARD;
     cmd = true;
     break;
     
  case IR_KEY_LEFT:
      direction = DIR_LEFT;
      cmd = true;
      break;
      
  case IR_KEY_RIGHT:
    direction = DIR_RIGHT;
    cmd = true;
    break;
  
  case IR_KEY_START:
    mode = MODE_AUTONOMOUS;
    break;
    
  case IR_KEY_PAUSE:
  case IR_KEY_STOP:
    mode = MODE_MANUAL;
    break;

  case IR_KEY_PPLUS:
    if (angle <= SERVO_MAX)
      angle += 20;
    updateServo();
    break;
    
  case IR_KEY_PMINUS:
    if (angle > 0)
      angle -= 20;
    updateServo();
    break;
    
  case IR_KEY_OK:
    Serial.println("LOOKAROUND");
    lookAround(NULL);
    break;

  case IR_KEY_BLUE:
    if ((millis() - lastSwitchTime) >= 500) {
      blueLedOn = !blueLedOn;
      lastSwitchTime = millis();
    }
    break;
    
    
  default:
       direction = DIR_STOP;   
  }
  if (cmd)
  {
    lastCommandTime = millis();
  }
}

void setup()
{
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println(APP_NAME " " APP_VERSION " " APP_REVISION " " APP_DATE " b");
  
  // setup infrared
  irrecv.blink13(0);
  irrecv.enableIRIn();
  
  // setup motor driver
  motorL.setSpeed(200);
  motorL.run(RELEASE);
  motorR.setSpeed(200);
  motorR.run(RELEASE);
  
  // setup servo
  servo1.attach(SERVO1_PIN);
  angle = SERVO_CENTER;
  updateServo();

  pinMode(BLUE_LED_PIN, OUTPUT);
  
}

void updateIrRemote()
{
  if (irrecv.decode(&results))
  {
    results.value &= IR_MASK;
    updateDirection(results.value);
    Serial.print("IR : ");
    Serial.println(results.value, HEX);
    Serial.print("MODE : ");
    Serial.println(direction, DEC);
    irrecv.resume();
  }
}

void drive(uint8_t direction)
{
  uint8_t speed = 0;
  switch(direction)
  {
    case DIR_FORWARD:
      motorL.run(FORWARD);
      motorR.run(FORWARD);
      speed = 255;
      if (distance != 0)
      {
        if (distance < 40)
          speed = 200;
        if (distance < 20)
          speed = 164;
        if (distance < 10)
          speed = 0;
      }
      break;
      
    case DIR_BACKWARD:
      speed = 255;
      motorL.run(BACKWARD);
      motorR.run(BACKWARD);
      break;
      
   case DIR_LEFT:
      if (mode == MODE_MANUAL)
        speed = 170;
     else
       speed = 170;   
      motorL.run(BACKWARD);
      motorR.run(FORWARD);
      break;
      
    case DIR_RIGHT:
      if (mode == MODE_MANUAL)
        speed = 170;
     else
       speed = 170; 
      motorL.run(FORWARD);
      motorR.run(BACKWARD);
      break;
      
    default:
      motorL.run(RELEASE);
      motorR.run(RELEASE);
  }
  
  motorL.setSpeed(((uint16_t)speed*8)/10);
  motorR.setSpeed(speed);

  digitalWrite(BLUE_LED_PIN, blueLedOn);
}


void loop()
{
  static uint8_t cycle = 0;
  static unsigned long lastTime = millis();
  static unsigned long cycleTimer = millis();
  static unsigned long lastAction = 0;
  
  ++cycle;
  while (millis() - cycleTimer < CYCLE_TIME)
  {
    delay(1);
  }
  //SimpleSoftwareServo::refresh();
  cycleTimer = millis();
  
  updateIrRemote();
  // updateDistance();

  if (mode == MODE_AUTONOMOUS)
  {
    direction = DIR_FORWARD;
    if ((distance <= 10) ||(distance <= 20 && (millis() - lastTime) > 1000))
    {
      drive(DIR_BACKWARD);
      delay(500);
      if (random(2) == 0)
        drive(DIR_LEFT);
      else
        drive(DIR_RIGHT);  
      delay(random(500,1500));
      lastTime = millis();
    }
    
    if ((distance <= 80) && ((millis() - lastTime) > 3000))
    {
      uint16_t dist[5];
      drive(DIR_STOP);
      delay(500);
      lookAround(dist);
      delay(500);
      uint8_t imax = 2;
      for(uint8_t i=0; i<5; ++i)
      {
        if (dist[i] > dist[imax])
        {
          imax = i;
        }
      }
      if (dist[2] <= 20)
      {
        drive(DIR_BACKWARD);
        delay(800);
        if (random(2) == 0)
          drive(DIR_LEFT);
        else
          drive(DIR_RIGHT);  
        delay(random(500,1500));
      }
      else
      {
        if (imax < 2)
        {
          drive(DIR_LEFT);
        }
        if (imax > 2)
        {
          drive(DIR_RIGHT);
        } 
        if (imax==0 || imax==4)
          delay(500);
        if (imax==1 || imax==3)
          delay(250);
      }
      lastTime = millis();
      lastAction = lastTime;
    }
    drive(direction);
  }
  else
  {
    if (millis() - lastCommandTime > COMMAND_TIME)
    {
      direction = DIR_STOP;
    }
    drive(direction);
  }
  static unsigned long lastSerialInfo = 0;
  if (millis() - lastSerialInfo >= 2000)
  {
    Serial.print("DIST : ");
    Serial.print(distance, DEC);
    Serial.print(" cm ");
    Serial.print("ANGLE : ");
    Serial.println(angle, DEC);
    lastSerialInfo = millis();
  }
}

