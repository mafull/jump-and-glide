#include <Servo.h>

#define PIN_LED_STATUS     13
#define PIN_MOTOR          10
#define PIN_SERVO_CLUTCH   9
#define PIN_SERVO_WING_L   12
#define PIN_SERVO_WING_R   11
#define DISENGAGE_ANGLE    25
#define MIDDLE_ANGLE       105

Servo clutchServo;
Servo wingServoL;
Servo wingServoR;

String inputString = "";
bool stringData = false;


void setup() 
{
  // Initialise serial
  Serial.begin(115200);
  while(!Serial){};
  Serial.println("Serial initialised");

  // LED
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_STATUS, LOW);
  delay(1000);
  digitalWrite(PIN_LED_STATUS, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_STATUS, LOW);

  // Initialise motor
  pinMode(PIN_MOTOR, OUTPUT);
  
  // Initialise servos
  clutchServo.attach(PIN_SERVO_CLUTCH);
  clutchServo.write(MIDDLE_ANGLE);
  wingServoL.attach(PIN_SERVO_WING_L);
  wingServoL.write(90);  
  wingServoR.attach(PIN_SERVO_WING_R);
  wingServoR.write(90);  
}


void loop() 
{
  if(stringData)
  {
      if(inputString == "clutch_l")
      {
        clutchServo.write(MIDDLE_ANGLE-DISENGAGE_ANGLE);
        Serial.println("Clutch left");        
      }
      else if(inputString == "clutch_r")
      {
        clutchServo.write(MIDDLE_ANGLE+DISENGAGE_ANGLE);
        Serial.println("Clutch right");
      }
      else if(inputString == "clutch_m")
      {
        clutchServo.write(MIDDLE_ANGLE);
        Serial.println("Clutch middle");
      }    
      else if(inputString == "start")
      {
        digitalWrite(PIN_MOTOR, HIGH);
        digitalWrite(PIN_LED_STATUS, HIGH);
        Serial.println("Start motor");        
      }
      else if(inputString == "stop")
      {
        digitalWrite(PIN_MOTOR, LOW);
        digitalWrite(PIN_LED_STATUS, LOW);
        Serial.println("Stop motor");
      }
      else if(inputString == "wings_l")
      {
        wingServoL.write(40);
        wingServoR.write(40);
        Serial.println("Wings left");
      }      
      else if(inputString == "wings_r")
      {
        wingServoL.write(130);
        wingServoR.write(130);
        Serial.println("Wings right");
      }         
      else if(inputString == "wings_m")
      {
        wingServoL.write(90);
        wingServoR.write(90);
        Serial.println("Wings middle");
      }         
      else if(inputString == "wings_d")
      {
        wingServoL.write(130);
        wingServoR.write(40);
        Serial.println("Wings dive");
      }          
      else if(inputString == "wings_p")
      {
        wingServoL.write(40);
        wingServoR.write(130);
        Serial.println("Wings pitch");
      }          
      inputString = "";
      stringData = false;      
  }
  delay(500);
}


void serialEventRun()
{
  while(Serial.available())
  {
    char inChar = (char)Serial.read();
    if (inChar =='\n') {stringData = true; return;}
    inputString = inputString + inChar;
  }    
}

