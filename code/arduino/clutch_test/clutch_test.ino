#include <Servo.h>

#define PIN_LED_STATUS     13
#define PIN_SERVO          9
#define DISENGAGE_ANGLE    25
#define MIDDLE_ANGLE       105

Servo myservo;

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

  // Initialise servo
  myservo.attach(PIN_SERVO);
  myservo.write(90);
}


void loop() 
{
  if(stringData)
  {
      if(inputString == "L")
      {
        myservo.write(MIDDLE_ANGLE-DISENGAGE_ANGLE);
        Serial.println("Servo left");        
      }
      else if(inputString == "R")
      {
        myservo.write(MIDDLE_ANGLE+DISENGAGE_ANGLE);
        Serial.println("Servo right");
      }
      else if(inputString == "M")
      {
        myservo.write(MIDDLE_ANGLE);
        Serial.println("Servo middle");
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

