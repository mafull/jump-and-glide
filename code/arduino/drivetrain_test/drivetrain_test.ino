#include <Wire.h>
#include <Adafruit_INA219.h>


#define PIN_LED_STATUS            13
#define PIN_MOTOR                 10


Adafruit_INA219 ina219;

String inputString = "";
bool stringData = false;
bool motorOn = false;


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

  // Current sensor
  ina219.begin();  
}


void loop() 
{
  if(stringData)
  {
      if(inputString == "start")
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
      inputString = "";
      stringData = false;      
  }
  Serial.print("Current = ");  Serial.print(ina219.getCurrent_mA());  Serial.println(" mA");
  Serial.print("Shunt V = ");  Serial.print(ina219.getShuntVoltage_mV());  Serial.println(" mV");
  Serial.print("Bus V = ");  Serial.print(ina219.getBusVoltage_V());  Serial.println(" V\n");
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

