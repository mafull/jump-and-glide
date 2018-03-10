// Requires the following libraries to compile:
//  https://github.com/adafruit/Adafruit_ASFcore
//  https://github.com/adafruit/Adafruit_ZeroTimer
//  TODO - https://github.com/cmaglie/FlashStorage
//
// Pinout: https://cdn-learn.adafruit.com/assets/assets/000/046/240/original/microcomputers_Adafruit_Feather_32u4_Basic_Proto_v2.3-1.png?1504884949

#include <Adafruit_ZeroTimer.h>

#define LOOP_FREQUENCY_HZ     100
#define PWM_FREQUENCY_HZ      100

#define PIN_LED_STATUS        13
#define PIN_MOTOR_WINDING     6
#define PIN_SERVO_CLUTCH      11
#define PIN_SERVO_LEFT_WING   9
#define PIN_SERVO_RIGHT_WING  10


typedef enum State_t {
  IDLE,
  WINDING,
  JUMPING,
  GLIDING,

  NUM_STATES
} State;


State state = GLIDING;

float pitch = 0.0f;
float roll = 0.0f;
float heading = 6.9f;

Adafruit_ZeroTimer zt3 = Adafruit_ZeroTimer(3);
Adafruit_ZeroTimer zt5 = Adafruit_ZeroTimer(5);





#include <string.h>
#include <stdlib.h>
char *dtostrf(double val, int width, unsigned int prec, char *sout)
{
  int decpt, sign, reqd, pad;
  const char *s, *e;
  char *p;
  s = fcvt(val, prec, &decpt, &sign);
  if (prec == 0 && decpt == 0) {
  s = (*s < '5') ? "0" : "1";
    reqd = 1;
  } else {
    reqd = strlen(s);
    if (reqd > decpt) reqd++;
    if (decpt == 0) reqd++;
  }
  if (sign) reqd++;
  p = sout;
  e = p + reqd;
  pad = width - reqd;
  if (pad > 0) {
    e += pad;
    while (pad-- > 0) *p++ = ' ';
  }
  if (sign) *p++ = '-';
  if (decpt <= 0 && prec > 0) {
    *p++ = '0';
    *p++ = '.';
    e++;
    while ( decpt < 0 ) {
      decpt++;
      *p++ = '0';
    }
  }    
  while (p < e) {
    *p++ = *s++;
    if (p == e) break;
    if (--decpt == 0) *p++ = '.';
  }
  if (width < 0) {
    pad = (reqd + width) * -1;
    while (pad-- > 0) *p++ = ' ';
  }
  *p = 0;
  return sout;
}






void advanceState() {
  state = (State)((int)state + 1);
  if(state == NUM_STATES) state = IDLE;
}


void serialEvent() {
  digitalWrite(PIN_LED_STATUS, digitalRead(PIN_LED_STATUS) == HIGH ? LOW : HIGH);
  while(Serial.available()) {
    char c = (char)Serial.read();
    
    if(c == 'a') advanceState();
  }
}


void setup() {
  while(!Serial);
  
  // ---- Initialise serial ----
  Serial.begin(115200);
  
  // ---- Initialise pins ----
  pinMode(PIN_LED_STATUS, OUTPUT);
  pinMode(PIN_MOTOR_WINDING, OUTPUT);
  pinMode(PIN_SERVO_CLUTCH, OUTPUT);
  pinMode(PIN_SERVO_LEFT_WING, OUTPUT);
  pinMode(PIN_SERVO_RIGHT_WING, OUTPUT);
  pinMode(5, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(5), advanceState, RISING);

  // ---- Initialise timers ----
  // Timer 3
  //  16-bit
  //  period = 65535
  //  PWM
  //    CH0 -> D2 (or D10)
  //    CH1 -> D5 (or D12)
  zt3.configure(TC_CLOCK_PRESCALER_DIV1,
                TC_COUNTER_SIZE_16BIT,
                TC_WAVE_GENERATION_NORMAL_PWM);
  if(!zt3.PWMout(true, 0, 10)) {
    Serial.println("Failed to configure PWM output! (Timer 3, CH0, pin D10)");
  }
  if(!zt3.PWMout(true, 1, 12)) {
    Serial.println("Failed to configure PWM output! (Timer 3, CH1, pin D12)");
  }
  zt3.setCompare(0, 0xFFFF/4);
  zt3.setCompare(1, 0xFFFF/2);
  zt3.enable(true);

  // Timer 5
  //  16-bit
  //  period = 1000
  //  PWM
  //    CH0 -> MOSI
  //    CH1 -> SCK #UNUSED#
  zt5.configure(TC_CLOCK_PRESCALER_DIV1,
                TC_COUNTER_SIZE_16BIT,
                TC_WAVE_GENERATION_MATCH_PWM);
  zt5.setPeriodMatch(1000, 200);
  if(!zt5.PWMout(true, 1, SCK)) {
    Serial.println("Failed to configure PWM output! (Timer 5, CH0, pin MOSI)");
  }
  //zt5.setCompare(0, 0xFFFF/4);
  zt5.enable(true);


  
  // Initialise loop timer
}


void controlLoop() {
  bool windingMotorOn = false;
  int clutchServoDC = 0;
  int leftWingServoDC = 0;
  int rightWingServoDC = 0;

  // ---- Inputs ----
  // READ SENSORS


  int ain = analogRead(0);// * (5.0/3.3);
  ain = constrain(ain, 0, 1024);

  // ---- State machine ----
  switch(state) {
    case IDLE:

      break;

    case WINDING:
      // Winding motor on
      windingMotorOn = true;
      break;

    case JUMPING:
      clutchServoDC = 1024;

      break;

    case GLIDING:
      clutchServoDC = 1024;


      // Use pitch/roll requirements to generate servo outputs
      // ...
      leftWingServoDC = ain;
      rightWingServoDC = 1024 - ain;
      clutchServoDC = ain;
    
      break;

    default:
      // WHY ARE YOU HERE
      break;
  }


  // ---- Outputs ----
  // Winding motor
  digitalWrite(PIN_MOTOR_WINDING, windingMotorOn);
  
  // Wing servos
  zt3.setCompare(0, (leftWingServoDC * 64));
  zt3.setCompare(1, (rightWingServoDC * 64));
  
  // Clutch servo
  zt5.setPeriodMatch(1000, constrain(leftWingServoDC, 0, 1000));
  

  // ---- Serial output ----
  char str[128] = "";
  char pitchStr[5] = "";
  char rollStr[5] = "";
  char headingStr[5] = "";
  dtostrf(pitch, 4, 2, pitchStr);
  dtostrf(roll, 4, 2, rollStr);
  dtostrf(heading, 4, 2, headingStr);
  sprintf(str, "%01d|-> rph: %s %s %s | servos: %4d %4d %4d | motor: %1d",
    state,
    rollStr, pitchStr, headingStr,
    clutchServoDC, leftWingServoDC, rightWingServoDC,
    windingMotorOn);
  Serial.println(str);
}


void loop() {
  // TODO - Replace with timer interrupt
  controlLoop();
  delay(10);
}
