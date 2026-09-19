#include <MeAuriga.h>
#include <Wire.h>

//Motor Left
const int M1_PWM = 11;
const int M1_IN1 = 48; // M1 ENA
const int M1_IN2 = 49; // M1 ENB

//Motor Right
const int M2_PWM = 10;
const int M2_IN1 = 47; // M2 ENA
const int M2_IN2 = 46; // M2 ENB

const int MAX_PWM = 255;
const int WALK_PWM = MAX_PWM * 75 / 100.0;
const int SLOW_PWM = MAX_PWM * 50 / 100.0;
const int TURN_PWM = 700;

// LEDS DEFINITION
#define LEDNUM  12 
#define LED_PIN 44
MeRGBLed led( 0, LEDNUM );

// DISTANCE SENSOR

MeUltrasonicSensor distSensor(PORT_8);

// STATE ENUM

enum State {WALK, SLOW, STOP, BACKWARD, TURN};

State state = WALK;

const int INITIAL_WALK_DIST = 101;
int distance = INITIAL_WALK_DIST;
bool engineFlag = true;
const int RATE_DISPLAY = 250;
const int RATE_READ = 100;  
const int NORMAL_DIST = 100;
const int TOO_NEAR_DIST = 30;

const int NUMBER_LEDS_TO_ACTIVATE = 7;
int frontLeds[NUMBER_LEDS_TO_ACTIVATE] = {11, 0, 1, 2, 3, 4, 5};
int backLeds[NUMBER_LEDS_TO_ACTIVATE] = {11, 10, 9, 8, 7, 6, 5};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(M1_PWM, OUTPUT);  //We have to set PWM pin as output
  pinMode(M1_IN2, OUTPUT);  //Logic pins are also set as output
  pinMode(M1_IN1, OUTPUT);

  pinMode(M2_PWM, OUTPUT);  //We have to set PWM pin as output
  pinMode(M2_IN2, OUTPUT);  //Logic pins are also set as output
  pinMode(M2_IN1, OUTPUT);

  led.setpin( LED_PIN );
}

void ReadDistance(int ct){

  static unsigned long previousTime = 0;

  if(ct - previousTime >= RATE_READ){
    distance = distSensor.distanceCm();
    previousTime = ct;
  }
}

void DisplayDistance(int ct){

  static unsigned long previousTime = 0;

  if(ct - previousTime >= RATE_DISPLAY){
    Serial.print("DISTANCE: ");
    Serial.println(distance);
    previousTime = ct;
  }

}

void Move(const int speed, const State st){

  int in2Val = st == BACKWARD? HIGH : LOW ;
  int in1Val = !in2Val;
  // ENGINE

  digitalWrite(M1_IN2, in2Val);
  digitalWrite(M1_IN1, in1Val);
  analogWrite(M1_PWM, speed);  

  digitalWrite(M2_IN2, in2Val);
  digitalWrite(M2_IN1, in1Val);
  analogWrite(M2_PWM, speed);  

}

void turn(const int speed){

  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_PWM, speed);  

  digitalWrite(M2_IN2, LOW);
  digitalWrite(M2_IN1, HIGH);
  analogWrite(M2_PWM, speed);  
}

void Stop() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
}

void FrontLeds(){
  

  for(int i = 0; i < NUMBER_LEDS_TO_ACTIVATE; i++){

    led.setColorAt(frontLeds[i] , 0, 25, 0);
  }
    
  led.show();
}

void BackLeds(){

  for(int i = 0; i < NUMBER_LEDS_TO_ACTIVATE; i++){

    led.setColorAt(backLeds[i] , 25, 25, 0);
    

  }

  led.show();
}

void AllLeds(){

  led.setColor(0 , 25, 0 , 0);
  led.show();
}

void ClearLeds(){

  led.setColor(0 , 0, 0 , 0);
  led.show();
}



void stateManagement(unsigned long ct){

  const int RATE_STOP = 2000;
  const int RATE_BACKWARD = 1000;
  static unsigned long previousTimeStop = 0;
  static unsigned long previousTimeBack = 0;
  static unsigned long previousTimeTurn = 0;

  switch(state){

    case WALK:

      if (distance >= NORMAL_DIST){

        if(engineFlag){

          Move(WALK_PWM, state);
          engineFlag = false;
        }
        
        FrontLeds();
        
      }else{

        ClearLeds();
        
        state = SLOW;
        engineFlag = true;
      }
      break;

    case SLOW:

      BackLeds();
      

      if(engineFlag){
        Move(SLOW_PWM, state);
        engineFlag = false;
      }
      
      if (distance >= NORMAL_DIST){

        ClearLeds();
        state = WALK;
        engineFlag = true;
      }
      else if(distance < TOO_NEAR_DIST ) {

        ClearLeds();
        state = STOP;
        engineFlag = true;
        previousTimeStop = ct;
      }
      break;

    case STOP:

      AllLeds();
      Stop();
      if(ct - previousTimeStop >= RATE_STOP){
        previousTimeStop = ct;
        state = BACKWARD;
        previousTimeBack = ct;
      }
      
      break;

    case BACKWARD: 

      if(ct - previousTimeBack >= RATE_BACKWARD){

        previousTimeBack = ct;
        state = TURN;
        engineFlag = true;
        previousTimeTurn = ct;

      }else {

        if(engineFlag){
          Move(WALK_PWM, state);
          engineFlag = false;
        }
      }

      break;

    case TURN:

      if(ct - previousTimeTurn >= TURN_PWM){

        ClearLeds();
        previousTimeTurn = ct;
        state = WALK;
        engineFlag = true;

      }else{

        if(engineFlag){
          turn(SLOW_PWM);
          engineFlag = false;
        }
      }

      break;
  }
}

void loop() {

  unsigned long ct = millis();

  ReadDistance(ct);
  DisplayDistance(ct);
  stateManagement(ct);

}

