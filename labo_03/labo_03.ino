#include <MeAuriga.h>
#include <Wire.h>
#include <limits.h>
// Roynel Fonseca
// Derniers chiffres du DA: 48

enum AppState {SETUP, SEGA, SEGB, SEGC, PIVOT, FIN};

AppState currentState = SETUP;

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6

// Déclaration du gyroscope
MeGyro gyro(0, 0x69);

// Déclaration des encodeurs
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

// Déclaration des leds
#define LEDNUM  12
#define LED_PIN 44

MeRGBLed led( 0, LEDNUM );
int completedLedsProgession = 0;
int progressiveBar[7] = {0, 1, 2, 3, 4, 5, 6};
int completedLeds[3] = {11, 10, 9};
int finLeds[5] = {11, 10, 9, 8, 7};

// Déclaration du ultrasonSensor
int distanceUltraSon = 0;
const int RATE_READ = 100;
const int TOO_NEAR_DIST = 30;
const int MAX_DIST = 400;
MeUltrasonicSensor distSensor(PORT_8);

unsigned long currentTime = 0;

unsigned long serialPrintPrevious = 0;
int serialPrintInterval = 500;
String msg = "";

int distToGoDeg = 0;
long initEncLeftDeg = 0;
long initEncRightDeg = 0;

int distSegA = 1000;
int distSegC = 1000;

const int FLASHING_FREC = 10;


// ********* INTERRUPTIONS ***********

void rightEncoderInterrupt(void){
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();
  }
}

void leftEncoderInterrupt(void){
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);

  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);

  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);

  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);

  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);

  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}


void setup() {
  Serial.begin(115200);
  encoderConfig();
  gyro.begin();

  led.setpin(LED_PIN);
  // delay(2000);
  // spinTask(195);
}


void loop() {
  currentTime = millis();

  ReadDistance(currentTime);

  stateManagement(currentTime);
  gyroTask(currentTime);
  encodersTask(currentTime);
  

  // Serial.println(currentState);
}

void goStraight(short speed = 100, short firstRun = 0) {
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  // PD Controller
  // Change les valeurs selon tes besoins
  // higher kp = plus réactive, peu osciller
  // lowewr kp = sluggish, moins d'oscillation
  // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
  const double kp = 3.5;
  //const double ki = 1.0;
  const double kd = 6.0;

  if (firstRun) {
    firstRun = 0;

    zAngleGoal = gyro.getAngleZ();
    Serial.println ("Setting speed");

    encoderLeft.setTarPWM(speed);
    encoderRight.setTarPWM(-speed);
    previousError = 0;

    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;

  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);

  previousError = error;

  msg = "z : ";
  msg += gyro.getAngleZ();
  msg += "\tleft : ";
  msg += encoderLeft.getCurPwm();
  msg += "\tright : ";
  msg += encoderRight.getCurPwm();
  msg += "\toutput :";
  msg += output;

  encoderLeft.setTarPWM(speed - output);
  encoderRight.setTarPWM(-speed - output);
}

void stop() {
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
}

void setDistanceToMove(int dist){
  distToGoDeg = dist / CIRC_WHEEL * 360;

  initEncLeftDeg = abs(encoderLeft.getCurPos());
  initEncRightDeg = abs(encoderRight.getCurPos());
  goStraight(100, 1);
}

bool distanceTask(){

  int curEncLeftDeg = abs(encoderLeft.getCurPos());
  int curEncRightDeg = abs(encoderRight.getCurPos());
  long actualPosition = (curEncLeftDeg + curEncRightDeg) / 2.0;
  long initialPosition = (initEncLeftDeg + initEncRightDeg) / 2.0;

  bool isThereADiff = curEncLeftDeg < initEncLeftDeg || curEncRightDeg < initEncRightDeg;
  int diff;

  if(curEncLeftDeg < initEncLeftDeg) diff = initEncLeftDeg - curEncLeftDeg;
  else if(curEncRightDeg < initEncRightDeg) diff = initEncRightDeg - curEncRightDeg;
  else actualPosition = (curEncLeftDeg + curEncRightDeg) / 2.0;
  

  if(isThereADiff){
    actualPosition = (curEncLeftDeg + diff + curEncRightDeg) / 2.0;
  }

  float remaining = distToGoDeg - abs(initialPosition - actualPosition);
  // Serial.print("But: ");
  // Serial.print(distToGoDeg);
  // Serial.print(" Intial: ");
  // Serial.print(initialPosition);
  // Serial.print(" Actuel: ");
  // Serial.print(actualPosition);
  // Serial.print(" Restant: ");
  // Serial.println(remaining);

  if(remaining <= 0){
    stop();
    return true;
  }

  int speed = constrain(remaining * 0.5, 40, 100);
  goStraight(speed);

  return false;
}


void gyroTask(unsigned long ct) {
  gyro.update();
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
}

// void stopCmd() {
//   Serial.println ("Stopping");
//   msg = "";

//   currentState = STOP;
// }

void spinTask(int goal) {

  float ratioSpin = (goal * 1.0) / 360;
  
  // Distance à parcourir
  float dist = ratioSpin * FULL_SPIN_CIRC;
  
  // Nombre de rotations de chaque roue
  float nbRots = dist/CIRC_WHEEL;
  
  // On multiplie par 360 pour obtenir
  // le nombre de degrés
  float angleLeft = nbRots * 360;
  float angleRight = angleLeft;
  
  encoderLeft.move(angleLeft);
  encoderRight.move(angleRight);
}

void ReadDistance(unsigned long ct){

  static unsigned long previousTime = 0;

  if(ct - previousTime >= RATE_READ){
    distanceUltraSon = distSensor.distanceCm();
    previousTime = ct;
  }
}

void progressiveBarTask(bool isAccurate){
  int leds = -1;
  int nbLeds = sizeof(progressiveBar) / sizeof(progressiveBar[0]);

  if(isAccurate){
    long actualPosition = (abs(encoderLeft.getCurPos()) + abs(encoderRight.getCurPos())) / 2.0;
    long initialPosition = (abs(initEncLeftDeg + initEncRightDeg)) / 2.0;

    float remaining = distToGoDeg - abs(initialPosition - actualPosition);
    leds = map(remaining, 0, distToGoDeg, 0, nbLeds);

    // Serial.print("ACT: ");
    // Serial.print(actualPosition);
    // Serial.print(" INIT: ");
    // Serial.print(initialPosition);
    // Serial.print(" WheelDeg: ");
    // Serial.print(distToGoDeg);
    // Serial.print(" LEDS: ");
    // Serial.print(leds);
    // Serial.print(" remainig: ");
    // Serial.println(remaining);
    
    static int previousLedValue = -1;

    if(leds < previousLedValue){
      led.setColor(0,0);
    }
    
    if(previousLedValue != leds){
      for(int i = leds; i < nbLeds; i++){
        led.setColorAt(i, 0, 0, 25);
      }
      previousLedValue = leds;
    }

  }else{

    static int previousLedValue = -1;
    leds = map(distanceUltraSon, TOO_NEAR_DIST, MAX_DIST, nbLeds, 0);

    if(leds < previousLedValue){
      led.setColor(0,0);
    }

    if(previousLedValue != leds){

      for(int i = 0; i < nbLeds; i++){
        
        if(i < leds)led.setColorAt(i, 0, 0, 25);
        else led.setColorAt(i, 0, 0, 0);
      }

      previousLedValue = leds;
    }   
  }
  led.show();
}

void completedLedsTask(){
  //led.setColor(0,0);

  if(completedLedsProgession != 0){

    for(int i = 0; i < completedLedsProgession; i++){
      led.setColorAt(completedLeds[i], 0, 35, 0);
    }
    led.show();
  }
}

// State functions ----------------------------------------------------------

void setupState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long exitTime = 0;

  const int timeout = 3000;

  if (firstTime) {
    firstTime = false;
    exitTime = ct + timeout;

    Serial.println("Attente de 3 secondes avant de démarrer.");
  }

  // Là la là... j'attends

  if (ct >= exitTime) {
    firstTime = true;
    currentState = SEGA;
  }
}

void pivotState(AppState callingState, unsigned long ct){

  static bool isSpinCalled = false;
  static bool isTimeTaked = false;
  int tolerance = 10;
  int degAdjustLeftPivot = 8;
  int degAdjustRightPivot = 12;
  static long initL;
  static long initR;
  int waitTime = 1000;  
  static unsigned long exitTime = 0;

  int degToSpin = callingState == SEGC? 188 : 95;
  float wheelDegGoal = degToSpin * FULL_SPIN_CIRC / CIRC_WHEEL;

  if(!isSpinCalled){
    callingState != SEGB? spinTask(degToSpin * -1) : spinTask(degToSpin);
    isSpinCalled = true;
    initL = encoderLeft.getCurPos();
    initR = encoderRight.getCurPos();
  }

  long moveLeft = abs(encoderLeft.getCurPos() - initL);
  long moveRight = abs(encoderRight.getCurPos() - initR);
  float moved = (moveLeft + moveRight) / 2.0;

  // if(moved >= wheelDegGoal - tolerance){
  //   isSpinCalled = false;
  //   currentState = FIN;
  //   if(callingState == SEGA) currentState = SEGB;
  //   else if(callingState == SEGB) currentState = SEGC;
  // }

  // set the time to wait for switch
  if(moved >= (wheelDegGoal - tolerance) && !isTimeTaked){
    exitTime = ct + waitTime;
    isTimeTaked = true;
  }

  if(ct >= exitTime && isTimeTaked){
    isSpinCalled = false;
    isTimeTaked = false;
    currentState = FIN;
    if(callingState == SEGA) currentState = SEGB;
    else if(callingState == SEGB) currentState = SEGC;
  }

  // Serial.print("MOV L: ");
  // Serial.print(moveLeft);
  // Serial.print(" MOV R: ");
  // Serial.print(moveRight);
  // Serial.print(" WheelDeg: ");
  // Serial.print(wheelDegGoal);
  // Serial.print(" moved: ");
  // Serial.println(moved);  
}

void segA_And_SegCState(int distToMove = 1000) {

  static bool firstCall = true;

  if(firstCall){
    firstCall = false;
    setDistanceToMove(distToMove);
  }else{
    if(distanceTask()){
      currentState = PIVOT;
      firstCall = true;
      completedLedsProgession++;
      completedLedsTask();
      Serial.println("Ça rentre");
    }
  }

  progressiveBarTask(true);
}

void segBState() {

  static bool firstCall = true;

  if(distanceUltraSon <= TOO_NEAR_DIST){

    stop();
    currentState = PIVOT;
    completedLedsProgession++;
    completedLedsTask();
    Serial.println("Ça rentre");
    firstCall = true;

  }else{

    if(firstCall){
      goStraight(100, firstCall);
      firstCall = false;
    }else{
      goStraight();
    }
  }

  progressiveBarTask(false);
}

void finState(unsigned long ct){

  static unsigned long previousTime = 0;
  int rate = (1.0 / FLASHING_FREC) * 1000;
  int ledTabSize = sizeof(finLeds) / sizeof(finLeds[0]);
  static bool isOn = false;

  if(ct - previousTime >= rate){
    if(isOn){
      led.setColor(0,0);
      isOn = false;
    }else{
      for(int i = 0; i < ledTabSize; i++){
        led.setColorAt(finLeds[i], 0, 25, 0);
      }
      isOn = true;
    }

    previousTime = ct;
  }
  

  led.show();
}

void stateManagement(unsigned long ct){

  static AppState callingState;

  switch(currentState){

    case SETUP:
      setupState(ct);
    break;

    case SEGA:
      callingState = currentState;
      segA_And_SegCState(distSegA);
      
    break;

    case SEGB:
      callingState = currentState;
      segBState();
      
    break;

    case SEGC:
      callingState = currentState;
      segA_And_SegCState(distSegC);
      
    break;

    case PIVOT:
      pivotState(callingState, ct);
    break;

    case FIN:
      finState(ct);
    break;
  }

  if(currentState != FIN){
    completedLedsTask();
  }
}
