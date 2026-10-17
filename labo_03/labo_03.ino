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
#define LEDNUM  13 // J'ai mis 13 leds pour que la fonction "getColorAt" tienne en compte la led 12
#define LED_PIN 44

MeRGBLed led( 0, LEDNUM );
int completedLedsProgession = -1;
int progressiveBar[7] = {0, 1, 2, 3, 4, 5, 6};
int completedLeds[3] = {11, 10, 9};

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
  delay(2000);
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
  const double kp = 3.0;
  //const double ki = 1.0;
  const double kd = 1.5;

  if (firstRun) {
    firstRun = 0;

    zAngleGoal = gyro.getAngleZ();
    Serial.println ("Setting speed");

    encoderLeft.setTarPWM(speed);
    encoderRight.setTarPWM(-speed);

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

  initEncLeftDeg = encoderLeft.getCurPos();
  initEncRightDeg = encoderRight.getCurPos();
  goStraight(100, 1);
}

void distanceTask(){

  long actualPosition = (abs(encoderLeft.getCurPos()) + abs(encoderRight.getCurPos())) / 2.0;
  long initialPosition = (abs(initEncLeftDeg + initEncRightDeg)) / 2.0;

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
    return;
  }

  int speed = constrain(remaining * 0.5, 40, 100);
  goStraight(speed);
}

void stateManagement(unsigned long ct){

  static AppState callingState;

  switch(currentState){

    case SETUP:
      setupState(ct);
    break;

    case SEGA:
      callingState = currentState;
      segA_And_SegCState();
      
    break;

    case SEGB:
      callingState = currentState;
      segA_And_SegCState();
      
    break;

    case SEGC:
      callingState = currentState;
      segBState();
      
    break;

    case PIVOT:
      pivotState(callingState);
    break;

    case FIN:
      finState();
    break;
  }
}

void loop() {
  currentTime = millis();

  ReadDistance(currentTime);

  stateManagement(currentTime);
  gyroTask(currentTime);
  encodersTask(currentTime);

  // Serial.println(currentState);
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
    leds = map(remaining, initialPosition, distToGoDeg, 0, nbLeds);
    static int previousLedValue = -1;
    
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

  led.setColor(0,0);
  if(completedLedsProgession != -1){
    for(int i = 0; i < completedLedsProgession; i++){
      led.setColorAt(completedLeds[completedLedsProgession], 0, 35, 0);
    }
    led.show();
  }
}

// State functions ----------------------------------------------------------

void setupState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
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

void pivotState(AppState callingState){

  static bool isSpinCalled = false;
  
  static long initEncLeftPos;
  static long initEncRightPos;
  long currEncLeftPos = abs(encoderLeft.getCurPos());
  long currEncRightPos = abs(encoderRight.getCurPos());

  int degToSpin = callingState == SEGC? 183 : 93;
  AppState stateToSwtich;

  if(!isSpinCalled){
    callingState == SEGB? spinTask(degToSpin * -1) : spinTask(degToSpin);
    isSpinCalled = true;
    initEncLeftPos = abs(encoderLeft.getCurPos());
    initEncRightPos = abs(encoderRight.getCurPos());
  }

  long currentPos = -1;
  long initialPos = -1;

  if(currEncLeftPos < initEncLeftPos){

    int diff = initEncLeftPos - currEncLeftPos;

    currentPos = (currEncLeftPos + diff + currEncRightPos) / 2.0;
    initialPos = (initEncLeftPos + initEncRightPos) / 2.0;

  }else if(currEncRightPos < initEncRightPos){

    int diff = initEncRightPos - currEncRightPos;

    currentPos = (currEncRightPos + diff + currEncLeftPos) / 2.0;
    initialPos = (initEncLeftPos + initEncRightPos) / 2.0;
  }

  switch(callingState){
    case SEGA:      
      stateToSwtich = SEGB;
    break;

    case SEGB:
      stateToSwtich = SEGC;
    break;
      
    case SEGC:
      stateToSwtich = FIN;
    break;
  }

  Serial.print("P Ini: ");
  Serial.print(initialPos);
  Serial.print(" P Act: ");
  Serial.print(currentPos);
  Serial.print(" Diff: ");
  Serial.println(currentPos - initialPos);

  // Serial.println(stateToSwtich);

  if(currentPos - initialPos >= degToSpin && currentPos!= -1){
    isSpinCalled = false;
    currentState = stateToSwtich;
  }
}

void segA_And_SegCState() {

  static bool firstCall = true;

  if(encoderLeft.getCurPwm() == 0 && !firstCall){
    currentState = PIVOT;
    firstCall = true;
    completedLedsProgession++;
    completedLedsTask();
  }

  if(firstCall){
    firstCall = false;
    setDistanceToMove(1000);
  }else{
    distanceTask();
  }

  progressiveBarTask(true);
}

void segBState() {

  if(distanceUltraSon <= TOO_NEAR_DIST){

    stop();
    currentState = PIVOT;
    completedLedsProgession++;
    completedLedsTask();

  }else{

    goStraight();
  }

  progressiveBarTask(false);
}

void finState(){

  for(int i = 0; i < LEDNUM -1; i++){
    led.setColorAt(i, 25, 0, 0);
  }}