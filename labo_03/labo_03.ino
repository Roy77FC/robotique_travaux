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

int distToGoDeg = 0;
long initEncLeftDeg = 0;
long initEncRightDeg = 0;

int distanceFoward = 1500;

const int FLASHING_FREC = 10;
const float ANGLE_TOLERANCE_DEG = 3.0;
const int MIN_PWM = 30;        
const int PIVOT_SPEED = 45;   

// Angle Z cumulatif (ne revient pas à -180 après 180)
double zAngle = 0.0;

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
}

void goStraight(short speed = 100, short firstRun = 0) {

  static unsigned long previousTime = 0;
  const short RATE = 20;

  static double zAngleGoal = 0.0;
  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  // PD Controller
  // Change les valeurs selon tes besoins
  // higher kp = plus réactive, peu osciller
  // lowewr kp = sluggish, moins d'oscillation
  // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
  const double kp = 4.0;
  //const double ki = 1.0;
  const double kd = 10.0;

  unsigned long ct = millis();

  if (firstRun) {

    zAngleGoal = zAngle;
    Serial.println ("Setting speed");

    encoderLeft.setTarPWM(speed);
    encoderRight.setTarPWM(-speed);
    previousError = 0;

    return;
  }

  //if(ct - previousTime >= RATE) return;

  previousTime = ct;

  error = zAngle - zAngleGoal;

  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);

  previousError = error;

  // Serial.print("error: ");
  // Serial.print(error);
  // Serial.print(" vitesse: ");
  // Serial.print(speed - output);
  // Serial.print(" Goal: ");
  // Serial.print(zAngleGoal);
  // Serial.print(" zAngle");
  // Serial.println(zAngle);

  encoderLeft.setTarPWM(speed - output);
  encoderRight.setTarPWM(-speed - output);
}

void loop() {
  currentTime = millis();

  ReadDistance(currentTime);
  static bool f = true;
  stateManagement(currentTime);
  gyroTask(currentTime);
  encodersTask(currentTime);
}


void stopMotors() {
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
  encoderLeft.setMotorPwm(0);   
  encoderRight.setMotorPwm(0);
}

void setDistanceToMove(int dist){
  distToGoDeg = dist / CIRC_WHEEL * 360;

  initEncLeftDeg = abs(encoderLeft.getCurPos());
  initEncRightDeg = abs(encoderRight.getCurPos());
  goStraight(100, 1);
}

int limitSpeed(float value, int maxSpeed) {

  value = constrain(value, -maxSpeed, maxSpeed);

  if (fabs(value) < MIN_PWM) {
    value = value < 0 ? -MIN_PWM : MIN_PWM;
  }
  return (int)value;
}

void ReadDistance(unsigned long ct){

  static unsigned long previousTime = 0;

  if(ct - previousTime >= RATE_READ){
    distanceUltraSon = distSensor.distanceCm();
    previousTime = ct;
  }
}

// ---------------------------------------------------------- Tasks functions ----------------------------------------------------------
void gyroTask(unsigned long ct) {
  static double previousZ = 0.0;
  static bool firstTime = true;

  gyro.update();
  double z = gyro.getAngleZ();

  if (firstTime) {
    firstTime = false;
    previousZ = z;
    return;
  }

  double delta = z - previousZ;
  if (delta > 180) delta -= 360;
  else if (delta < -180) delta += 360;

  zAngle += delta;
  previousZ = z;
}

void encodersTask(unsigned long ct) {
  encoderRight.loop();
  encoderLeft.loop();
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
    stopMotors();
    return true;
  }

  int speed = constrain(remaining * 0.5, 40, 100);
  goStraight(speed);

  return false;
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

bool pivotTask(unsigned long ct, float angleDeg, int speed, bool init = false) {
  static unsigned long previousTime = 0;
  const unsigned long rate = 20;

  static double zAngleGoal = 0.0;
  static double previousError = 0.0;

  const double kp = 2.0;
  const double kd = 10.0;

  if (init) {
    previousTime = ct;
    zAngleGoal = zAngle + angleDeg;   // cible = angle actuel + rotation voulue
    previousError = angleDeg;
    return false;
  }

  if (ct - previousTime < rate) return false;
  previousTime = ct;

  double error = zAngleGoal - zAngle;

  if (fabs(error) <= ANGLE_TOLERANCE_DEG) {
    stopMotors();
    return true;
  }

  double output = kp * error + kd * (error - previousError);
  previousError = error;

  int pivotSpeed = limitSpeed(output, speed);

  // Serial.print("error: ");
  // Serial.print(error);
  // Serial.print(" vitesse: ");
  // Serial.print(pivotSpeed);
  // Serial.print(" zAngle");
  // Serial.println(zAngle);

  // Même signe sur les deux roues = rotation sur place
  encoderLeft.setTarPWM(pivotSpeed);
  encoderRight.setTarPWM(pivotSpeed);

  return false;
}

// ---------------------------------------------------------- State functions ----------------------------------------------------------

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

    stopMotors();
    currentState = PIVOT;
    completedLedsProgession++;
    completedLedsTask();
    firstCall = true;

  }else{

    if(firstCall){
      goStraight(100, firstCall);
      firstCall = false;
      led.setColor(0,0); //----------------------------------------------------------------------------------
    }else{
      goStraight();
    }
  }

  progressiveBarTask(false);
}

void pivotState(AppState callingState, unsigned long ct) {
  static bool firstTime = true;
  static bool isDone = false;
  static unsigned long exitTime = 0;
  const unsigned long WAIT_TIME = 500;

  // Angles exacts : même convention de signe que ton spinTask actuel
  float angle;
  if (callingState == SEGA)      angle = -90.0;
  else if (callingState == SEGB) angle = 90.0;
  else                           angle = -180.0;   // SEGC

  if (firstTime) {
    firstTime = false;
    isDone = false;
    pivotTask(ct, angle, PIVOT_SPEED, true);   // initialise la cible
    return;
  }

  if (!isDone) {
    isDone = pivotTask(ct, angle, PIVOT_SPEED);
    if (isDone) exitTime = ct + WAIT_TIME;
    return;
  }

  if (ct >= exitTime) {
    firstTime = true;
    currentState = FIN;
    if (callingState == SEGA) currentState = SEGB;
    else if (callingState == SEGB) currentState = SEGC;
  }
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
      segA_And_SegCState(distanceFoward);
      
    break;

    case SEGB:
      callingState = currentState;
      segBState();
      
    break;

    case SEGC:
      callingState = currentState;
      segA_And_SegCState(distanceFoward);
      
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