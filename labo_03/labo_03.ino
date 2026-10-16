#include <MeAuriga.h>
#include <Wire.h>
#include <limits.h>
// Roynel Fonseca
// Derniers chiffres du DA: 48

//enum AppState {SETUP, STOP, SEGA, SEGB, SEGC, PIVOT, FIN};

//AppState currentState = STOP;

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6

#define LEDNUM  13 // J'ai mis 13 leds pour que la fonction "getColorAt" tienne en compte la led 12
#define LED_PIN 44
int progressiveBar[7] = {0, 1, 2, 3, 4, 5, 6};

// Déclaration du gyroscope
MeGyro gyro(0, 0x69);

// Déclaration des encodeurs
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

// Déclaration des leds
MeRGBLed led( 0, LEDNUM );

// Déclaration du ultrasonSensor
MeUltrasonicSensor ultraSensor(PORT_8);

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
  //currentState = SETUP;
  delay(5000);
}

// void setupState(unsigned long ct) {
//   static bool firstTime = true;
//   static unsigned long lastTime = 0;
//   static unsigned long exitTime = 0;

//   const int timeout = 3000;

//   if (firstTime) {
//     firstTime = false;
//     exitTime = ct + timeout;

//     Serial.println("Attente de 3 secondes avant de démarrer.");
//   }

//   // Là la là... j'attends

//   if (ct >= exitTime) {
//     firstTime = true;
//     straightCmd();
//   }
// }

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

void distanceState(){

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


void loop() {
  currentTime = millis();

  // for(int i = 1; i < 13; i++){
  //   led.setColorAt(i, 5, 0, 0);
  // }

  // led.setColorAt(0, 5, 0, 0);

  // cRGB var = {led.getColorAt(12)};

  // Serial.print("R: ");
  // Serial.print(var.r);
  // Serial.print(" G: ");
  // Serial.print(var.g);
  // Serial.print(" B: ");
  // Serial.println(var.b);

  // led.show();
  static bool f = true;

  if(f){
    f = false;
    setDistanceToMove(1000);
    progressiveBarTask();
  }else{
    distanceState();
    progressiveBarTask();
  }

  
  gyroTask(currentTime);
  encodersTask(currentTime);
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

void progressiveBarTask(){
  long actualPosition = (abs(encoderLeft.getCurPos()) + abs(encoderRight.getCurPos())) / 2.0;
  long initialPosition = (abs(initEncLeftDeg + initEncRightDeg)) / 2.0;

  float remaining = distToGoDeg - abs(initialPosition - actualPosition);

  int leds = map(remaining, initialPosition, distToGoDeg, 0, 7);

  static int previousLedValue = -1;

  // led.setColor(0, 0);
  
  if(previousLedValue != leds){
    for(int i = leds; i < (sizeof(progressiveBar) / sizeof(progressiveBar[0])); i++){
      Serial.print("Leds: ");
      Serial.print(leds);
      Serial.print(" I: ");
      Serial.println(i);
      led.setColorAt(i, 0, 25, 0);
    }

    previousLedValue = leds;

    led.show();
  }
}