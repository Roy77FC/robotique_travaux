#include "fonctions_generales.h"
#include "lab_03.h"
#include "suivre_ligne.h"

enum AppState {BOOT, ATTENTE, LAB3, CALIB, SUIVRE_LIGNE, CMD_INVALIDE, NON_CALIB};
enum Command  {CMD_NONE, CMD_STOP, CMD_LAB3, CMD_CAL, CMD_LIGNE, CMD_INCONNUE};

const unsigned long BOOT_DELAY_MS = 3000;   // durée de l'état BOOT
const unsigned long WARNING_MS    = 3000;   // durée des clignotements (CMD_INVALIDE / NON_CALIB)
const unsigned long BLINK_RATE_MS = 250;    // demi-période du clignotement
const int DIST_SEGMENT_MM = 1000;           // distance des segments A et C du labo 3 (mm)

unsigned long stateStart = 0;               // moment où l'état courant a commencé
int blinkPhase = -1;   

AppState currentState= BOOT;

Lab03 lab03;
SuivreLigne Sligne;
bool fisrtTimeEnter = true;
float zAngle;

float pos;


const char* stateName(AppState s) {
  switch (s) {
    case BOOT:         return "BOOT";
    case ATTENTE:      return "ATTENTE";
    case LAB3:         return "LAB3";
    case CALIB:        return "CALIB";
    case SUIVRE_LIGNE: return "SUIVRE_LIGNE";
    case CMD_INVALIDE: return "CMD_INVALIDE";
    case NON_CALIB:    return "NON_CALIB";
  }
  return "?";
}

void changeState(AppState next, unsigned long ct) {
  stopMotors();                  // chaque état repart avec les moteurs arrêtés
  ledClear();
  ledShow();

  currentState = next;
  stateStart = ct;
  blinkPhase = -1;

  Serial.print("État : ");
  Serial.println(stateName(next));

  // Actions d'entrée : exécutées UNE seule fois
  switch (next) {
    case LAB3:         lab03.begin(); break;
    case CALIB:        Sligne.mouvementCalibrer(ct, zAngle, true); break;
    case SUIVRE_LIGNE: Sligne.startFollow(ct); break;
    default: break;
  }
}

// ------------------------------- Bluetooth / série -------------------------------
// Lit les caractères reçus sans bloquer. Retourne une commande quand une ligne complète est reçue.
Command commandTask() {
  static String buf = "";

  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n') { buf += c; continue; }

    // Retire l'entête 0xFF55 s'il est présent (voir cours 09), puis les espaces / \r
    if (buf.length() >= 2 && (uint8_t)buf[0] == 0xFF && (uint8_t)buf[1] == 0x55) buf.remove(0, 2);
    buf.trim();

    String msg = buf;
    buf = "";
    if (msg.length() == 0) continue;

    Serial.print("cmd reçu : ");
    Serial.println(msg);

    if (msg == "stop")  return CMD_STOP;
    if (msg == "lab3")  return CMD_LAB3;
    if (msg == "cal")   return CMD_CAL;
    if (msg == "ligne") return CMD_LIGNE;

    Serial.println("cmd INCONNUE");
    return CMD_INCONNUE;
  }
  return CMD_NONE;
}

void blinkTask(unsigned long ct, int r, int g, int b) {
  int phase = ((ct - stateStart) / BLINK_RATE_MS) % 2;   // 0 = allumé, 1 = éteint
  if (phase == blinkPhase) return;
  blinkPhase = phase;

  if (phase == 0) ledSetAll(r, g, b);
  else            ledSetAll(0, 0, 0);
  ledShow();
}

void stateManagement(unsigned long ct, Command cmd) {

  // "stop" fonctionne dans tous les états (sauf BOOT) et ramène en ATTENTE
  if (cmd == CMD_STOP && currentState != BOOT) {
    if (currentState != ATTENTE) changeState(ATTENTE, ct);
    return;
  }

  switch (currentState) {

    case BOOT:
      if (ct - stateStart >= BOOT_DELAY_MS) changeState(ATTENTE, ct);
      break;

    case ATTENTE:
      if      (cmd == CMD_LAB3)     changeState(LAB3, ct);
      else if (cmd == CMD_CAL)      changeState(CALIB, ct);
      else if (cmd == CMD_LIGNE)    changeState(Sligne.is_calibrated ? SUIVRE_LIGNE : NON_CALIB, ct);
      else if (cmd == CMD_INCONNUE) changeState(CMD_INVALIDE, ct);
      break;

    case LAB3:
      if (lab03.update(DIST_SEGMENT_MM, ct, zAngle)) changeState(ATTENTE, ct);
      break;

    case CALIB:
      if (Sligne.mouvementCalibrer(ct, zAngle)) {
        Sligne.is_calibrated = true;
        changeState(ATTENTE, ct);
      }
      break;

    case SUIVRE_LIGNE:
      Sligne.followTask(ct, zAngle);
      break;

    case CMD_INVALIDE:
      blinkTask(ct, 60, 0, 0);                                   // rouge
      if (ct - stateStart >= WARNING_MS) changeState(ATTENTE, ct);
      break;

    case NON_CALIB:
      blinkTask(ct, 60, 40, 0);                                  // jaune
      if (ct - stateStart >= WARNING_MS) changeState(ATTENTE, ct);
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  encoderConfig();
  gyroBegin();
  ledSetUp();
  Sligne.setUpSuivreLigne();
  changeState(BOOT, millis());
}

bool lab03Task(unsigned long ct, int dist = 1000){
  lab03.setDistanceUltraSon(ct);
  return lab03.update(dist, ct, zAngle);
}

void loop() {

  unsigned long ct = millis();
  gyroTask(millis(), zAngle);
  encodersTask(millis()); 
  stateManagement(ct, commandTask());
}