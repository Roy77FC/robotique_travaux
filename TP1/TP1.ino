#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <HCSR04.h>

#define TRIG_PIN 12
#define ECHO_PIN 11

HCSR04 hc(TRIG_PIN, ECHO_PIN);
const int MIN_READ_DIST = 10;
const int MAX_READ_DIST = 50;
const int CAR_HEIGHT = 46;
const int CAR_Y = 30;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Erreur : écran non détecté");
    while (true); // bloque si l'écran n'est pas trouvé
  }

  display.setTextSize(1);       // taille du texte
  display.setTextColor(SSD1306_WHITE);
}

void loop() {

  unsigned long ct = millis();
  display.clearDisplay();
  drawSun();
  drawName();
  animateCar(ct);
  display.display();

}

int sonar(unsigned long ct){

  static unsigned long lastTimeRead = 0;
  static unsigned long lastTimePrint = 0;
  const int READ_RATE = 100;
  const int PRINT_RATE = 250;
  static int lastVal = -1;
  int distance = 0;

  if(ct - lastTimeRead >= READ_RATE){
    lastTimeRead = ct;
    distance = hc.dist();
  }

  if(distance < MIN_READ_DIST || distance >= MAX_READ_DIST){
    distance = lastVal;
  }else{
    lastVal = distance;
  }

  if(ct - lastTimePrint >= PRINT_RATE){
    lastTimePrint = ct;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println("cm");
  }

  return distance;
}

void drawSun(){
  display.fillCircle(5, 5, 15, SSD1306_WHITE); 
}

void drawCar(int x, int y) {
  // Carrosserie (rectangle principal)
  display.fillRect(x, y + 8, CAR_HEIGHT, 9, SSD1306_WHITE);

  // Toit / habitacle (trapèze via un rectangle + triangles)
  display.fillRect(x + 11, y, 19, 8, SSD1306_WHITE);
  display.fillTriangle(x + 11, y + 8, x + 11, y, x + 3, y + 8, SSD1306_WHITE);  // pente avant
  display.fillTriangle(x + 30, y + 8, x + 30, y, x + 38, y + 8, SSD1306_WHITE); // pente arrière

  // Roues
  display.fillCircle(x + 9, y + 17, 3, SSD1306_WHITE);
  display.fillCircle(x + 37, y + 17, 3, SSD1306_WHITE);

  // Centre des roues (effet "jante")
  display.fillCircle(x + 9, y + 17, 1, SSD1306_BLACK);
  display.fillCircle(x + 37, y + 17, 1, SSD1306_BLACK);
}

void animateCar(unsigned long ct){

  static unsigned long previousTime = 0;
  const int REFRESH_RATE = 100;
  int distance = sonar(ct);

  int carX; 


  if(!(distance < MIN_READ_DIST || distance >= MAX_READ_DIST)){
    carX = map(distance, MIN_READ_DIST, MAX_READ_DIST, SCREEN_WIDTH - CAR_HEIGHT, 0);
    drawCar(carX, CAR_Y);
  }else{
    drawCar(SCREEN_WIDTH - CAR_HEIGHT, CAR_Y);
    Serial.println("ça rentre");
  }
}

void drawName(){
  display.setCursor(0, 50);      // position du texte
  display.println("Fonseca Cabarcas");
}