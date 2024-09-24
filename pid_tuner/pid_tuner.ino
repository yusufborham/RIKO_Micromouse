#include "Adafruit_VL53L0X.h"

#define SDA 21
#define SCL 19

#define LED_L	13
#define LED_R	23
#define LED_M	22

// left
#define MOT1A 33
#define MOT1B 32
// right
#define MOT2A 26
#define MOT2B 25
float sampling_time = 0.05 ;
float fc = 0.01 ;
double alpha =(2*3.1415*sampling_time*fc)/(2*3.1415*sampling_time*fc+1) ;
int prvError = 0 ;
int error = 0 ;
float outputError = 0 ;
// Yi=aXi=(1-a)*Yi-1 ## calculating the output


const int LOX_ADDR[3] = { 0x30, 0x31, 0x32};
const int SHT_LOX[3] = { 18, 12, 05 };
Adafruit_VL53L0X lox[3] = { Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X() };
int lox_reading[3];
unsigned long lastMicros = 0;
int counter = 0 ;
void LOXInit() {
  Serial.println(F("Initializing LOX..."));
  for (int i = 0; i < 3; i++)
    pinMode(SHT_LOX[i], OUTPUT);
  for (int i = 0; i < 3; i++)
    digitalWrite(SHT_LOX[i], LOW);
  delay(10);
  for (int i = 0; i < 3; i++)
    digitalWrite(SHT_LOX[i], HIGH);
  delay(10);
  for (int i = 0; i < 3; i++)
    digitalWrite(SHT_LOX[i], LOW);
  for (int i = 0; i < 3; i++) {
    digitalWrite(SHT_LOX[i], HIGH);
    delay(10);
    if (!lox[i].begin(LOX_ADDR[i])) {
      Serial.print(F("Failed to boot VL53L0X #"));
      Serial.println(i);
      while (1)
        ;
    }
    delay(10);
  }
  for (int i = 0; i < 3; i++) {
    lox[i].startRangeContinuous();
  }
}

void LOXRead() {
  for (int i = 0; i < 3; i++) {
    lox_reading[i] = lox[i].readRangeResult();
    if (lox[i].readRangeStatus() == 4)
      lox_reading[i] = 8191;
  }
}

bool LOXReady() {
  for (int i = 0; i < 3; i++) {
    if (!lox[i].isRangeComplete())
      return false;
  }
  return true;
}

void setup() {
  pinMode(MOT1A, OUTPUT);
  pinMode(MOT1B, OUTPUT);
  pinMode(MOT2A, OUTPUT);
  pinMode(MOT2B, OUTPUT);

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_M, OUTPUT);

  digitalWrite(MOT1A, LOW);
  digitalWrite(MOT1B, LOW);
  digitalWrite(MOT2A, LOW);
  digitalWrite(MOT2B, LOW);
  

  digitalWrite(LED_L, HIGH);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_M, HIGH);

  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  while (!Serial) {
    delay(1);
  }

  delay(1000);

  LOXInit();

  
  delay(5000);
  digitalWrite(MOT1A, LOW);
  analogWrite(MOT1B, 255);
  digitalWrite(MOT2A, LOW);
  analogWrite(MOT2B, 255);

  lastMicros = micros();
  prvError = 0 ;
}

void loop() {
  if (!LOXReady())
    return;
  LOXRead();
  // Serial.print(millis() - lastMicros);
  // Serial.print("    reading number    ");
   //Serial.print(counter);
  // Serial.print(" time is  ") ;
  // lastMicros = millis();
  error = lox_reading[1]-lox_reading[2] ;
  // Yi=aXi+(1-a)*Yi-1 ## calculating the output


  Serial.printf(" the error  %d \n ", error );


  //counter++ ;
}