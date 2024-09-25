#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <PID_v1.h>


#define GYRO_CONFIG 0x1B

#define WIDTH 180
#define DURATION 1000
#define TURN 90

#define BTN1 36
#define BTN2 39

#define ENC1A 34
#define ENC1B 35

#define ENC2A 17
#define ENC2B 16
// left
#define MOT1A 33
#define MOT1B 32
// right
#define MOT2A 26
#define MOT2B 25

#define XSH_B 27
#define XSH_L 12
#define XSH_F 18
#define XSH_R 5

#define SCKL 14
#define SDIO 3

#define LED_L 13
#define LED_R 23
#define LED_M 22
#define LED_B 2

#define VBAT 1
#define SDA 21
#define SCL 19

#define forward 0
#define left 1
#define right 2

int state = 0;
// initialzing LEDS // middle // left // right // back

byte leds[3] = { LED_M, LED_L, LED_R };
// initializing VLX

const int LOX_ADDR[3] = { 0x30, 0x31, 0x32 };
const int SHT_LOX[3] = { 18, 12, 05 };

Adafruit_VL53L0X lox[3] = { Adafruit_VL53L0X(), Adafruit_VL53L0X(), Adafruit_VL53L0X() };
int loxReading[3];

// wall flags
// if true means there is a WALL
// wall array // forward / LEFT / RIGHT
bool walls[3] = { 0, 0, 0 };
// PID Constatnts
const double motKp = 1;
const double motKi = 0.09321;
const double motKd = 1.19;   //0.8068;

double motSetpoint = 0.0;
double motInput = 0.0;
double motOutput = 0.0;
byte pid_case = 0;
PID motPID(&motInput, &motOutput, &motSetpoint, motKp, motKi, motKd, DIRECT);

long lastMicros = 0;


// IMU initialzing

float threshold = 0.1;
float angle_threshold = 80;
float rotating_speed = 100;

byte ledState = 3 ;



/////// encoders ////////

#define ticks 0.055
//200 * tick = cir 11.14

float  distance_turn_threshold = 3000;

long counts_left_pinA = 0;
long counts_left_pinB = 0;

long counts_right_pinA = 0;
long counts_right_pinB = 0;

double distance_right = 0;
double distance_left = 0;
double distance_avg = 0;

bool flag_right_encoder = 0;
bool flag_left_encoder = 0;

float coordinates[2] = { 0, 0 };
float cell_location[2] = { 0, 0 };

byte direction = 0;  // 0 -> north , 1-> east , 2-> south , 3-> west
//////////////////////////////////////////////////////////////////////////



void MOTInit() {
  motPID.SetMode(AUTOMATIC);
  motPID.SetOutputLimits(-255, 255);
  motPID.SetSampleTime(50);

  pinMode(MOT1A, OUTPUT);
  pinMode(MOT1B, OUTPUT);
  pinMode(MOT2A, OUTPUT);
  pinMode(MOT2B, OUTPUT);
}

void MOTForward(long durationMillis) {
  static int flag = 4 ;
  long startMillis = millis();
  while (millis() - startMillis < durationMillis) {
    LOXRead();
    measureDistance();

    static double leftSensor = 0, rightSensor = 0;
    leftSensor = loxReading[1];
    rightSensor = loxReading[2];

    if (leftSensor + rightSensor <= WIDTH) {  // < 170
      motInput = leftSensor - rightSensor;
      motPID.Compute();
      pid_case = 1;
      flag = 4 ;
    }

    else if (leftSensor + rightSensor > WIDTH && leftSensor + rightSensor < 2 * WIDTH ) {  // < 340
      (rightSensor > leftSensor) ? (motInput = leftSensor - (rightSensor - WIDTH)) : (motInput = (leftSensor - WIDTH) - rightSensor);
      motPID.Compute();
      pid_case = 2;
      flag = 4 ;
    }

    else if (leftSensor + rightSensor > 2 * WIDTH && leftSensor + rightSensor < 3 * WIDTH && !(lox[1].readRangeStatus() == 2) && !(lox[2].readRangeStatus() == 2) ) {  // < 510
      (rightSensor > leftSensor) ? (motInput = leftSensor - (rightSensor - 2 * WIDTH)) : (motInput = (leftSensor - 2 * WIDTH) - rightSensor);
      //motOutput = 0;
      motPID.Compute();
      pid_case = 3;
      flag = 4 ;
      
    }
    else {
      if (flag ==4 ){
        motOutput=-motOutput;
      }
      
      else if (flag == 0)
      motOutput=0;

      pid_case = 0;
      flag--;
    }



    // else if (leftSensor + rightSensor > 3*WIDTH  &&  leftSensor + rightSensor < 4*WIDTH  ){ // < 580
    //   (rightSensor > leftSensor) ? (motInput = leftSensor - ( rightSensor - 3*WIDTH )) : (motInput = (leftSensor - 3*WIDTH) - rightSensor  ) ;
    //   pid_case = 4 ;
    //   motPID.Compute();

    // }

    // else if (leftSensor + rightSensor > 4*WIDTH  &&  leftSensor + rightSensor < 5*WIDTH  ){ // <
    //   (rightSensor > leftSensor) ? (motInput = leftSensor - ( rightSensor - 4*WIDTH )) : (motInput = (leftSensor - 4*WIDTH) - rightSensor  ) ;
    //   pid_case = 5 ;
    //   motPID.Compute();

    // }




    // right motor
    analogWrite(MOT1A, 0);
    //analogWrite(MOT1B, constrain(255 + motOutput, 0, 255));
    analogWrite(MOT1B, 127);

    // left motor
    analogWrite(MOT2A, 0);
    //analogWrite(MOT2B, constrain(255 - motOutput, 0, 255));
    analogWrite(MOT2B, 127);


    Serial.printf("F: %d, L: %d, R: %d, B: %d, motOutput: %f  and my case is %d  and my sum is  %d ", loxReading[0], loxReading[1], loxReading[2], motOutput, pid_case, loxReading[1] + loxReading[2]);
    //Serial.printf(" distance_right  is   %d  distance_left   %d and distance_avg is   %d   \n", distance_right, distance_left, distance_avg);
    Serial.printf("right %d  left %d \n ",counts_right_pinA ,counts_left_pinA);
  }
}


void moveforward(){
  resetDistance();
  do {
    MOTForward(1);
    measureDistance();
  
    // switch (direction){
    //   case 0:
    //   Ycell++;
    //   break;

    //   case 1:
    //   Xcell++;
    //   break;

    //   case 3:
    //   Ycell--;
    //   break;

    //   case 4:
    //   Xcell--;
    //   break;
    // }
   // Serial.print("Xcell=%ld & Ycell =%ld",Xcell,Ycell);
  }while(distance_avg>=18);
}

void MOTBrake() {
  analogWrite(MOT1A, 255);
  analogWrite(MOT1B, 255);
  analogWrite(MOT2A, 255);
  analogWrite(MOT2B, 255);
  delay(400);
}



void Turn2(bool isLeft) {
  resetDistance();

  digitalWrite(MOT1A, LOW);
  digitalWrite(MOT1B, LOW);
  digitalWrite(MOT2A, LOW);
  digitalWrite(MOT2B, LOW);

  analogWrite(MOT1A, isLeft ? 0 : rotating_speed);
  analogWrite(MOT1B, isLeft ? rotating_speed : 0);
  analogWrite(MOT2A, isLeft ? rotating_speed : 0);
  analogWrite(MOT2B, isLeft ? 0 : rotating_speed);

  do {
    LOXRead();
    measureDistance();
    Serial.printf("right distance is %f and the left distance is %f and the averafge distance is %f \n", distance_right, distance_left, distance_avg);
  } while (isLeft ? (distance_avg < distance_turn_threshold) : (distance_avg < distance_turn_threshold));
  Serial.println("finished rotation ");
  isLeft ? (direction ? direction = 3 : direction -= 1) : direction = (direction + 1) % 4;
  Serial.println(direction);
  MOTBrake();
  delay(200);
}

void turn(bool isLeft) {
      MOTBrake();
      int target_angle = 90;
      int current_angle = 0;
      int error = target_angle - current_angle;
      int control_signal = 0;
      int Kp = 2;

      while (abs(error) > 5) { 
          // update the angle
          error = target_angle - current_angle;
          control_signal = Kp * error;
          analogWrite(MOT1A, !isLeft ? 0 : control_signal);
          analogWrite(MOT1B, !isLeft ? control_signal : 0);
          analogWrite(MOT2A, !isLeft ? control_signal : 0);
          analogWrite(MOT2B, !isLeft ? 0 : control_signal);

      }
      MOTBrake();
}
void
 MOTTurn4(bool isLeft) {
  LOXRead();
  MOTBrake();
  // digitalWrite(MOT1A, LOW);
  // digitalWrite(MOT1B, LOW);
  // digitalWrite(MOT2A, LOW);
  // digitalWrite(MOT2B, LOW);

  analogWrite(MOT1A, !isLeft ? 0 : rotating_speed);
  analogWrite(MOT1B, !isLeft ? rotating_speed : 0);
  analogWrite(MOT2A, !isLeft ? rotating_speed : 0);
  analogWrite(MOT2B, !isLeft ? 0 : rotating_speed);

  delay(380);
  MOTBrake();
}

void LOXDisable() {
  for (int i = 0; i < 3; i++)
    digitalWrite(SHT_LOX[i], LOW);
}

void LOXInit() {
  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_M, OUTPUT);

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
    if (!lox[i].begin(LOX_ADDR[i], false, &Wire1)) {
      Serial.print(F("Failed to boot VL53L0X #"));
      Serial.println(i);
      while (1)
        ;
    }
    delay(10);
  }

  for (int i = 0; i < 3; i++)
    lox[i].startRangeContinuous();
}

bool LOXRead() {
  for (int i = 0; i < 3; i++) {
    // read readings for sensor
    if (!lox[i].isRangeComplete())
      return false;
  }
  // Serial.print("Status: ");
  for (int i = 0; i < 3; i++) {
    loxReading[i] = lox[i].readRangeResult();

    if (lox[i].readRangeStatus() == 4)  // || lox[i].readRangeStatus() == 2
      loxReading[i] = 8191;

    digitalWrite(leds[i], loxReading[i] < (WIDTH / 1) && !(lox[i].readRangeStatus() == 2));
    walls[i] = loxReading[i] < (WIDTH / 1) && !(lox[i].readRangeStatus() == 2);
    
    // Serial.print(lox[i].readRangeStatus());
    //Serial.print(", ");
  }
  // Serial.println();
  return true;
}

// bool LOXReady() {
//   for (int i = 0; i < 3; i++) {
//     if (!lox[i].isRangeComplete())
//       return false;
//   }
//   return true;
// }
void IRAM_ATTR rightEncoderPinAHandle() {
  // digitalRead(ENC1B) ? s_left++ : counts_left-- ;
  if (!flag_right_encoder) {
    counts_right_pinB++;
  }
  counts_right_pinA++;
  flag_right_encoder = 0;
}

void IRAM_ATTR rightEncoderPinBHandle() {
  //digitalRead(ENC2B) ? counts_right++ : counts_right-- ;
  if (flag_right_encoder) {
    counts_right_pinA++;
  }
  counts_right_pinB++;
  flag_right_encoder = 1;
}

void IRAM_ATTR leftEncoderPinAHandle() {
  // digitalRead(ENC1B) ? counts_left++ : counts_left-- ;
  if (!flag_left_encoder) {
    counts_left_pinB++;
  }
  counts_left_pinA++;
  flag_left_encoder = 0;
}

void IRAM_ATTR leftEncoderPinBHandle() {
  //digitalRead(ENC2B) ? counts_right++ : counts_right-- ;
  if (flag_left_encoder) {
    counts_left_pinA++;
  }
  counts_left_pinB++;
  flag_left_encoder = 1;
}
void measureDistance() {

  distance_left = counts_left_pinA * ticks;
  distance_right = counts_right_pinA * ticks;
  distance_avg = (distance_left + distance_right) / 2;
}

void resetDistance() {

  counts_left_pinA = 0;
  counts_left_pinB = 0;
  counts_right_pinA = 0;
  counts_left_pinB = 0;
  distance_right = 0;
  distance_left = 0;
  distance_avg = 0;
}
void encoderInit() {
  pinMode(ENC1A, INPUT_PULLUP);
  pinMode(ENC1B, INPUT_PULLUP);
  pinMode(ENC2A, INPUT_PULLUP);
  pinMode(ENC2B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC2B), rightEncoderPinAHandle, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), rightEncoderPinBHandle, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC1B), leftEncoderPinAHandle, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC1A), leftEncoderPinBHandle, FALLING);
}

// void leftWallfollowing() {
//   LOXRead();

//   if (loxReading[forward] > WIDTH / 2) {
//     MOTForward(1);
//   } else if ((loxReading[forward] < WIDTH / 2) && !walls[left] && !(lox[forward].readRangeStatus() == 2)) {
//     Turn2(true);
//   }

//   else if (loxReading[forward] < WIDTH / 2 && walls[left]) {
//     Turn2(false);
//   }
// }

// void leftWallfollowing2() {
//   LOXRead();

//   if (loxReading[forward] > WIDTH / 2) {
//     MOTForward(1);
//   } else if ((loxReading[forward] < WIDTH / 2) && !walls[left] && !(lox[forward].readRangeStatus() == 2)) {
//     MOTTurn4(true);
//   }

//   else if (loxReading[forward] < WIDTH / 2 && walls[left]) {
//     MOTTurn4(false);
//   }
// }

// void wallfollowing() {
//   LOXRead();
//   if (!walls[left]) {

//     MOTForward(220);
//     MOTTurn4(true);
//     MOTForward(280);
//   }

// else if ((loxReading[forward] < WIDTH * 2/3)) {
//   MOTTurn4(false);
// }

// else {
//   MOTForward(1);
// }
//}

// void blink(){

//     if (millis() > last_ms + 1000){
//       ledState--; 
//     digitalWrite(LED_L,(ledState==2));
//     digitalWrite(LED_R,(ledState==1));
//     digitalWrite(LED_M,(ledState==0));

//     if (!ledState)  ledState=3;

//     last_ms = millis();
//     }
    

//}

void setup() {
  Serial.begin(115200);
  Wire1.begin(SDA, SCL);
 // Wire.begin(SDIO,SCKL);


  while (!Serial) {
    delay(1);
  }

  delay(1000);

  Serial.println(" initialzing vlx sensors ");
  LOXInit();
  Serial.println(" initialzing motors  ");
  MOTInit();
  
  encoderInit();
  lastMicros = micros();

  pinMode(36, INPUT);
  pinMode(39, INPUT);

  pinMode(LED_L,OUTPUT);
  pinMode(LED_R,OUTPUT);
  pinMode(LED_M,OUTPUT);


  while (digitalRead(36) && digitalRead(39)) 
  ;
  //{

  //   if (digitalRead(36) == LOW) {
  //     state = 1;
  //     Serial.println("wall following");
  //   }

  //   else if (digitalRead(39) == LOW) {
  //     state = 2;
  //     Serial.println("borham following");
  //   }
  // }
  
  // MOTTurn2(true) ;
}




void loop() {
  // Serial.printf("[%d] ", micros() - lastMicros);
  // lastMicros = micros();
  //Serial.printf("F: %d, L: %d, R: %d, B: %d\n", loxReading[0], loxReading[1], loxReading[2], loxReading[3]);
   //wallfollowing();
   MOTForward(1);
  // leftWallfollowing2();
  // if (state == 1) {
    // wallfollowing();
  // } else if (state == 2) {
  //   leftWallfollowing2();
  // }
  //  MOTTurn2(true);
  //  delay(2000) ;
  //  MOTTurn2(true);i
  //  delay(2000);
  //  MOTTurn2(true);
  //  delay(2000);  llowing();
 // blink();
  return;


  // if (!LOXWall('L')) {
  //   MOTForward(DURATION / 2);
  //   MOTBrake();
  //   MOTTurn(true);
  //   MOTForward(DURATION / 2);
  // } else if (!LOXWall('F')) {
  //   MOTForward(1);
  // } else {
  //   MOTTurn(false);
  // }
}