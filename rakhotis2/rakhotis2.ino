#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <PID_v1.h>


#define GYRO_CONFIG 0x1B

#define WIDTH 170
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
#define SDIO 4

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
bool forward_flag = 1;
bool left_turn_flag = 0;
unsigned long t = 0;
// wall flags
// if true means there is a WALL
// wall array // forward / LEFT / RIGHT
bool walls[3] = { 0, 0, 0 };
bool walls_prv[3] = { 0, 1, 1 };
// PID Constatnts
const double motKp = 4;
const double motKi = 2;
const double motKd = 1;  //0.8068;

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

byte ledState = 3;

TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;

void Task1(void *pvParameters) {
  while (1) {
    IMUUpdate();
    Serial.println("Task 1");
    vTaskDelay(1);
  }
}

void Task2(void *pvParameters) {
  while (1) {
    LOXRead();
    Serial.println("Task 2");
    vTaskDelay(1);
  }
}

/////// encoders ////////

#define ticks 0.11
//100 * tick = cir 11.14

float distance_turn_threshold = 5;

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
float pitch, yaw, roll;  // MAKE GLOBAL

int mode = 0;

void MOTInit() {
  motPID.SetMode(AUTOMATIC);
  motPID.SetOutputLimits(-255, 255);
  motPID.SetSampleTime(50);

  pinMode(MOT1A, OUTPUT);
  pinMode(MOT1B, OUTPUT);
  pinMode(MOT2A, OUTPUT);
  pinMode(MOT2B, OUTPUT);
}

// void MOTForward(long durationMillis) {
//   long startMillis = millis();
//   while (millis() - startMillis < durationMillis) {
//     //IMUUpdate();
//     LOXRead();
//     //measureDistance();

//     static double leftSensor = 0, rightSensor = 0;
//     leftSensor = loxReading[1];
//     rightSensor = loxReading[2];

//     if (leftSensor + rightSensor <= WIDTH) {  // < 170
//       motInput = leftSensor - rightSensor;
//       motPID.Compute();
//       pid_case = 1;

//     }

//     else if (leftSensor + rightSensor > WIDTH && leftSensor + rightSensor < 2 * WIDTH) {  // < 340
//       (rightSensor > leftSensor) ? (motInput = leftSensor - (rightSensor - WIDTH)) : (motInput = (leftSensor - WIDTH) - rightSensor);
//       motPID.Compute();
//       pid_case = 2;

//     }

//     else if (leftSensor + rightSensor > 2 * WIDTH && leftSensor + rightSensor < 3 * WIDTH && !(lox[1].readRangeStatus() == 2) && !(lox[2].readRangeStatus() == 2)) {  // < 510
//       (rightSensor > leftSensor) ? (motInput = leftSensor - (rightSensor - 2 * WIDTH)) : (motInput = (leftSensor - 2 * WIDTH) - rightSensor);
//       //motOutput = 0;
//       motPID.Compute();
//       pid_case = 3;


//     } else {

//       pid_case = 0;
//       motOutput = 0;

//     }


//     // else if (leftSensor + rightSensor > 3*WIDTH  &&  leftSensor + rightSensor < 4*WIDTH  ){ // < 580
//     //   (rightSensor > leftSensor) ? (motInput = leftSensor - ( rightSensor - 3*WIDTH )) : (motInput = (leftSensor - 3*WIDTH) - rightSensor  ) ;
//     //   pid_case = 4 ;
//     //   motPID.Compute();

//     // }

//     // else if (leftSensor + rightSensor > 4*WIDTH  &&  leftSensor + rightSensor < 5*WIDTH  ){ // <
//     //   (rightSensor > leftSensor) ? (motInput = leftSensor - ( rightSensor - 4*WIDTH )) : (motInput = (leftSensor - 4*WIDTH) - rightSensor  ) ;
//     //   pid_case = 5 ;
//     //   motPID.Compute();

//     // }

//     // right motor
//     analogWrite(MOT1A, 0);
//     analogWrite(MOT1B, constrain(255 + motOutput, 0, 255));
//     //analogWrite(MOT1B, 127);

//     // left motor
//     analogWrite(MOT2A, 0);
//     analogWrite(MOT2B, constrain(255 - motOutput, 0, 255));
//     //analogWrite(MOT2B, 127);


//     Serial.printf("F: %d, L: %d, R: %d, B: %d, motOutput: %f  and my case is %d  and my sum is  %d ", loxReading[0], loxReading[1], loxReading[2], motOutput, pid_case, loxReading[1] + loxReading[2]);
//     //Serial.printf(" distance_right  is   %d  distance_left   %d and distance_avg is   %d   \n", distance_right, distance_left, distance_avg);
//     Serial.printf("right %d  left %d \n ", counts_right_pinA, counts_left_pinA);
//   }
// }
void MOTForward() {
  IMUUpdate();
  LOXRead();

  //measureDistance();

  static double leftSensor = 0, rightSensor = 0;
  leftSensor = loxReading[1];
  rightSensor = loxReading[2];

  if (leftSensor + rightSensor <= WIDTH) {  // < 170
    motInput = leftSensor - rightSensor;
    motPID.Compute();
    pid_case = 1;

  }

  else if (leftSensor + rightSensor > WIDTH && leftSensor + rightSensor < 2 * WIDTH) {  // < 340
    (rightSensor > leftSensor) ? (motInput = leftSensor - (rightSensor - WIDTH)) : (motInput = (leftSensor - WIDTH) - rightSensor);
    motPID.Compute();
    pid_case = 2;

  }

  else if (leftSensor + rightSensor > 2 * WIDTH && leftSensor + rightSensor < 3 * WIDTH && !(lox[1].readRangeStatus() == 2) && !(lox[2].readRangeStatus() == 2)) {  // < 510
    (rightSensor > leftSensor) ? (motInput = leftSensor - (rightSensor - 2 * WIDTH)) : (motInput = (leftSensor - 2 * WIDTH) - rightSensor);
    //motOutput = 0;
    motPID.Compute();
    pid_case = 3;


  } else {

    pid_case = 0;
    motOutput = 0;
  }


  // right motor
  analogWrite(MOT1A, 0);
  analogWrite(MOT1B, constrain(255 + motOutput, 0, 255));
  //analogWrite(MOT1B, 127);

  // left motor
  analogWrite(MOT2A, 0);
  analogWrite(MOT2B, constrain(255 - motOutput, 0, 255));
  //analogWrite(MOT2B, 127);


  Serial.printf("F: %d, L: %d, R: %d, B: %d, motOutput: %f  and my case is %d  and my sum is  %d ", loxReading[0], loxReading[1], loxReading[2], motOutput, pid_case, loxReading[1] + loxReading[2]);
  //Serial.printf(" distance_right  is   %d  distance_left   %d and distance_avg is   %d   \n", distance_right, distance_left, distance_avg);
  Serial.printf("right %d  left %d \n ", counts_right_pinA, counts_left_pinA);
}


void moveforward() {
  resetDistance();
  do {
    MOTForward();
    measureDistance();
  } while (distance_left <= 5);
  MOTBrake();
}

void MOTBrake() {
  analogWrite(MOT1A, 255);
  analogWrite(MOT1B, 255);
  analogWrite(MOT2A, 255);
  analogWrite(MOT2B, 255);
  delay(100);
  LOXRead();
  analogWrite(MOT1A, 0);
  analogWrite(MOT1B, 0);
  analogWrite(MOT2A, 0);
  analogWrite(MOT2B, 0);
  delay(200);
}



void TurnWithEncoder(bool isLeft) {
  resetDistance();

  digitalWrite(MOT1A, LOW);
  digitalWrite(MOT1B, LOW);
  digitalWrite(MOT2A, LOW);
  digitalWrite(MOT2B, LOW);

  analogWrite(MOT1A, !isLeft ? 0 : rotating_speed);
  analogWrite(MOT1B, !isLeft ? rotating_speed : 0);
  analogWrite(MOT2A, !isLeft ? rotating_speed : 0);
  analogWrite(MOT2B, !isLeft ? 0 : rotating_speed);

  do {
    LOXRead();
    measureDistance();
    Serial.printf("right distance is %f and the left distance is %f and the averafge distance is %f \n", distance_right, distance_left, distance_avg);
  } while (distance_left < distance_turn_threshold);
  Serial.println("finished rotation ");
  isLeft ? (direction ? direction = 3 : direction -= 1) : direction = (direction + 1) % 4;
  Serial.println(direction);
  MOTBrake();
  delay(200);
}

void TurnWithIMU(bool isLeft) {
  MOTBrake();
  IMUUpdate();  // 87
  //LOXRead();
  float yaw_offset = yaw;
  int error;  // 177 - 87
  int Kpt = 2;

  do {
    IMUUpdate();
    //LOXRead();
    if (yaw - yaw_offset <= -180)
      yaw += 360;
    else if (yaw - yaw_offset >= 180)
      yaw -= 360;
    error = 90 - abs(yaw - yaw_offset);
    Serial.println(error);
    int control_signal = Kpt * error + 30;
    analogWrite(MOT1A, !isLeft ? 0 : control_signal);
    analogWrite(MOT1B, !isLeft ? control_signal : 0);
    analogWrite(MOT2A, !isLeft ? control_signal : 0);
    analogWrite(MOT2B, !isLeft ? 0 : control_signal);
  } while (error > 8);

  MOTBrake();
}

void TurnWithDelay(bool isLeft) {
  LOXRead();
  MOTBrake();

  analogWrite(MOT1A, !isLeft ? 0 : rotating_speed);
  analogWrite(MOT1B, !isLeft ? rotating_speed : 0);
  analogWrite(MOT2A, !isLeft ? rotating_speed : 0);
  analogWrite(MOT2B, !isLeft ? 0 : rotating_speed);

  delay(380);
  MOTBrake();
}

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

void leftAdv() {
  LOXRead();
  if (!walls[left]) {
    forward_flag = 0;
    left_turn_flag = 1;
    walls_prv[forward] = walls[forward];
    walls_prv[left] = walls[left];
    walls_prv[right] = walls[right];
    digitalWrite(LED_L, HIGH);
    // MOTForward(250);
    //moveforward();

    t = millis();
    while (millis() > t + 260)
      MOTForward();

    TurnWithIMU(true);

    if (walls[forward]) {
      TurnWithIMU(true);
    } else {
      t = millis();
      while (millis() > t + 260)
        MOTForward();
    }
    // moveforward();
    // MOTForward(250);
    digitalWrite(LED_L, LOW);

  }

  else if (walls[forward]) {
    forward_flag = 0;
    left_turn_flag = 0;
    walls_prv[forward] = walls[forward];
    walls_prv[left] = walls[left];
    walls_prv[right] = walls[right];
    digitalWrite(LED_R, HIGH);
    TurnWithIMU(false);
    digitalWrite(LED_R, LOW);

  }

  else if (!walls[forward] && left_turn_flag ? ((!forward_flag) ? !walls_prv[left] : 1) : ((!forward_flag) ? !walls_prv[right] : 1)) {  // if no wall infront of you and before that you made a right turn check if brfore turning you had a wall to your right or not
    forward_flag = 1;
    left_turn_flag = 0;
    digitalWrite(LED_M, HIGH);
    MOTForward();
    walls_prv[forward] = walls[forward];
    walls_prv[left] = walls[left];
    walls_prv[right] = walls[right];

    digitalWrite(LED_M, LOW);
  }
}

void rightAdv() {
  LOXRead();
  if (!walls[right]) {
    forward_flag = 0;
    left_turn_flag = 1;
    walls_prv[forward] = walls[forward];
    walls_prv[left] = walls[left];
    walls_prv[right] = walls[right];
    digitalWrite(LED_R, HIGH);
    // MOTForward(250);
    //moveforward();

    t = millis();
    while (millis() > t + 260)
      MOTForward();

    TurnWithIMU(false);

    if (walls[forward]) {
      TurnWithIMU(false);
    } else {
      t = millis();
      while (millis() > t + 260)
        MOTForward();
    }
    // moveforward();
    // MOTForward(250);
    digitalWrite(LED_R, LOW);

  }

  else if (walls[forward]) {
    forward_flag = 0;
    left_turn_flag = 0;
    walls_prv[forward] = walls[forward];
    walls_prv[left] = walls[left];
    walls_prv[right] = walls[right];
    digitalWrite(LED_L, HIGH);
    TurnWithIMU(true);
    digitalWrite(LED_L, LOW);

  }

  else if (!walls[forward] && left_turn_flag ? ((!forward_flag) ? !walls_prv[right] : 1) : ((!forward_flag) ? !walls_prv[left] : 1)) {  // if no wall infront of you and before that you made a right turn check if brfore turning you had a wall to your right or not
    forward_flag = 1;
    left_turn_flag = 0;
    digitalWrite(LED_M, HIGH);
    MOTForward();
    walls_prv[forward] = walls[forward];
    walls_prv[left] = walls[left];
    walls_prv[right] = walls[right];

    digitalWrite(LED_M, LOW);
  }
}

void leftNorm() {
  LOXRead();
  IMUUpdate();
  if (!walls[left]) {
    digitalWrite(LED_L, HIGH);
    //moveforward();
    long startTime = millis();
    while (millis() < startTime + 260)
      MOTForward();
    TurnWithIMU(true);
    //moveforward();
    startTime = millis();
    while (millis() < startTime + 260)
      MOTForward();
    digitalWrite(LED_L, LOW);
  }

  else if (walls[forward]) {  // fe 7eta odamo
    digitalWrite(LED_R, HIGH);
    TurnWithIMU(false);
    digitalWrite(LED_R, LOW);
  }

  else {
    digitalWrite(LED_M, HIGH);
    MOTForward();
    digitalWrite(LED_M, LOW);
  }
}

void rightNorm() {
  LOXRead();
  IMUUpdate();
  if (!walls[right]) {
    digitalWrite(LED_R, HIGH);
    //moveforward();
    long startTime = millis();
    while (millis() < startTime + 260)
      MOTForward();
    TurnWithIMU(false);
    //moveforward();
    startTime = millis();
    while (millis() < startTime + 260)
      MOTForward();
    digitalWrite(LED_R, LOW);
  }

  else if (walls[forward]) {  // fe 7eta odamo
    digitalWrite(LED_L, HIGH);
    TurnWithIMU(true);
    digitalWrite(LED_L, LOW);
  }

  else {
    digitalWrite(LED_M, HIGH);
    MOTForward();
    digitalWrite(LED_M, LOW);
  }
}

void setup() {
  Serial.begin(115200);
  Wire1.begin(SDA, SCL);
  Wire.begin(SDIO, SCKL);
  IMUInit();

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

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_M, OUTPUT);

  // xTaskCreate(
  //   Task1,
  //   "Task 1",
  //   1000,
  //   NULL,
  //   1,
  //   &Task1Handle
  // );

  // xTaskCreate(
  //   Task2,
  //   "Task 2",
  //   1000,
  //   NULL,
  //   1,
  //   &Task2Handle
  // );

  randomSeed(1);
  Serial.println("Press any button..");
  // while (digitalRead(36) && digitalRead(39))
  //   ;

  bool btn = true;
  digitalWrite(LED_L, HIGH);
  digitalWrite(LED_M, HIGH);
  digitalWrite(LED_R, LOW);
  while (digitalRead(36)) {
    if (!digitalRead(39) && btn) {
      mode = (mode + 1) % 6;
      if (mode == 0) {
        digitalWrite(LED_L, HIGH);
        digitalWrite(LED_M, HIGH);
        digitalWrite(LED_R, LOW);
      } else if (mode == 1) {
        digitalWrite(LED_L, LOW);
        digitalWrite(LED_M, HIGH);
        digitalWrite(LED_R, HIGH);
      } else if (mode == 2) {
        digitalWrite(LED_L, HIGH);
        digitalWrite(LED_M, LOW);
        digitalWrite(LED_R, LOW);
      } else if (mode == 3) {
        digitalWrite(LED_L, LOW);
        digitalWrite(LED_M, LOW);
        digitalWrite(LED_R, HIGH);
      } else if (mode == 4) {
        digitalWrite(LED_L, HIGH);
        digitalWrite(LED_M, HIGH);
        digitalWrite(LED_R, HIGH);
      } else if (mode == 5) {
        digitalWrite(LED_L, HIGH);
        digitalWrite(LED_M, LOW);
        digitalWrite(LED_R, HIGH);
      }
    }
    btn = digitalRead(39);
    delay(50);
  }

  digitalWrite(LED_L, LOW);
  digitalWrite(LED_R, LOW);
  digitalWrite(LED_M, HIGH);
  delay(1000);
  digitalWrite(LED_M, LOW);
  //TurnWithIMU(true);
  // delay(500);
  //  moveforward();
  //  delay(1000);
  //  moveforward();
  //  delay(1000);
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

  // LOXRead();
  // Serial.printf("[%d] ", micros() - lastMicros);
  // lastMicros = micros();
  //Serial.printf("F: %d, L: %d, R: %d, B: %d\n", loxReading[0], loxReading[1], loxReading[2], loxReading[3]);

  //MOTForward(1);

  if (mode == 0) {
    Serial.println("leftAdv");
    leftAdv();
  } else if (mode == 1) {
    Serial.println("rightAdv");
    rightAdv();
  } else if (mode == 2) {
    Serial.println("leftNorm");
    leftNorm();
  } else if (mode == 3) {
    Serial.println("rightNorm");
    rightNorm();
  } else if (mode == 4) {
    Serial.println("randAdv");
    if (random(2))
      leftAdv();
    else
      rightAdv();
  } else if (mode == 5) {
    Serial.println("randNorm");
    if (random(2))
      leftNorm();
    else
      rightNorm();
  }
}