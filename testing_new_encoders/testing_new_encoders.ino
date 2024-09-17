#define GYRO_CONFIG 0x1B

#define WIDTH 200
#define DURATION 1000
#define TURN 90

#define BTN1 36
#define BTN2 39

#define ENC1A 34
#define ENC1B 35

#define ENC2A 17
#define ENC2B 16

#define MOT1A 32
#define MOT1B 33
#define MOT2A 25
#define MOT2B 26

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

#define MOT1A 32
#define MOT1B 33
#define MOT2A 25
#define MOT2B 26
// initialzing LEDS // middle // left // right // back

unsigned long t = 0;

/////// encoders ////////

#define ticks 10
long counts_left_pinA = 0;
long counts_left_pinB = 0;

long counts_right_pinA = 0;
long counts_right_pinB = 0;

double distance_right = 0;
double distance_left = 0;
double distance_avg = 0;

bool flag_right_encoder = 0;
bool flag_left_encoder = 0;

void IRAM_ATTR rightEncoderPinAHandle() {
  // digitalRead(ENC1B) ? counts_left++ : counts_left-- ;
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
/*void measureDistance(){

    distance_left = counts_left*ticks ;
    distance_right = counts_right*ticks ;
    distance_avg = (distance_left+distance_right)/2 ;
  
}*/

/*void resetDistance(){

    counts_left    =  0 ;
    counts_right   =  0 ;
    distance_left  =  0 ;
    distance_right =  0 ;
    distance_avg   =  0 ;

}*/
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
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  encoderInit();
  pinMode(MOT1A, OUTPUT);
  pinMode(MOT1B, OUTPUT);
  pinMode(MOT2A, OUTPUT);
  pinMode(MOT2B, OUTPUT);
  pinMode(36, INPUT);
  pinMode(39, INPUT);

  while (digitalRead(36) && digitalRead(39))
    ;

  delay(1000);
  analogWrite(MOT1A, 0);
  analogWrite(MOT1B, 200);

  analogWrite(MOT2A, 0);
  analogWrite(MOT2B, 200);

  //t = millis();
}

void loop() {
  Serial.printf("%d    %d     %d      %d \n", counts_right_pinB, counts_right_pinA, counts_left_pinA, counts_left_pinB);
  // if (millis() > 1000 + t) {

  //   analogWrite(MOT1A, 0);
  //   analogWrite(MOT1B, 0);

  //   analogWrite(MOT2A, 0);
  //   analogWrite(MOT2B, 0);
  // }

  // put your main code here, to run repeatedly:
}
