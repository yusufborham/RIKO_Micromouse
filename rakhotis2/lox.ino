void LOXInit() {
  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_M, OUTPUT);

  pinMode(XSH_B, OUTPUT);
  digitalWrite(XSH_B, LOW);

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

    //digitalWrite(leds[i], loxReading[i] < (WIDTH / 1) && !(lox[i].readRangeStatus() == 2));
    i ? (walls[i] = loxReading[i] < (WIDTH / 1) && !(lox[i].readRangeStatus() == 2) ) : (walls[i] = loxReading[i] < (WIDTH * 0.7 ) && !(lox[i].readRangeStatus() == 2)) ;
    // digitalWrite(leds[i], loxReading[i] < (WIDTH / 1));
    // walls[i] = loxReading[i] < (WIDTH / 1);
    
    // Serial.print(lox[i].readRangeStatus());
    //Serial.print(", ");
  }
  // Serial.println();
  return true;
}
