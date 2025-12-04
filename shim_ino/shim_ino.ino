#include <Arduino.h>

#define DAT_R 25
#define CLK_R 33
#define DAT_L 22
#define CLK_L 23
#define DAT_F 26
#define CLK_F 13
#define CONTROLLER_SUPPLY 21
#define PITCH_HIGH 4700
#define PITCH_LOW 1000
#define FAST_HIGH 14
#define FAST_LOW 10.3
#define SLOW_HIGH 9
#define SLOW_LOW 4
#define PERIOD_MINI 100
#define PERIOD_MAX 500

const int RUD_PIN = A13;
const int ELE_PIN = A10;
const int LED_PIN = 18;
const int BUZZER_PIN = 14;

float offset_front = 0;
float offset_right = 0;
float offset_left = 0;
float pilot_m = 14960.0;
float c_m = 8091.0;
float tail_length = 0.764;
float front_length = 1.0;
float zero_length = 0.0768;

String air_speed_data = "0.000";
float airspeed = 0;

// =====================
// 新BuzzerControllerクラス
// =====================
class BuzzerController {
public:
  BuzzerController(int pin)
    : buzzerPin(pin), lastBeepTime(0), beepOn(false), beepLength(0), beepPeriod(0), pitch(0), lastVal(-999) {
    pinMode(buzzerPin, OUTPUT);
  }

  void sound(float val) {
    unsigned long now = millis();

    // fast zone
    if (val > FAST_LOW) {
      pitch = PITCH_HIGH;
      beepPeriod = calcFastPeriod(val);
      beepLength = beepPeriod / 2;
      manageBeep(now);
    }
    // slow zone
    else if (SLOW_LOW < val && val < SLOW_HIGH) {
      pitch = PITCH_LOW;
      beepPeriod = calcSlowPeriod(val);
      beepLength = beepPeriod / 2;
      manageBeep(now);
    }
    // dead zone (silent)
    else {
      noTone(buzzerPin);
      beepOn = false;
      lastBeepTime = now;
    }
    lastVal = val;
  }

private:
  int buzzerPin;
  unsigned long lastBeepTime;
  bool beepOn;
  int beepLength;
  int beepPeriod;
  int pitch;
  float lastVal;

  int calcFastPeriod(float val) {
    if (val >= FAST_HIGH) return PERIOD_MINI;
    float v = (val - FAST_LOW) / (FAST_HIGH - FAST_LOW);
    return (int)((PERIOD_MAX - PERIOD_MINI) * (1 - v) + PERIOD_MINI);
  }

  int calcSlowPeriod(float val) {
    if (val < SLOW_LOW) return PERIOD_MINI;
    float v = (val - SLOW_LOW) / (SLOW_HIGH - SLOW_LOW);
    return (int)((PERIOD_MAX - PERIOD_MINI) * v + PERIOD_MINI);
  }

  void manageBeep(unsigned long now) {
    if (!beepOn && (now - lastBeepTime >= beepPeriod)) {
      tone(buzzerPin, pitch, beepLength); // ピーッ
      beepOn = true;
      lastBeepTime = now;
    }
    if (beepOn && (now - lastBeepTime >= beepLength)) {
      noTone(buzzerPin);
      beepOn = false;
    }
  }
};

BuzzerController buzzer(BUZZER_PIN);

// ========================
// その他の関数・setup/loop
// ========================

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("start");
  pinMode(DAT_R, INPUT);
  pinMode(CLK_R, OUTPUT);
  pinMode(DAT_L, INPUT);
  pinMode(CLK_L, OUTPUT);
  pinMode(DAT_F, INPUT);
  pinMode(CLK_F, OUTPUT);
  pinMode(RUD_PIN, INPUT);
  pinMode(ELE_PIN, INPUT);
  pinMode(CONTROLLER_SUPPLY, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(CONTROLLER_SUPPLY, HIGH);
  xTaskCreatePinnedToCore(loo1, "loo1", 8192, NULL, 1, NULL, 0);

  offset_front = Read(CLK_F, DAT_F);
  offset_right = Read(CLK_R, DAT_R);
  offset_left = Read(CLK_L, DAT_L);
}

void loo1(void* pvParameters) {
  while (1) {
    airspeed = air_speed_data.toFloat();
    buzzer.sound(airspeed);
    delay(5); // 5msに変更（あまり短すぎても意味ないため）
  }
}

int applyDeadZone(int value, int deadZone = 3) {
  return (value > -deadZone && value < deadZone) ? 0 : value;
}

void loop() {
  float data_front = Read(CLK_F, DAT_F);
  txrx();
  float data_right = Read(CLK_R, DAT_R);
  txrx();
  float data_left = Read(CLK_L, DAT_L);
  txrx();
  float ans_f = offset_front - data_front;
  float ans_r = offset_right - data_right;
  float ans_l = offset_left - data_left;
  float total = ans_f +  ( ans_r * 2  ) ;
  txrx();
  int rud = map(analogRead(RUD_PIN), 0, 3600, 11, -10);
  int ele = map(analogRead(ELE_PIN), 1000, 2800, -10, 9);
  txrx();
  float x = (ans_f * front_length) / total;
  int pilot_cog = 1000 * x;
  char str[100];
  sprintf(str, "%d,%d,%d", -applyDeadZone(ele), -applyDeadZone(rud), 320);
  txrx();
  Serial.println(str);
  txrx();
}

void txrx() {
  if (Serial.available() > 0) {
    String air_data = Serial.readStringUntil(13);
    Serial.read();
    Serial2.println(air_data);
    String dataArray[18];
    int index = 0;
    int startIndex = 0;
    int endIndex = air_data.indexOf(',', startIndex);
    while (endIndex != -1) {
      dataArray[index++] = air_data.substring(startIndex, endIndex);
      startIndex = endIndex + 1;
      endIndex = air_data.indexOf(',', startIndex);
    }
    dataArray[index] = air_data.substring(startIndex);
    air_speed_data = dataArray[4];
  }
}

float Read(int CLK, int DAT) {
  long sum = 0;
  for (int i = 0; i < 3; i++) {
    long data = 0;
    while (digitalRead(DAT) != 0)
      ;
    for (char i = 0; i < 24; i++) {
      digitalWrite(CLK, 1);
      delayMicroseconds(1);
      digitalWrite(CLK, 0);
      delayMicroseconds(1);
      data = (data << 1) | (digitalRead(DAT));
    }
    digitalWrite(CLK, 1);
    delayMicroseconds(1);
    digitalWrite(CLK, 0);
    delayMicroseconds(1);
    data = data ^ 0x800000;
    sum += data;
  }
  float data = sum / 3;
  float volt = data * (4.2987 / 16777216.0 / 128);
  return volt / (0.001 * 4.2987 / 20000.0);
}