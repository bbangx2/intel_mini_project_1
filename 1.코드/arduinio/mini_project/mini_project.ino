#include <SoftwareSerial.h>

#define LED_PIN 13        // 내장 LED
#define SW1_PIN 2         // 첫번째 스위치
#define SW2_PIN 3         // 두번째 스위치

#define MY_ID "[HKT_ARD]"

SoftwareSerial BTSerial(10, 11); // RX, TX (BT: TXD, RXD)

bool sw1_last = HIGH;
bool sw2_last = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  digitalWrite(LED_PIN, LOW);

  BTSerial.begin(9600);
  Serial.println("Simple BT+LED+Switch Ready");
}

void loop() {
  // 블루투스 명령 수신
  if (BTSerial.available()) {
    String recv = BTSerial.readStringUntil('\n');
    Serial.print("Recv: "); Serial.println(recv);

    // [HKT_STM]DHT@... 수신 시 LED ON
    if (recv.indexOf("[HKT_STM]DHT@") >= 0) {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED ON (by DHT message)");
    }
  }

  // 스위치 1 상태 변화 감지 및 처리
  bool sw1_now = digitalRead(SW1_PIN);
  if (sw1_now != sw1_last) {
    sw1_last = sw1_now;
    if (sw1_now == LOW) { // 1번 스위치 눌렀을 때
      digitalWrite(LED_PIN, LOW); // LED OFF
      Serial.println("SW1 PRESSED, FAN ON");
      BTSerial.println("[HKT_STM]FAN@ON\r\n");
    }
    delay(30);
  }

  // 스위치 2 상태 변화 감지 및 처리
  bool sw2_now = digitalRead(SW2_PIN);
  if (sw2_now != sw2_last) {
    sw2_last = sw2_now;
    if (sw2_now == LOW) { // 2번 스위치 눌렀을 때
      digitalWrite(LED_PIN, LOW); // LED OFF
      Serial.println("SW2 PRESSED, FAN OFF");
      BTSerial.println("[HKT_STM]FAN@OFF\r\n");
    }
    delay(30);
  }

  delay(10);
}
