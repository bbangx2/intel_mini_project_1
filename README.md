라즈베리 파이 - 메인 서버, STM32 및 아두이노 보드와 통신, DB관리
STM32 보드 - LCD, Wifi 모듈, 온습도 센서, 스위치, 서보모터 제어, EZ모터 제어
아두이노 - 블루투스 모듈, 스위치, 부저, LED 제어

 STM32 보드 
스위치 입력 - PC0-2 핀 (GPIO INPUT)
FAN 제어 - PC8 (GPIO Output)
서보모터 (3개) - PA6-7, PB0 (TIM3_CH1-3 PWM Output)
DHT11 Sensor - PC10 (GPIO output)
LCE - PB8-9 (I2C SDA/SCL)
WIFI ESP Module - PC6-7 (UART6)

아두이노
LED & 부저 - D13
블루투스 module - D11, D10 TX, RX (RXD, TXD)
스위치 입력 - D2-D3

C언어를 이용하여 라즈베리 파이에 메인 서버 구축.
STM32보드와 아두이노 보드의 클라이언트 ID와 Password 확인 후 소켓 연결 승인
수신 데이터를 MariaDB에 저장 및 관리 (SQL)
CUBE IDE를 이용 C언어 위주의 STM32 보드 코딩 (HAL 드라이버 이용)
다양한 패리패럴들의 조작 및 센서 센싱 코드 작성
지정된 시간이 되면 자동으로 정확한 알약 및 영양제를 정확한 갯수만큼 자동 배출
실시간으로 습도 및 온도 측정 (5초 단위) 적정 온,습도 이탈 시 라즈베리 파이로 알림 전송
라즈베리 파이에서 아두이노로 이상 알림 전송 
아두이노 스케치를 이용하여 사용자에게 부저 및 LED 알림 전송
스위치 값을 입력받아 EZ 팬 모터 구동 신호 발생, STM32보드에서 팬 구동
