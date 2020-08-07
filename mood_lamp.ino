#include <ESP8266WiFi.h>                  // wifi,IP adress, Sever,
                                          // Client, UDP class 기능 제공
#include <WiFiUdp.h>                      // NTP통신과 UDP프토토콜을 사용하기 위한 라이브러리
#include <TimeLib.h>                      // 시간, 날짜 관련 데이터를 다루기 위한 라이브러리
#include <LiquidCrystal_I2C.h>            // LCD 작동을 위한 라이브러리

LiquidCrystal_I2C lcd(0x27, 16, 2);       // 0x27 I2C 주소를 가지고 있는 16x2 LCD객체 생성

const char* ssid = "1703";                // 연결할 wifi 이름과 비밀번호
const char* password = "43552618"; 

IPAddress timeServer(106, 247, 248, 106); // NTP 서버

const int timeZone = 9;                   // 표준 시간대를 설정

WiFiUDP Udp;
unsigned int localPort = 8888;            // UDP 패킷을 수신하기 위한 로컬 포트

int cds = A0;                             // 조도센서 연결 핀
int Led = D0;                             // LED 연결 핀
int buzzerPin = D1;                       // 부저 연결 핀
int relay = D3;                           // 릴레이 연결 핀

void setup() 
{
  pinMode(Led, OUTPUT);                   // LED 연결핀 출력모드 설정
  digitalWrite(Led, LOW);                 // LED핀 0V로 변경

  pinMode(buzzerPin, OUTPUT);             // 부저 연결핀 출력모드 설정

  pinMode(relay,OUTPUT);                  // 릴레이 연결핀 출력모드 설정

  Serial.begin(9600);                     // 통신속도
  Serial.println();
  Serial.println("LCD/NTP_Time");
  Serial.print("Connecting to ");
  Serial.println(ssid);  

  lcd.begin(D6, D5);                      // LCD 연결 핀번호
  lcd.backlight();                        // LCD 백라이트 켜기
  lcd.clear();                            // 디스플레이 지우기
  lcd.setCursor(0, 0);                    // 커서를 지정된 열과 행으로 실행 (0, 0)
  lcd.print("Network Clock");
  lcd.setCursor(0,1);
  lcd.print("C2 "); 
  lcd.print(ssid);     

  WiFi.begin(ssid, password);             // wifi 연결시도
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Starting UDP!!");
  Udp.begin(localPort);                   // UDP 통신 시도
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync...");
  setSyncProvider(getNtpTime);            // 시간 동기화
  lcd.clear();
}
time_t prevDisplay = 0;                   // 디지털 시계가 표시되었을 때

void loop()
{ 
  if (timeStatus() != timeNotSet) {
    if (now() != prevDisplay) {           // 시간이 바뀌었을 때만 출력 내용을 업데이트
      prevDisplay = now();
      digitalClockDisplay();  
    }
  }
  delay(1000);
  
  //-------------------------------조도센서 부분-------------------------------------//
  int cdsValue = analogRead(cds);

  Serial.print("cds =  ");                // 시리얼 모니터에 
  Serial.println(cdsValue);               // 수치 출력
 
  if (cdsValue > 300) {
    digitalWrite(Led, HIGH);              // LED 점등
    Serial.println("LED ON (cds > 300)"); // 시리얼 모니터 표시
  } else {
    digitalWrite(Led, LOW);               // LED 소등
    Serial.println("LED OFF (cds < 300)");//시리얼 모니터 표시
  }

  //--------------------------------릴레이 부분-------------------------------------//
  if(cdsValue > 300){                     // 조도센서 값이 300이상 일 때
    Serial.println(LOW);
    digitalWrite(relay, LOW);             // 릴레이를 0V로 변경 -> LED 점등
  }else{
    Serial.println(HIGH);
    digitalWrite(relay,HIGH);             // 릴레이를 5V로 변경 -> LED 소등
  }
}

void digitalClockDisplay(){               // 시간 표시
  String hhmmss, mmddyy;

  if(hour()>=10) {
     hhmmss = hour();
  } else {
     hhmmss = "0";
     hhmmss += hour();
  }
  hhmmss += ":";
  if(minute()>=10) {  
     hhmmss += minute();
  } else {
     hhmmss += "0";
     hhmmss += minute();
  } 
  hhmmss += ":";
  if(second()>=10) {  
     hhmmss += second();
  } else {
     hhmmss += "0";
     hhmmss += second();
  }
  
  if(hhmmss + "0" + minute() + ":" + "0" + // 정각에 부저 울리기
  second() && minute()== 0 && second() == 0) {
    tone(buzzerPin,31,2000);
  }
  
  lcd.setCursor(0, 0);                    // 0번째 줄 0번째 셀부터 입력하기
  lcd.print(hhmmss);                      // 시분초
  lcd.print("  ");                      

  if(month()>=10) {                       // 월
     mmddyy = month();                  
  } else {
     mmddyy = "0";
     mmddyy += month();
  }
  mmddyy += ".";                        
  if(day()>=10) {                         // 일
     mmddyy += day();
  } else {
     mmddyy += "0";
     mmddyy += day();
  }
  mmddyy += ".";                          // 년도
  mmddyy += year();
  
  lcd.setCursor(0, 1);                    // 1번째 줄 0번째 셀부터 입력하기
  lcd.print(mmddyy);                      // 월일년도
  lcd.print("  ");

  Serial.print(hhmmss);                   // 시리얼 모니터에
  Serial.print(" <> ");                   //
  Serial.println(mmddyy);                 // 출력할 내용들
}

//-------- NTP code -------------시간을 동기화 하기 위한 네트워크 프로토콜(NTP)-----------//
const int NTP_PACKET_SIZE = 48; 
byte packetBuffer[NTP_PACKET_SIZE]; 

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; 
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);
      unsigned long secsSince1900;

      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * 3600;
    }
  }
  Serial.println("No NTP Response!!");
  return 0; 
}

void sendNTPpacket(IPAddress &address)
{
  memset(packetBuffer, 0, NTP_PACKET_SIZE);

  packetBuffer[0] = 0b11100011;   
  packetBuffer[1] = 0;     
  packetBuffer[2] = 6;     
  packetBuffer[3] = 0xEC;  
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
             
  Udp.beginPacket(address, 123); 
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
