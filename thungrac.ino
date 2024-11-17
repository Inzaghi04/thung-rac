#include <DistanceSRF04.h>
#include <Servo.h>

DistanceSRF04 Dist;
Servo myservo;
#define BTN_LOA     10
#define SERVO       11
#define SRF04_ECHO  3
#define SRF04_TRIG  2

#define GOC_DONG 130 //Đây là góc đóng của servo
#define GOC_MO 0 //Đây là góc mở của servo
#define INACTIVE_TIME 5000  // Thời gian không có vật thể để kích hoạt chế độ ngủ đông (5 giây)

int distance;
unsigned long previousMillis = 0;
unsigned char autoTrigger = 0;
unsigned long autoMillis = 0;
int sensorValueCBRung = 0; 
uint8_t k = 0;

typedef enum
{
  IDLE_STATE,
  OPEN_STATE,
  CLOSE_STATE
}MODE_STATE;
uint8_t modeRun = IDLE_STATE;
uint32_t timeMillis = 0;
uint8_t timeOpen  = 2;

void setup()
{
  Serial.begin(9600);
  //echo, trigger
  Dist.begin(3, 2);
  myservo.attach(11);
  myservo.write(GOC_DONG);
  pinMode(BTN_LOA, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(A7, INPUT_PULLUP);
  digitalWrite(BTN_LOA, HIGH);

  // Bật cảm biến siêu âm khi bắt đầu
  Dist.begin(3, 2);
}

void loop()
{ 
  Serial.println(sensorValueCBRung);
  switch(modeRun)
  {
    case IDLE_STATE:
      if(readSRF04() == 1 )
      {
        modeRun = OPEN_STATE;
        myservo.write(GOC_MO);
        digitalWrite(BTN_LOA, LOW);
        delay(50);
        digitalWrite(BTN_LOA, HIGH);
        digitalWrite(13, HIGH);
        timeMillis = millis();
      }
      break;
    case OPEN_STATE:
      if(millis() - timeMillis > timeOpen*1000)
      {
         timeMillis = millis() ;
         modeRun = CLOSE_STATE;
      }
      if(readSRF04() == 1 )
      {
        timeMillis = millis();
      }  
      break;
    case CLOSE_STATE:
       myservo.write(GOC_DONG);
       digitalWrite(13, LOW);
       delay(500);
       modeRun = IDLE_STATE;
      break;
  }

  // Kiểm tra thời gian không có vật thể gần thùng rác (sau INACTIVE_TIME)
  if (millis() - autoMillis >= INACTIVE_TIME) {
    // Sau 5 giây không có vật thể, tắt cảm biến siêu âm
    myservo.write(GOC_DONG);  // Giữ servo ở góc đóng
    digitalWrite(13, LOW);    // Tắt đèn
  }

  // Nếu có vật thể gần, bật lại cảm biến siêu âm và servo
  if (readSRF04() == 1) {
    autoMillis = millis();  // Đánh dấu thời gian gần nhất có vật thể
    digitalWrite(13, HIGH);  // Bật đèn
  }
}

uint8_t readSRF04()
{
  // Phần previousMillis >= 100 đây chính là thời gian lấy mẫu của cảm biến siêu âm 100ms
  if (millis() - previousMillis >= 10 )
  {
    previousMillis = millis();
    distance = Dist.getDistanceCentimeter();
    // Phần distance < 10 đây là phần cài đặt khoảng cách cảm biến nhận được kích hoạt mở thùng rác
    if (distance < 10 && distance > 1) 
    {  
        autoMillis = millis();
        return 1;
    }
    return 0;
  }
}
