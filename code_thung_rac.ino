#include "EspMQTTClient.h" //để sử dụng kết nối WiFi và giao tiếp với MQTT broker.
#include <Ultrasonic.h> //Sử dụng thư viện để giao tiếp với cảm biến siêu âm HC-SR04.
#include <ArduinoJson.h>  //Tạo và phân tích chuỗi JSON
#include <ESP32Servo.h>   // Bao gồm thư viện Servo

// Khai báo chân cảm biến ánh sáng
#define trig 20    
#define echo 21

// --- LED ---
#define ledGreen 12  // LED xanh: Rỗng/ít rác
#define ledYellow 11 // LED vàng: Lưng lửng rác
#define ledRed 10   // LED đỏ: Đầy rác

// --- Cảm biến mưa ---
#define rainSensor 22 // Chân DO của cảm biến mưa

// --- Động cơ Servo ---
Servo myServo;              // Tạo đối tượng Servo
#define servoPin 15    // Chân tín hiệu của Servo

// --- Góc Servo cho nắp thùng ---
#define ClosedAngle  0   // Góc đóng nắp
#define OpenAngle  90    // Góc mở nắp (hoặc 180 tùy thuộc vào cơ cấu)

#define MAX_DISTANCE_CM 40.0
#define MIN_DISTANCE_CM 5.0



Ultrasonic ultrasonic(trig, echo);//tạo một đối tượng cảm biến siêu âm dùng thư viện Ultrasonic.h.
// một số câu lệnh khác vs thư viện ultrasonic.h: 
//long distance = ultrasonic.read();  // Tự động tính khoảng cách (cm)

//tạo một kết nối từ ESP8266/ESP32 đến MQTT broker qua Wi-Fi.
EspMQTTClient client(
  "aaaaaaaa",     // Thay bằng SSID Wi-Fi của bạn
  "12345678",     // Thay bằng Mật khẩu Wi-Fi của bạn
  "192.168.222.92",   // IP của MQTT Broker
  "",                 // Bỏ trống nếu không dùng Username
  "",                 // Bỏ trống nếu không dùng Password
  "TestClient",       // Client ID
  1883                // Port MQTT (mặc định là 1883)
);

// Biến để quản lý thời gian gửi tin nhắn định kỳ cho tất cả các cảm biến
unsigned long previousMillis = 0; //Lưu thời điểm lần cuối gửi dữ liệu (đơn vị: mili giây).
const long interval = 5000; // Gửi tin nhắn mỗi 5 giây

void setup()
{
  Serial.begin(115200); //Mở cổng Serial monitor để giao tiếp với máy tính với tốc độ 115200 bps. Dùng để xem log, debug, hoặc xuất dữ liệu.
  Serial.println("Starting MQTT Client with Light Sensor...");

  client.enableDebuggingMessages(); //Bật chế độ gỡ lỗi (debug) cho EspMQTTClient, cho phép bạn thấy thông tin như: kết nối Wi-Fi, MQTT, lỗi... qua Serial Monitor.

  pinMode(ledGreen,OUTPUT);// cấu hình đèn xanh ban đầu là tắt 
  digitalWrite(ledGreen, LOW);
  pinMode(ledYellow,OUTPUT);// cấu hình đèn vàng ban đầu là tắt 
  digitalWrite(ledYellow, LOW);
  pinMode(ledRed,OUTPUT);// cấu hình đèn đỏ ban đầu là tắt 
  digitalWrite(ledRed, LOW);

  pinMode(rainSensor, INPUT); // DO của cảm biến mưa

  myServo.setPeriodHertz(50);    // Tần số PWM 50Hz
  myServo.attach(servoPin, 500, 2400); // Thiết lập xung PWM cho ESP32
  myServo.write(ClosedAngle);

  delay(1000); // Đợi servo ổn định
}

void onConnectionEstablished() //hàm callback (được gọi tự động) khi kết nối WiFi và MQTT đã thành công.
{
  Serial.println("WiFi and MQTT Connection Established!");

  client.subscribe("mytopic/test", [](const String & payload) {              // Đăng ký lắng nghe topic "mytopic/test"

    Serial.println("Received on mytopic/test: " + payload);
  });

  client.subscribe("khoang_cach/#", [](const String & topic, const String & payload) {          //Đăng ký lắng nghe tất cả các sub-topic thuộc "khoang_cach/"
    Serial.println("(From khoang_cach) topic: " + topic + ", payload: " + payload);             //Dấu # là wildcard trong MQTT (giống như “mọi thứ phía sau”).
  });

  client.publish("mytopic/test", "ESP32 TestClient with Light Sensor is online!");             //Gửi một thông báo tới topic "mytopic/test" để báo rằng ESP32 đã khởi động xong và đang hoạt động.
}

int calculateFillPercentage(float distance_cm) {
  float percent = 100.0 * (MAX_DISTANCE_CM - distance_cm) / (MAX_DISTANCE_CM - MIN_DISTANCE_CM);
  percent = constrain(percent, 0, 100); //Đảm bảo giá trị nằm trong khoảng 0–100%, tránh trường hợp kết quả âm hoặc lớn hơn 100 do lỗi đo
  return (int)round(percent);//Làm tròn kết quả và ép kiểu sang int để dễ hiển thị hoặc xử lý.

}
void resetLEDs() {
  digitalWrite(ledRed, LOW);
  digitalWrite(ledYellow, LOW);
  digitalWrite(ledGreen, LOW);
}

void loop()
{
  client.loop(); // LUÔN LUÔN gọi client.loop()

  unsigned long currentMillis = millis(); // lấy thời gian hiện tại 

  if (currentMillis - previousMillis >= interval) {//Dùng millis() để chạy định kỳ 5 giây một lần, không dùng delay() → giúp chương trình không bị treo.

    previousMillis = currentMillis;//sau khi đo thì gắn lại mốc thời gian 

    if (client.isConnected()) {  //Kiểm tra MQTT có đang kết nối không trước khi gửi dữ liệu.
        // Đọc giá trị từ cảm biến ánh sáng
      long distance_cm = ultrasonic.read(); // Đọc khoảng cách từ cảm biến
      int rainStatus = digitalRead(rainSensor); // Đọc trạng thái chân DO

      if (rainStatus == LOW) { // Thường là LOW khi có mưa, HIGH khi không (tùy module)
        Serial.println("--> TROI DANG MUA! Dong nap thung rac.");
        myServo.write(ClosedAngle); // Đảm bảo nắp đóng khi trời mưa
      
      } else {
        Serial.println("--> Troi khong mua.");
        myServo.write(OpenAngle); // Mở nắp nếu không mưa và còn chỗ
      }

      if (distance_cm == -1 || distance_cm == 0 || distance_cm > 400) {  // Xử lý trường hợp lỗi hoặc ngoài tầm
        Serial.println("Error or out of range.");
      } else { // Sử dụng giá trị khoảng cách
        Serial.print("Distance: ");
        Serial.print(distance_cm);
        Serial.println(" cm"); // in thông tin trong serial monitor 

        // Gửi qua MQTT:
        Serial.println("Published: khoang_cach  " + String(distance_cm));
        // chuyển đổi sang phần trăm và gửi đi 
        int percent_full = calculateFillPercentage(distance_cm);
        StaticJsonDocument<200> doc;
        // Thêm các cặp key-value vào đối tượng JSON
        doc["distance_cm"] = distance_cm;
        doc["percent_full"] = percent_full;
        doc["rainStatus"] = rainStatus ;
        // Chuyển đối tượng JSON thành chuỗi JSON
        String a;
        serializeJson(doc, a);
        client.publish("khoang_cach", a);
      }

      resetLEDs();
      if (distance_cm <= 5) {
        digitalWrite(ledRed, HIGH);   // Đèn đỏ sáng: Thùng đầy
        Serial.println("--> THUNG RAC DAY!");
      } else if (distance_cm <= 20) {
        digitalWrite(ledYellow, HIGH); // Đèn vàng sáng: Thùng lưng lửng
        Serial.println("--> THUNG RAC LUNG LUNG.");
      } else {
        digitalWrite(ledGreen, HIGH);  // Đèn xanh sáng: Thùng rỗng/ít rác
        Serial.println("--> THUNG RAC IT RAC.");
      }

    } else {
      Serial.println("MQTT not connected, skipping sensor publish.");
    }

  }

}

