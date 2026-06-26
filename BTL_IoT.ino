/////////////////////////////////////// ĐÃ XONG BTL IOT NGÀY 30/03/2025  Ở NHÀ MINH *****************************************************************
#include <WiFi.h>  //giúp ESP32 kết nối với mạng WiFi
#include <HTTPClient.h> //thực hiện các yêu cầu HTTP như GET và POST, giúp ESP32 gửi và nhận dữ liệu từ web hoặc gọi API. (API là một "giao diện lập trình ứng dụng" như gg sheet)
#include <ArduinoJson.h> //xử lý dữ liệu JSON, giúp dễ dàng phân tích và tạo dữ liệu JSON khi giao tiếp với các API hoặc lưu trữ dữ liệu có cấu trúc.
#include <PubSubClient.h> //giao tiếp với giao thức MQTT (Message Queuing Telemetry Transport), một giao thức giao tiếp giữa các thiết bị(cảm biến, RFID) và máy chủ Iot (thingsboard)
#include <SPI.h> //SPI (Serial Peripheral Interface) giúp giao tiếp với các thiết bị ngoại vi như cảm biến hoặc mô-đun RFID.
#include <MFRC522.h> //điều khiển module RFID MFRC522, cho phép ESP32 đọc dữ liệu từ thẻ RFID để xác thực người dùng hoặc điều khiển thiết bị.
#include <ESP32Servo.h> //điều khiển động cơ servo trên ESP32, giúp tạo chuyển động quay chính xác cho các ứng dụng như mở khóa cửa bằng thẻ RFID.
#include <time.h>  //Cung cấp các hàm liên quan đến thời gian, giúp ESP32 đồng bộ hóa thời gian từ máy chủ NTP hoặc quản lý thời gian trong hệ thống.
#include <map>  //Thư viện STL của C++ giúp lưu trữ và tra cứu dữ liệu dưới dạng cặp key-value, hữu ích cho việc quản lý dữ liệu trong chương trình.
#include <SPIFFS.h> // SPIFFS (SPI Flash File System) cho phép lưu trữ dữ liệu trên bộ nhớ flash của ESP32, giúp lưu trạng thái hệ thống ngay cả khi thiết bị khởi động lại.
#include <WiFiManager.h>  //Hỗ trợ quản lý WiFi, giúp ESP32 tự động kết nối với mạng WiFi đã lưu hoặc cung cấp cổng kết nối để người dùng nhập thông tin WiFi nếu chưa có mạng nào được lưu trước đó.

// const char* ssid = "DIENTHOAICUATOI";
// const char* password = "123456789";
const char* AP_NAME = "ESP32-CarPark";  // Tên AP khi vào chế độ cấu hình, char* là con trỏ trỏ tới một chuỗi ký tự (kiểu char), const ở đây có nghĩa là giá trị mà con trỏ trỏ tới là hằng số, không được phép thay đổi.
const char* AP_PASSWORD = "12345678"; //"12345678" là một chuỗi không thay đổi được, và AP_PASSWORD là một con trỏ trỏ đến chuỗi đó.
const char* thingsboardServer = "demo.thingsboard.io";
const char* accessToken = "WlVrXoOZJu85j0PhTa3V";

std::map<String, int> scanMap; //biến toàn cục hoặc cục bộ tên là scanMap, có kiểu là std::map<String, int>. Đây là bản đồ ánh xạ (map) trong C++ chuẩn, dùng để lưu các cặp khóa–giá trị (key–value).

WiFiClient espClient;  //Tạo một kết nối TCP/IP qua WiFi để có thể kết nối từ xa với máy chủ
PubSubClient client(espClient);  // sử dụng để thực thi kết nối và giao tiếp với máy chủ MQTT
// => Tổng kết 2 đối tượng trên là cho phép ESP32 gửi và nhận dữ liệu từ máy chủ MQTT, giúp thiết bị hoạt động như một nút IoT thông minh.

#define SS_PIN 5 // chân ss của RFID dùng để giao tiếp SPI , SCK-18, MISO-19, MOSI-23
#define RST_PIN 22 // chân reset RFID 
#define IR_SENSOR_PIN 4 // Chân GPIO cho cảm biến hồng ngoại thứ 1
#define SERVO_PIN 13
#define BUZZER_PIN 15
#define IR_SENSOR_PIN_2 14  // Chân GPIO cho cảm biến hồng ngoại thứ 2

Servo myServo; //Tạo một đối tượng myServo thuộc lớp Servo, dùng để điều khiển động cơ servo mở/đóng
MFRC522 rfid(SS_PIN, RST_PIN);  // Tạo đối tượng rfid từ thư viện MFRC522, sử dụng các chân SS_PIN và RST_PIN
std::map<String, bool> carStatus; // Lưu trạng thái xe: true = IN, false = OUT
bool objectDetected = false; // Xác định xem có vật thể nào trước cảm biến hồng ngoại không.
bool doorOpen = false;  // biến kiểm tra trạng thái cửa chắn (servo), true nếu đang mở.
bool manualOverride = false;  //  biến điều khiển thủ công hệ thống, có thể dùng để mở cửa bằng tay trong trường hợp cần thiết.

bool outdate = 0; // biến hết hạn ( lượt quét ), giá trị mặc định là 0 nghĩa là còn hạn, khi nào cập nhật lên 1 là hết hạn

bool reset; // biến reset
///// unsigned dùng để khai báo biến số nguyên không dấu (không có giá trị âm) nhận giá trị từ âm trở lên,||||||| int là kiểu số nguyên có dấu ( không có phần thập phân )
unsigned long lastObjectTime = 0; //biến ghi nhận thời điểm phát hiện vật thể trước cảm biến IR.
unsigned long objectGoneTime = 0; //biến ghi nhận thời điểm vật thể rời đi.
unsigned long lastActivityTime = 0;  // Thời gian hoạt động cuối cùng

bool sleepStatusSent = false;  // Biến kiểm tra xem đã gửi trạng thái chưa
const unsigned long SEND_STATUS_TIMEOUT = 50000;  // 50 giây để gửi trạng thái
const unsigned long SLEEP_TIMEOUT = 60000;  // 60 giây không hoạt động thì ngủ

const char* ntpServer = "pool.ntp.org"; //Địa chỉ máy chủ NTP giúp ESP32 đồng bộ thời gian từ internet.
const long gmtOffset_sec = 7 * 3600; // Múi giờ GMT+7 (Việt Nam), tương ứng 7 giờ x 3600 giây.
const int daylightOffset_sec = 0; //Không sử dụng giờ mùa hè (Daylight Saving Time).

String getTimeStamp() { // struct tm là một cấu trúc dữ liệu chuẩn trong thư viện C (có sẵn trong <time.h>), thường được dùng để làm việc với thời gian
    struct tm timeinfo; // tạo biến để lưu thông tin thời gian 
    if (!getLocalTime(&timeinfo)) { // Gọi hàm getLocalTime() để lấy thời gian thực từ hệ thống đã đồng bộ với NTP server trước đó thông qua thư viện time.h 
        Serial.println("Lỗi lấy thời gian thực!");
        return "Unknown";
    }
    char timeString[25];
    strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo); // sử dụng hàm strftime để định dạng thời gian dạng chuỗi %Y-%m-%d %H:%M:%S
    return String(timeString);
}

volatile bool wakeUpFlag = false;  // Cờ đánh thức từ interrupt ( volatile được sử dụng với mục đích là biến này có thể được thay đổi )

void IRAM_ATTR wakeUpISR() {
    wakeUpFlag = true;  // Đặt cờ khi cảm biến 2 phát hiện chuyển động
}

void sendData(String cardID = "", String event = "", String name = "Unknown", String sleepStatus = "" ) {
    StaticJsonDocument<256> doc;  //Tạo một đối tượng JSON nhỏ gọn bằng thư viện ArduinoJson, dùng để chứa dữ liệu gửi đi.
    if (cardID != "") doc["cardID"] = cardID;  // từ dòng này trở xuống hiểu đơn giản là thêm dữ liệu vào các biến giá trị tránh dữ liệu rỗng 
    if (event != "") doc["event"] = event;     // Nếu cardID không rỗng, thì gán giá trị cardID vào trường "cardID" của doc.
    if (name != "Unknown") doc["name"] = name;
    if (sleepStatus != "") doc["sleepStatus"] = sleepStatus;  // Thêm sleepStatus

    doc["doorStatus"] = doorOpen ? "Open" : "Closed";
    doc["objectDetected"] = objectDetected ? "Yes" : "No";
    doc["timestamp"] = getTimeStamp();

    char buffer[256];  //Chuyển JSON thành chuỗi ký tự để truyền qua mạng.
    serializeJson(doc, buffer); //chuyển đổi (serialize) dữ liệu JSON trong đối tượng doc thành chuỗi JSON và ghi nó vào buffer.

    Serial.print("Gửi lên ThingsBoard: ");  //In ra chuỗi JSON để dễ theo dõi/log khi debug.
    Serial.println(buffer);
    client.publish("v1/devices/me/telemetry", buffer); //Gửi (publish) dữ liệu JSON trong buffer lên topic MQTT "v1/devices/me/telemetry".
} //"v1/devices/me/telemetry": Là tên topic MQTT, thường dùng trong nền tảng IoT như ThingsBoard

/// v1 nghĩa là version của nó, devices nghĩa là tên thiết bị, me nghĩa là đại diện cho chính thiết bị hiện tại, telemetry nghĩa là lấy dữ liệu từ cảm biến/trạng thái gửi về server

void openDoor() {
    Serial.println("Cửa đang mở...");
    myServo.write(90);
    doorOpen = true;
    sendData();
}

void closeDoor() {
    Serial.println("Cửa đóng lại.");
    myServo.write(0);
    doorOpen = false;
    sendData();
}

// nói cách khác hàm này chủ yếu để gửi thông tin nút nhấn trên server về ESP32 thực hiện đóng/mở cửa
void callback(char* topic, byte* payload, unsigned int length) {  // hàm callback là một hàm nhận và phản hồi MQTT với topic - chủ đề MQTT nhận được, payload - dữ liệu (JSON), length - độ dài chuỗi payload.
    Serial.print("Nhận dữ liệu từ RPC: "); //RPC = Remote Procedure Call (Gọi hàm từ xa), gửi lệnh từ server về thiết bị
    String message;
    for (unsigned int i = 0; i < length; i++) {  //Từng byte trong payload được ghép lại thành một chuỗi String đầy đủ để dễ xử lý.
        message += (char)payload[i];
    }
    Serial.println(message);  //Sau đó, in ra để kiểm tra/log nội dung đã nhận.

    StaticJsonDocument<200> doc;  //Tạo một đối tượng JSON nhỏ gọn bằng thư viện ArduinoJson, dùng để chứa dữ liệu gửi đi.
    DeserializationError error = deserializeJson(doc, message); //Giải mã (phân tích cú pháp) một chuỗi JSON trong message và lưu kết quả vào doc. Nếu có lỗi xảy ra, nó sẽ được lưu trong biến error.
    if (error) {
        Serial.println("Lỗi đọc JSON từ RPC");  // Nếu có lỗi (ví dụ JSON không hợp lệ), hàm sẽ thoát ra và in thông báo lỗi.
        return;
    }

    String method = doc["method"];
    if (method == "setState") {  //Kiểm tra lệnh điều khiển (method) có phải là "setState" hay không.
        bool state = doc["params"];  //Nếu có, đọc giá trị params (true/false) – đại diện cho lệnh mở hoặc đóng cửa từ xa.
        manualOverride = state;  //Gán manualOverride = state để lưu trạng thái hiện tại đang điều khiển thủ công.
        if (manualOverride) openDoor();  // Gọi openDoor() nếu true, 
        else closeDoor(); // hoặc closeDoor() nếu false.
    }

    if (method == "setReset") // tương tự ở trên nhưng là của nút reset lượt quét
        {
            bool state2 = doc["params"]; // Nếu có, đọc giá trị params ( nghĩa là reset cho thẻ hết hạn )
            reset = state2; // gán giá trị mới cho state2 ( reset về số lượt ban đầu )
        }

}

void reconnectMQTT() {  //đóng vai trò duy trì kết nối ổn định giữa thiết bị ESP32 và MQTT Broker ( thingboard )
    while (!client.connected()) {  //Kiểm tra xem client MQTT đã kết nối hay chưa.
        Serial.print("Đang kết nối lại MQTT...");
        if (client.connect("ESP32", accessToken, NULL)) {  // Gửi yêu cầu kết nối MQTT với thông tin: ESP32: tên client (client ID) và accessToken: mã định danh thiết bị trong ThingsBoard, NULL: không có mật khẩu
            Serial.println(" Kết nối thành công!");
            client.subscribe("v1/devices/me/rpc/request/+"); // Đăng ký chủ đề (topic) để nhận lệnh RPC từ ThingsBoard. 
        } else {                                             //Cụ thể, "v1/devices/me/rpc/request/+" là định dạng chuẩn cho RPC yêu cầu từ server gửi xuống.
            Serial.print(" Thất bại, mã lỗi=");              // v1 nghĩa là version của nó, devices nghĩa là tên thiết bị, me nghĩa là đại diện cho chính thiết bị hiện tại,
            Serial.println(client.state());                  // rpc là viết tắt của Remote Procedure Call-lệnh điều khiển từ server, request là lệnh yêu cầu gửi từ server xuống, dấu "+" là đại diện cho một số nào đó sau request
            delay(5000);  //Tạm dừng 5 giây trước khi thử lại, tránh làm nghẽn mạng hoặc CPU quá tải.
        }
    }
}

void sendScanCountToThingsBoard(String uidStr, int scanCount) { // hàm gửi dữ liệu lên thingsboard gồm mã UID và số lượt quét
  if (!client.connected()) return; // Nếu client MQTT không được kết nối, thoát khỏi hàm luôn

  // 4 dòng này là tạo một cái chuỗi json để gửi có dạng { "uid": "abc123", "scan_count": 5}
  String payload = "{";
  payload += "\"uid\":\"" + uidStr + "\",";
  payload += "\"scan_count\":" + String(scanCount);
  payload += "}";

  client.publish("v1/devices/me/telemetry", payload.c_str()); // Gửi dữ liệu JSON đó tới ThingsBoard MQTT với topic là v1/devices/me/telemetry.
  Serial.println("Đã gửi: " + payload);   //payload.c_str() dùng để chuyển từ kiểu String thành const char* vì phương thức publish yêu cầu kiểu đó
}

void checkWiFiConnection() { // hàm kiểm tra kết nối wifi, tạo ra wifi của chính nó từ ban đầu, sau đó chọn wifi
    if (WiFi.status() != WL_CONNECTED) { //Nếu ESP32 không còn kết nối WiFi (WL_CONNECTED), sẽ in cảnh báo và tiến hành xử lý kết nối lại.
        Serial.println("Mất kết nối WiFi. Đang thử kết nối lại..."); //WL_CONNECTED là trạng thái đã kết nối wifi được định nghĩa trong thư viện Wifi.h
        
        WiFiManager wifiManager;  //Khởi tạo một đối tượng từ thư viện WiFiManager, giúp thiết bị kết nối lại mạng WiFi dễ dàng mà không cần thao tác thủ công.
        // wifiManager.setEnableConfigPortal(false);  // Không tự động mở config portal
        wifiManager.autoConnect(AP_NAME, AP_PASSWORD);  // Thử kết nối với mạng WiFi đã lưu trước đó.
        // Thử kết nối lại với thông tin đã lưu, nếu không thành công thì sẽ tạo điểm truy cập (AP) với tên ESP32-CarPark và mật khẩu 12345678
        if (!wifiManager.autoConnect()) {
            Serial.println("Không thể kết nối lại!");
            delay(5000);
            return;
        }
        
        Serial.println("Đã kết nối lại WiFi thành công!"); //In thông báo kết nối thành công và địa chỉ IP để dễ quản lý hoặc truy cập thiết bị qua mạng nội bộ.
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    }
}

String getNameFromSheet(String cardID, String status) {  //hàm cho phép ESP32 truy xuất dữ liệu từ Google Sheets thông qua Web API.
    HTTPClient http;  // xử dụng thư viện HTTPClient gọi ra http để truy cập đường dẫn bên dưới
    String url = "https://script.google.com/macros/s/AKfycbzlpaWFebRCdD10wKAYFgA1Q7X8KW3qJuiDAqJAuHANw-7qpPnLeNPlUo4BcsmYEgmr/exec?cardID=" + cardID + "&status=" + status; // mã ID và trạng thái vào/ra

    Serial.print("📡 Gửi request API: ");
    Serial.println(url);

    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS); // Khởi tạo kết nối HTTP đến URL.
    http.begin(url);
    int httpResponseCode = http.GET(); // Gửi yêu cầu "GET" tới API Google Apps Script để lấy dữ liệu

    Serial.print("🔄 Mã phản hồi HTTP: ");
    Serial.println(httpResponseCode);

    if (httpResponseCode == 200) { //Nếu phản hồi là 200 (OK) thì mới xử lý tiếp → nghĩa là server phản hồi thành công.
        String payload = http.getString(); // Nhận nội dung JSON trả về từ API (ví dụ: { "name": "Nguyen Van A" })
        Serial.print("📩 Dữ liệu API trả về: ");
        Serial.println(payload);

        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload); //Dùng ArduinoJson để chuyển payload JSON thành đối tượng doc.
        if (error) {
            Serial.print("❌ Lỗi JSON: ");  // Nếu có lỗi JSON hoặc kết nối thất bại: trả về Unknown 
            Serial.println(error.f_str());
            http.end();
            return "Unknown";
        }

        String name = doc["name"].as<String>(); //Nếu thành công trích xuất giá trị của trường "name" từ tài liệu JSON → đây là tên người tương ứng với thẻ RFID.
        Serial.print("✅ Tên lấy được: ");
        Serial.println(name);

        http.end();
        return name;
    }

    Serial.println("⚠️ Không kết nối được API!"); // Nếu không kết nối được API thì trả về Unknown để xử lý tiếp trong chương trình chính.
    http.end();
    return "Unknown";
}

void beep() {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
}

// Hàm lưu carStatus vào SPIFFS   [SPIFFS (SPI Flash File System) là hệ thống file dùng để lưu trữ dữ liệu tạm hoặc cấu hình trên ESP32.]
void saveCarStatus() {
    File file = SPIFFS.open("/carStatus.json", "w"); //Dùng SPIFFS.open() để mở tệp tin JSON trong bộ nhớ nội với chế độ  viết (w).
    if (!file) {  //Nếu không mở được → in lỗi và kết thúc hàm.
        Serial.println("Lỗi mở tệp để ghi!");
        return;
    } // auto const& pair : carStatus là duyệt qua từng cặp (key, value), pair first là ID còn pair second là trạng thái
    StaticJsonDocument<1024> doc; // Tạo một StaticJsonDocument có kích thước 1024 byte để chứa dữ liệu, Tăng kích thước nếu có nhiều thẻ
    for (auto const& pair : carStatus) { //Duyệt qua từng phần tử trong carStatus
        doc[pair.first] = pair.second; // Lưu cardID và trạng thái (true = IN, false = OUT)
    }
    serializeJson(doc, file); //Chuyển đổi tài liệu JSON thành chuỗi và ghi vào file mở sẵn.
    file.close();  //Đóng file lại để đảm bảo dữ liệu được ghi hoàn tất.
    Serial.println("Đã lưu carStatus vào SPIFFS"); // In thông báo thành công.
}

// Hàm lưu scanMap vào SPIFFS
void saveScanMap() {
  File file = SPIFFS.open("/scan_map.txt", FILE_WRITE); // Mở file tên scan_map.txt trong chế độ ghi (FILE_WRITE) trong hệ thống file SPIFFS
  if (file) {  // kiểm tra mở file thành công không
    for (auto const& entry : scanMap) { // duyệt qua từng phần tử trong scanMap
      file.println(entry.first + "," + String(entry.second)); // mỗi dòng trong file có dạng là entry.first: UID (chuỗi), entry.second: số lượt quét tương ứng (số nguyên)
    }
    file.close();
  } else {
    Serial.println("Lỗi khi lưu scanMap!");
  }
}

// Hàm khôi phục carStatus từ SPIFFS
void loadCarStatus() {
    File file = SPIFFS.open("/carStatus.json", "r");  //Mở file /carStatus.json ở chế độ đọc (r).
    if (!file) { ////Nếu không mở được → in lỗi và kết thúc hàm.
        Serial.println("Không tìm thấy tệp carStatus.json, khởi tạo mới.");
        return;
    }
    StaticJsonDocument<1024> doc; // Tạo một StaticJsonDocument có kích thước 1024 byte để chứa dữ liệu, Tăng kích thước nếu cần
    DeserializationError error = deserializeJson(doc, file); //Dùng deserializeJson() để chuyển chuỗi JSON từ file thành đối tượng JSON.
    if (error) {  // Nếu quá trình phân tích lỗi → đóng file và kết thúc.
        Serial.println("Lỗi đọc JSON từ SPIFFS");
        file.close();
        return;
    }
    for (JsonPair pair : doc.as<JsonObject>()) {  //Duyệt từng phần tử trong đối tượng JSON.
        carStatus[pair.key().c_str()] = pair.value().as<bool>(); //Gán giá trị từng cardID và trạng thái (IN/OUT) trở lại carStatus.
    } //c_str() dùng để chuyển từ kiểu String thành const char* vì phương thức publish yêu cầu kiểu đó
    file.close(); // Đóng file sau khi đọc xong để giải phóng tài nguyên.
    Serial.println("Đã khôi phục carStatus từ SPIFFS"); // In thông báo đã nạp thành công.
}

// Hàm khôi phục scanMap từ SPIFFS
void loadScanMap() {
  File file = SPIFFS.open("/scan_map.txt", FILE_READ); //Mở file scan_map.txt từ bộ nhớ SPIFFS trong chế độ đọc
  if (file) { // kiểm tra file mở thành công không
    while (file.available()) {
      String line = file.readStringUntil('\n'); // Đọc từng dòng từ file cho đến khi hết dữ liệu
      line.trim(); // Dùng trim() để loại bỏ ký tự xuống dòng và khoảng trắng dư thừa
      if (line.length() == 0) continue; // Bỏ qua các dòng trống và tiếp tục
      int sepIndex = line.indexOf(','); // Tìm vị trí dấu phẩy , — dùng để tách UID và số lượt quét---- khai báo biến ghi nhớ vị trí dấu phẩy nếu tìm thấy tên là sepIndex
      if (sepIndex != -1) { // Kiểm tra nếu có dấu phẩy (để tránh lỗi nếu định dạng dòng không đúng)
        // Tách chuỗi với sepIndex là dấu phẩy
        String uid = line.substring(0, sepIndex); // uid: phần trước dấu phẩy ---- 0, sepIndex thực hiện lấy trước dấu phẩy
        int count = line.substring(sepIndex + 1).toInt(); // count: phần sau dấu phẩy, chuyển sang kiểu int ---- sepIndex + 1 thực hiện lấy sau dấu phẩy
        scanMap[uid] = count; // Gán vào scanMap với key là uid và value là count.
      }
    }
    file.close();
  } else {
    Serial.println("Không tìm thấy file scan_map.txt!");
  }
}

void setup() { // Hàm setup() là điểm khởi đầu của chương trình – nơi cấu hình tất cả các phần cứng và phần mềm cần thiết trước khi thiết bị bắt đầu chạy chính thức.
    Serial.begin(115200); //Bật cổng Serial để in log (debug).   ----   115200 là tốc độ phổ biến nhất và ổn định nhất
    SPI.begin(); // Khởi động giao tiếp SPI (dùng cho RFID).
    rfid.PCD_Init(); // Bắt đầu module RFID.
    myServo.attach(SERVO_PIN); //Gắn servo vào chân điều khiển để mở/đóng cửa.
    pinMode(IR_SENSOR_PIN, INPUT); // Thiết lập chân cảm biến hồng ngoại 1 làm đầu vào, dùng để xác nhận có vật thể qua cổng chưa
    pinMode(BUZZER_PIN, OUTPUT); // Thiết lập chân buzzer làm đầu ra.

    pinMode(IR_SENSOR_PIN_2, INPUT);  // Cấu hình chân cảm biến hồng ngoại 2 làm đầu vào, dùng để đánh thức ESP32
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN_2), wakeUpISR, FALLING);  // Gắn ngắt ngoài (interrupt) vào IR_SENSOR_PIN_2 → đánh thức thiết bị khi có chuyển động.
    // FALLING là kí hiệu của giảm từ HIGH về LOW
    digitalWrite(BUZZER_PIN, LOW); //Đảm bảo buzzer tắt ban đầu, tránh phát tiếng không mong muốn.

    // Khởi tạo SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Lỗi khởi tạo SPIFFS!");
        return;
    }
    loadCarStatus(); // Nếu thành công, gọi loadCarStatus() khôi phục trạng thái thẻ từ SPIFFS khi khởi động, để phục hồi thông tin các thẻ đã lưu.
    loadScanMap();
    WiFiManager wifiManager; //Tạo WiFiManager để tự động kết nối hoặc tạo điểm phát WiFi cấu hình.
    
    // Đặt timeout cho chế độ cấu hình (60 giây)
    // wifiManager.setConfigPortalTimeout(60);
    wifiManager.resetSettings(); //xóa cấu hình cũ khi debug hoặc cần kết nối mạng mới.
    
    
    // Tự động kết nối hoặc vào chế độ cấu hình
    if (!wifiManager.autoConnect(AP_NAME, AP_PASSWORD)) { // Nếu không kết nối được WiFi trong thời gian quy định → khởi động lại ESP32.
        Serial.println("Không thể kết nối và hết thời gian cấu hình!");
        delay(3000);
        ESP.restart();  // Khởi động lại nếu không kết nối được
    }
    
    Serial.println("\nWiFi đã kết nối!"); //In ra địa chỉ IP của thiết bị sau khi kết nối thành công để tiện debug/truy cập.
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //Thiết lập đồng bộ thời gian để lấy timestamp chính xác khi ghi log lên ThingsBoard.
    Serial.println("Đã đồng bộ thời gian thực!");

    client.setServer(thingsboardServer, 1883); //Cấu hình thông tin MQTT server (ThingsBoard).  ---  1883 là số cổng tiêu chuẩn dùng để kết nối đến MQTT server
    client.setCallback(callback); //Đặt hàm callback() để xử lý khi nhận lệnh từ server.
    reconnectMQTT(); // Gọi reconnectMQTT() để bắt đầu kết nối lại server MQTT.
}

void loop() { //Hàm loop() được gọi liên tục sau khi hàm setup() chạy xong, đóng vai trò là bộ điều khiển chính của hệ thống
    checkWiFiConnection();  // Kiểm tra kết nối WiFi, đảm bảo kết nối WiFi luôn sẵn sàng.
    reconnectMQTT();  //Nếu mất kết nối MQTT thì sẽ tự động reconnect.
    client.loop();  //client.loop() xử lý vòng lặp nhận dữ liệu RPC từ ThingsBoard. --- Phương thức client trong thư viện PubSubClient

    if (manualOverride) { //Nếu dashboard gửi lệnh mở/tắt cửa thủ công thì bỏ qua toàn bộ xử lý còn lại.
        lastActivityTime = millis();  // Cập nhật thời gian hoạt động khi điều khiển thủ công để tránh vào sleep.
        delay(500);
        return;
    }

    // Kiểm tra RFID
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) { //kiểm tra xem có thẻ RFID nào mới được đưa vào gần đầu đọc chưa, nếu có thẻ, hàm này sẽ đọc ID (số seri) của thẻ.
        String uidStr = ""; // Tạo một chuỗi rỗng để lưu UID của thẻ RFID dưới dạng chuỗi.
        for (byte i = 0; i < rfid.uid.size; i++) { // chạy qua từng byte trong rfid.uid.uidByte[], với rfid.uid.size là số byte trong UID
          uidStr += String(rfid.uid.uidByte[i], HEX); //Lấy từng byte trong rfid.uid.uidByte[] chuyển sang dạng hexadecimal (cơ số 16)
          if (i < rfid.uid.size - 1) uidStr += ":"; // Nếu không phải byte cuối cùng, thêm dấu : phân tách giữa các byte (trừ byte cuối cùng), Kết quả: uidStr sẽ là dạng "AB:CD:EF:12"
        } // giả sử UID có kí tự là {0xAB, 0xCD, 0xEF, 0x12} thì sau khi xử lý sẽ được là uidStr = "AB:CD:EF:12"
        uidStr.toUpperCase(); // dòng này nghĩa là giữ nguyên không thay đổi gì tới bản gốc

        // scanMap[uidStr]++;
        // sendScanCountToThingsBoard(uidStr, scanMap[uidStr]);
        lastActivityTime = millis();  // Cập nhật thời gian hoạt động khi quét thẻ, dùng để theo dõi thời gian hoạt động cuối cùng (cho chế độ tiết kiệm điện).
        String cardID = ""; //Lấy mã ID của thẻ
        for (byte i = 0; i < rfid.uid.size; i++) { //Lặp qua từng byte trong mã ID của thẻ.
            cardID += String(rfid.uid.uidByte[i] < 0x10 ? "0" : ""); //Nếu byte < 0x10 thì thêm số 0 phía trước để giữ đúng định dạng. ( 0x10 là cách viết số 16 ở hệ cơ số 16 (hexadecimal).), Để đảm bảo mỗi byte luôn có 2 ký tự
            cardID += String(rfid.uid.uidByte[i], HEX);// Chuyển byte sang hệ thập lục phân (HEX) và nối vào chuỗi cardID.
        }
        Serial.print("ID Thẻ: ");
        Serial.println(cardID);
        beep();

        String status = ""; //Nếu thẻ chưa từng quét trước đó, hoặc đang có trạng thái là false (tức là đã ra ngoài) → coi như xe đang vào.
        if (carStatus.find(cardID) == carStatus.end() || !carStatus[cardID]) { //Nếu carStatus[cardID] == true → xe đang ở trong → coi như đang ra.
            status = "in"; // Xe vào 
        } else {
            status = "out"; // Xe ra
        }

        String name = getNameFromSheet(cardID, status); //Gửi yêu cầu tới API Google Apps Script kèm theo ID thẻ và trạng thái.
        if (name == "Unknown" || name == "") { // Nếu không tìm thấy tên người dùng hợp lệ → báo "Không tìm thấy".
            name = "Không tìm thấy";
        }
        Serial.print("Tên: "); // Nếu tìm thấy thì in ra tên
        Serial.println(name);

        sendData(cardID, "Quẹt thẻ", name); //Gửi dữ liệu lên ThingsBoard với loại hành động là "Quẹt thẻ" và thông tin người dùng tương ứng

        if (name == "Không tìm thấy") { // Nếu tên là không tìm thấy
            Serial.println("🚫 Thẻ không hợp lệ - KHÔNG mở cửa!"); // in ra serial monitor
            for (int i = 0; i < 3; i++) { // kêu lên 3 tiếng bíp để cảnh báo
                beep();
                delay(100);
            }
            rfid.PICC_HaltA(); //Dừng giao tiếp với thẻ (HaltA, StopCrypto1).
            rfid.PCD_StopCrypto1();
            return; //thoát khỏi đoạn xử lý.
        }

        // Đoạn code kiểm tra xem có ấn reset từ thingsboard không
        if (reset) { // nếu kiểm tra là reset ( ấn nút để reset)
            scanMap[uidStr] = 0; // Đặt lại số lượt quét tương ứng với UID uidStr về 0 trong scanMap.
            delay(500); // delay tránh tình trạng nhầm 2 quét liên tục
            saveScanMap(); //  lưu lại toàn bộ scanMap xuống file scan_map.txt trong SPIFFS
            sendScanCountToThingsBoard(uidStr, scanMap[uidStr]); // Gửi dữ liệu mới (với scan_count = 0) lên ThingsBoard thông qua MQTT, để cập nhật trạng thái UID vừa được reset
            return; // kết thúc hàm
        }

        // Xử lí sự kiện xe vào/xe ra để gửi sự kiên lên dashboard thông qua sendData, bên cạnh đó in ra serial monitor
        if (status == "in") {
            Serial.println("🚗 Xe vào");
            carStatus[cardID] = true;
            sendData(cardID, "Xe vào", name);
            scanMap[uidStr]++;
        } else {
            Serial.println("🚙 Xe ra");
            carStatus[cardID] = false;
            sendData(cardID, "Xe ra", name);
            
            
        }
        sendScanCountToThingsBoard(uidStr, scanMap[uidStr]);
        saveCarStatus();  // Ghi trạng thái vào SPIFFS
        saveScanMap();
        // openDoor();
        // lastObjectTime = millis(); // Lưu thời gian vật thể xuất hiện
        // rfid.PICC_HaltA(); ////Dừng giao tiếp với thẻ hiện tại để chuẩn bị đọc thẻ mới(HaltA, StopCrypto1)
        // rfid.PCD_StopCrypto1();
        if ( scanMap[uidStr] >= 3) // nếu lượt quét lớn hơn bằng 3
        {
            outdate = 1; // cập nhật outdate lên bằng 1, nghĩa là hết lượt quét
            rfid.PICC_HaltA(); ////Dừng giao tiếp với thẻ hiện tại để chuẩn bị đọc thẻ mới(HaltA, StopCrypto1)
            rfid.PCD_StopCrypto1();
            for (int i = 0; i < 5; i++) { // kêu lên 5 tiếng bíp để cảnh báo
                beep();
                delay(100);
            }
            
        } else {
            outdate = 0; // gắn outdate bằng 0 nghĩa là còn lượt quét
            openDoor();
            lastObjectTime = millis(); // Lưu thời gian vật thể xuất hiện
            rfid.PICC_HaltA(); ////Dừng giao tiếp với thẻ hiện tại để chuẩn bị đọc thẻ mới(HaltA, StopCrypto1)
            rfid.PCD_StopCrypto1();
        }
    }

    if (outdate == 0){
        // Kiểm tra cảm biến hồng ngoại 1
        bool newObjectDetected = (digitalRead(IR_SENSOR_PIN) == LOW); // Nếu LOW (0) → có vật chắn tia (tức là có vật thể, Kết quả lưu vào biến newObjectDetected.
        if (newObjectDetected != objectDetected) { //So sánh trạng thái hiện tại (newObjectDetected) với trạng thái cũ (objectDetected), Nếu khác nhau → tức là trạng thái đã thay đổi (vật thể vừa xuất hiện hoặc vừa rời đi).
            lastActivityTime = millis();  // Ghi nhận thời điểm vừa xảy ra hoạt động (cho việc quản lý sleep mode sau này).
            objectDetected = newObjectDetected; //Cập nhật lại trạng thái vật thể hiện tại.
            sendData();  // Gửi trạng thái cảm biến lên dashboard

            if (objectDetected) { //Nếu vừa phát hiện có vật thể
                lastObjectTime = millis(); //Lưu thời điểm đó vào lastObjectTime.
                objectGoneTime = 0; //Reset biến objectGoneTime về 0.
            } else { //Nếu vật thể vừa rời khỏi cảm biến:
                objectGoneTime = millis(); //Ghi lại thời gian rời đi vào objectGoneTime.
            }
        }

        // Logic điều khiển cửa
        if (objectDetected && doorOpen) {
            Serial.println("Giữ cửa mở do có vật thể!");
            lastObjectTime = millis();
        }
        // Cửa đang mở, Không còn vật thể (!objectDetected), Đã từng có thời điểm vật thể biến mất (objectGoneTime > 0), Và đã hơn 2 giây kể từ thời điểm đó
        if (doorOpen && !objectDetected && objectGoneTime > 0 && millis() - objectGoneTime > 2000) {
            closeDoor();
            objectGoneTime = 0;
        }

        if (doorOpen && millis() - lastObjectTime > 7000) {
            closeDoor();
        }
    }

    

    unsigned long inactiveTime = millis() - lastActivityTime; // Tính thời gian không hoạt động (tính bằng ms) kể từ lần cuối có sự kiện như: quẹt thẻ, cảm biến phát hiện...
    if (inactiveTime > SEND_STATUS_TIMEOUT && !sleepStatusSent) { //Nếu đã hơn 50 giây và chưa gửi thông báo "sleeping" thì thực hiện gửi.
        Serial.println("Gửi trạng thái Sleeping lên ThingsBoard ở giây 50...");
        if (!client.connected()) { //// Kết nối lại MQTT trước khi gửi dữ liệu, Đảm bảo MQTT vẫn được kết nối trước khi gửi dữ liệu lên ThingsBoard.
            reconnectMQTT();
        }
        sendData("", "", "", "Sleeping"); //// Gửi dữ liệu temp, hum, gas, status lên MQTT nhưng vì không có 3 cái đầu nên chỉ gửi Sleeping thôi
        delay(500);  // Chờ gửi xong
        sleepStatusSent = true; // true nghĩa là chuẩn bị vào chế độ ngủ, Đánh dấu đã gửi rồi, không gửi lại nữa.
    }

    if (inactiveTime > SLEEP_TIMEOUT) { // Nếu đã 60s sau khi không có sự kiện thì vào chế độ ngủ Light Sleep.
        Serial.println("Đã đến giây 60, vào Light Sleep..."); // In ra log và đảm bảo mọi thứ được in hết ra cổng Serial.
        Serial.flush(); // Đợi cho đến khi dữ liệu đang gửi được gửi hết
        esp_sleep_enable_ext0_wakeup((gpio_num_t)IR_SENSOR_PIN_2, 0); //Cấu hình cảm biến hồng ngoại 2 làm nguồn đánh thức (wake-up source).
        esp_light_sleep_start(); // Gọi esp_light_sleep_start() để vào chế độ ngủ nhẹ (các chức năng chính tạm ngừng, tiết kiệm pin).

        // Serial.println("Đã tỉnh dậy từ Light Sleep"); // Khi được đánh thức bởi cảm biến IR 2 → in log xác nhận đã tỉnh lại.
        // Serial.flush();

        // // Gửi Awake sau khi kết nối thành công
        // sendData("", "", "", "Awake"); //Gửi thông báo "Awake" lên ThingsBoard để người dùng biết thiết bị đã hoạt động trở lại.
        // delay(500);  // Chờ gửi xong

        // if (wakeUpFlag) { // Nếu được đánh thức bởi cảm biến 2
        //     Serial.println("Được đánh thức bởi cảm biến 2!");
        //     wakeUpFlag = false;
        //     lastActivityTime = millis(); //Cập nhật lại thời gian hoạt động.
        //     sleepStatusSent = false; // false nghĩa là đã thoát chế độ ngủ, cho phép gửi "Sleeping" lại ở vòng lặp sau nếu tiếp tục không có hoạt động.
        // }

        // // Kết nối lại MQTT trước khi gửi dữ liệu, Đảm bảo MQTT vẫn được kết nối trước khi gửi dữ liệu lên ThingsBoard.
        // if (!client.connected()) {
        //     reconnectMQTT();
        // }

        // // Gửi Awake sau khi kết nối thành công
        // sendData("", "", "", "Awake");
        // delay(500);  // Chờ gửi xong

        if (wakeUpFlag) { // Nếu được đánh thức bởi cảm biến 2
            Serial.println("Được đánh thức bởi cảm biến 2!");
            wakeUpFlag = false;
            lastActivityTime = millis(); //Cập nhật lại thời gian hoạt động.
            sleepStatusSent = false; // false nghĩa là đã thoát chế độ ngủ, cho phép gửi "Sleeping" lại ở vòng lặp sau nếu tiếp tục không có hoạt động.
        }

        // Kết nối lại MQTT trước khi gửi dữ liệu, Đảm bảo MQTT vẫn được kết nối trước khi gửi dữ liệu lên ThingsBoard.
        if (!client.connected()) {
            reconnectMQTT();
        }

        // Gửi Awake sau khi kết nối thành công
        sendData("", "", "", "Awake");
        delay(500);  // Chờ gửi xong
        Serial.println("Đã tỉnh dậy từ Light Sleep"); // Khi được đánh thức bởi cảm biến IR 2 → in log xác nhận đã tỉnh lại.
        Serial.flush();

    }
    // Đoạn này giúp đảm bảo chỉ chạy "mỗi hành động" sau mỗi 500ms, tránh lặp quá nhanh gây quá tải CPU.
    unsigned long currentTime = millis(); //Lấy thời gian hiện tại (tính bằng mili-giây kể từ khi thiết bị khởi động)
    static unsigned long lastLoopTime = 0; // static giúp biến này chỉ được khởi tạo 1 lần duy nhất và giữ nguyên giá trị, Được khởi tạo ban đầu và nhớ lần cuối cùng hàm này được thực thi.
    if (currentTime - lastLoopTime >= 500) { // So sánh nếu đủ 500ms rồi, thì cho phép chạy phần bên trong if.
        lastLoopTime = currentTime; //Cập nhật mốc thời gian gần nhất đã xử lý, Lần sau sẽ lại so sánh từ thời điểm này.
    }
}