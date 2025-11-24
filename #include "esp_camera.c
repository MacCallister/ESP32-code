#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <time.h>
#include <UniversalTelegramBot.h>

// =============================================
// CONFIGURATION - UPDATE THESE VALUES
// =============================================
const char* ssid = "MacCallister";
const char* password = "Logician";

// Server URLs
const char* serverURL = "https://web-production-23072.up.railway.app/upload";
const char* serverTestURL = "https://web-production-23072.up.railway.app/test";

// Telegram Bot Configuration
#define BOTtoken "8260428040:AAHopZu53sdpM5-gPxa9nL2-Y2d7tsnOcRI"
#define CHAT_ID "5765390339"

// CAMERA ID - CHANGE THIS FOR EACH CAMERA
const char* CAMERA_ID = "CAM001";

// NTP Time Server Configuration
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 0;

// =============================================
// PIN DEFINITIONS
// =============================================
#define OLED_SDA_PIN 14
#define OLED_SCL_PIN 15
#define BUTTON_PIN 13

#define CAMERA_INIT_RETRIES 5
#define CAMERA_INIT_DELAY 2000

// =============================================
// OLED DISPLAY SETTINGS
// =============================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =============================================
// ESP32-CAM AI THINKER PIN DEFINITIONS
// =============================================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// =============================================
// SYSTEM VARIABLES
// =============================================
WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

bool buttonPressed = false;
bool lastButtonState = false;
unsigned long lastCaptureTime = 0;
unsigned long buttonPressStartTime = 0;
bool buttonHoldDetected = false;
const unsigned long CAPTURE_COOLDOWN = 3000;
const unsigned long CAPTURE_DELAY = 5000;
const unsigned long BUTTON_HOLD_TIME = 5000;
int captureCount = 0;
bool systemInitialized = false;
bool timeInitialized = false;
bool serverReachable = false;
bool systemError = false;
bool displayAvailable = false;

// =============================================
// FUNCTION DECLARATIONS
// =============================================
void displayMessage(String line1, String line2 = "", String line3 = "");
void initializePins();
bool initializeDisplay();
bool connectToWiFi();
void initializeTime();
bool testServerConnection();
void captureAndProcessImage();
bool uploadImageToServer(uint8_t* imageData, size_t imageLen, String timestamp);
bool sendPhotoToTelegram(camera_fb_t * fb);
void powerOffSystem();
void checkButtonForRestart();

// =============================================
// CAMERA INITIALIZATION (SAFER VERSION)
// - 10s delay BEFORE esp_camera_init()
// - uses fb_count = 1 when PSRAM present to reduce camera task pressure
// =============================================
bool initializeCamera() {
  Serial.println("Starting camera init sequence...");

  // Check PSRAM early
  if (!psramFound()) {
    Serial.println("❌ PSRAM not found! Camera needs PSRAM");
    return false;
  }
  Serial.printf("✓ PSRAM found: %u bytes\n", ESP.getPsramSize());

  // Power cycle camera before initialization
  if (PWDN_GPIO_NUM != -1) {
    pinMode(PWDN_GPIO_NUM, OUTPUT);
    digitalWrite(PWDN_GPIO_NUM, 1); // power down
    delay(500);
    digitalWrite(PWDN_GPIO_NUM, 0); // power up
    delay(500);
  }

  // --- IMPORTANT: wait BEFORE starting the camera driver ---
  Serial.println("⏳ Waiting 10 seconds BEFORE esp_camera_init() to reduce cam_task stack pressure...");
  if (displayAvailable) {
    displayMessage("CAMERA INIT", "Delaying 10s before init");
  }
  for (int i = 0; i < 100; ++i) {
    delay(100); // total 10s
    yield();    // let other tasks run
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 8000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // Use single framebuffer for safer memory footprint (reduces camera task pressure)
  if (psramFound()) {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 10;
    config.fb_count = 1; // reduced to 1 for stability
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Retry initialization
  esp_err_t err = ESP_FAIL;
  for (int attempt = 1; attempt <= CAMERA_INIT_RETRIES; attempt++) {
    Serial.printf("🎥 Camera init attempt %d/%d...\n", attempt, CAMERA_INIT_RETRIES);
    if (displayAvailable) displayMessage("CAMERA INIT", String("Attempt ") + String(attempt));

    err = esp_camera_init(&config);
    if (err == ESP_OK) {
      Serial.println("✓ Camera initialized successfully!");
      break;
    }

    Serial.printf("⚠️ Camera init failed (0x%x)\n", err);
    esp_camera_deinit();
    delay(CAMERA_INIT_DELAY);
  }

  if (err != ESP_OK) {
    Serial.println("❌ Camera initialization failed!");
    return false;
  }

  // Get sensor handle and apply settings
  sensor_t * s = esp_camera_sensor_get();
  if (s == NULL) {
    Serial.println("❌ ERROR: Could not get camera sensor handle");
    esp_camera_deinit();
    return false;
  }

  s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_quality(s, 10);
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 0);
  s->set_ae_level(s, 0);
  s->set_aec_value(s, 300);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);

  Serial.println("✓ Camera configured and ready");
  if (displayAvailable) displayMessage("CAMERA READY", "Initialized");

  return true;
}

// =============================================
// SETUP FUNCTION
// =============================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  Serial.println("\n\n=================================");
  Serial.println("ESP32-CAM License Plate Detector");
  Serial.println("Cloud + Telegram Version");
  Serial.printf("Camera ID: %s\n", CAMERA_ID);

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Status: WAKING FROM SLEEP");
  } else {
    Serial.println("Status: COLD BOOT");
  }

  Serial.println("=================================");

  initializePins();

  delay(200);
  displayAvailable = initializeDisplay();
  if (!displayAvailable) {
    Serial.println("⚠️ OLED initialization failed - continuing without display");
  }

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 && displayAvailable) {
    displayMessage("WAKING UP...", "System starting");
    delay(1500);
  }

  displayMessage("System Starting...", "Camera: " + String(CAMERA_ID), "Initializing...");

  if (!initializeCamera()) {
    Serial.println("❌ CAMERA ERROR! Entering recovery mode");
    systemError = true;
    displayMessage("CAMERA ERROR!", "Try:", "1. Check camera");
    delay(2000);
    displayMessage("CAMERA ERROR!", "2. Press button", "to restart");

    unsigned long errorStartTime = millis();
    while (millis() - errorStartTime < 10000) {
      checkButtonForRestart();
      delay(100);
    }

    Serial.println("🔄 Auto-restarting after camera error...");
    displayMessage("AUTO RESTART", "Please wait...");
    delay(2000);
    ESP.restart();
  }

  displayMessage("Camera Ready!", "Connecting WiFi...");

  if (connectToWiFi()) {
    clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT);

    initializeTime();

    Serial.println("\n🔍 Testing cloud server connection...");
    displayMessage("TESTING SERVER", "Please wait...");
    delay(1000);

    if (testServerConnection()) {
      serverReachable = true;
      systemInitialized = true;
      displayMessage("SYSTEM READY!", "Server: Online", "Press button");
      Serial.println("✅ System Fully Ready!");
      delay(2000);
    } else {
      serverReachable = false;
      systemInitialized = true;
      displayMessage("SYSTEM READY!", "Server: Offline", "Press button");
      Serial.println("✅ System Ready (Telegram only)");
      delay(2000);
    }
  } else {
    systemError = true;
    displayMessage("WiFi FAILED!", "Press to restart");
    Serial.println("❌ WiFi failed - waiting for restart");

    unsigned long wifiErrorTime = millis();
    while (millis() - wifiErrorTime < 10000) {
      checkButtonForRestart();
      delay(100);
    }

    Serial.println("🔄 Auto-restarting after WiFi error...");
    displayMessage("AUTO RESTART", "Please wait...");
    delay(2000);
    ESP.restart();
  }
}

// =============================================
// MAIN LOOP
// =============================================
void loop() {
  if (!systemInitialized) {
    delay(1000);
    return;
  }

  buttonPressed = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonPressed && !lastButtonState) {
    buttonPressStartTime = millis();
    buttonHoldDetected = false;
  }

  if (buttonPressed && !buttonHoldDetected) {
    unsigned long holdDuration = millis() - buttonPressStartTime;

    if (holdDuration >= 1000 && holdDuration < BUTTON_HOLD_TIME) {
      int secondsRemaining = (BUTTON_HOLD_TIME - holdDuration) / 1000 + 1;
      displayMessage("HOLD TO POWER OFF", String(secondsRemaining) + " seconds...");
    }

    if (holdDuration >= BUTTON_HOLD_TIME) {
      buttonHoldDetected = true;
      powerOffSystem();
    }
  }

  if (buttonPressed && !lastButtonState && !buttonHoldDetected) {
    Serial.println("🔘 Button pressed! Starting 5-second countdown...");
    displayMessage("BUTTON PRESSED", "Wait 5 seconds...");
    delay(200);

    if (millis() - lastCaptureTime > CAPTURE_COOLDOWN) {
      bool stillHeld = true;
      for (int i = 5; i > 0 && stillHeld; i--) {
        Serial.printf("⏱️ Capturing in %d seconds...\n", i);
        displayMessage("CAPTURING IN...", String(i) + " seconds");

        for (int j = 0; j < 10; j++) {
          delay(100);
          if (digitalRead(BUTTON_PIN) == HIGH) {
            stillHeld = false;
            break;
          }
        }
      }

      if (!stillHeld || digitalRead(BUTTON_PIN) == HIGH) {
        captureAndProcessImage();
        lastCaptureTime = millis();
      }
    } else {
      Serial.println("⏱️ Capture cooldown active, skipping...");
      displayMessage("COOLDOWN ACTIVE", "Skipping capture");
      delay(1000);
    }
  }

  if (!buttonPressed && lastButtonState && !buttonHoldDetected) {
    if (serverReachable) {
      displayMessage("SERVER REACHABLE", "Camera: " + String(CAMERA_ID), "Press button");
    } else {
      displayMessage("SERVER NOT", "REACHABLE", "Press button");
    }
    Serial.println("Button released");
  }

  lastButtonState = buttonPressed;
  delay(50);
}

// =============================================
// HARDWARE INITIALIZATION
// =============================================
void initializePins() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("✓ GPIO pins initialized (Button on GPIO 13)");
}

bool initializeDisplay() {
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN, 50000);
  Wire.setTimeOut(1000);
  delay(200);

  Wire.beginTransmission(SCREEN_ADDRESS);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.printf("❌ OLED not detected at 0x%X (Error: %d)\n", SCREEN_ADDRESS, error);
    return false;
  }

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("❌ SSD1306 OLED allocation failed");
    return false;
  }

  display.clearDisplay();
  display.display();
  Serial.println("✓ OLED display initialized");
  return true;
}

void checkButtonForRestart() {
  bool currentButtonState = (digitalRead(BUTTON_PIN) == LOW);

  if (currentButtonState && !lastButtonState) {
    Serial.println("🔄 Button pressed - Restarting ESP32...");
    displayMessage("RESTARTING...", "Please wait...");
    delay(1000);
    ESP.restart();
  }

  lastButtonState = currentButtonState;
}

bool connectToWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  displayMessage("CONNECTING WiFi", "Please wait...");
  Serial.print("Connecting to WiFi");

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts % 5 == 0) {
      displayMessage("CONNECTING WiFi", String("Attempt ") + String(attempts) + "/30");
    }
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("✓ WiFi connected successfully!");
    Serial.print("📡 IP address: ");
    Serial.println(WiFi.localIP());
    displayMessage("WiFi CONNECTED", WiFi.localIP().toString());
    delay(2000);
    return true;
  } else {
    Serial.println("❌ WiFi connection failed!");
    return false;
  }
}

void initializeTime() {
  Serial.println("🕐 Initializing time from NTP server...");
  displayMessage("SYNCING TIME", "Please wait...");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;
  int attempts = 0;
  while (!getLocalTime(&timeinfo) && attempts < 5) {
    Serial.print(".");
    delay(500);
    attempts++;
  }

  if (attempts < 5) {
    Serial.println("\n✓ Time synchronized!");
    Serial.println(&timeinfo, "Current time: %A, %B %d %Y %H:%M:%S");
    timeInitialized = true;

    char timeStr[10];
    strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
    displayMessage("TIME SYNCED", String(timeStr));
    delay(1000);
  } else {
    Serial.println("\n⚠️ Time sync timeout - using system time");
    timeInitialized = false;
    displayMessage("TIME: Local Mode");
    delay(1000);
  }
}

String getTimestamp() {
  if (timeInitialized) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      char timestamp[30];
      strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
      return String(timestamp);
    }
  }

  unsigned long seconds = millis() / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  return String("Uptime: ") + String(hours) + "h " + String(minutes % 60) + "m " + String(seconds % 60) + "s";
}

// =============================================
// SERVER CONNECTION TEST
// =============================================
bool testServerConnection() {
  HTTPClient http;

  Serial.println("\n========================");
  Serial.println("🔍 TESTING CLOUD SERVER");
  Serial.println("========================");
  Serial.printf("📍 Target: %s\n", serverTestURL);

  http.begin(serverTestURL);
  http.setTimeout(8000);
  http.setConnectTimeout(5000);

  int httpCode = http.GET();

  if (httpCode > 0) {
    Serial.printf("📡 Response code: %d\n", httpCode);
  } else {
    Serial.printf("❌ Error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();

  if (httpCode > 0 && httpCode < 400) {
    Serial.println("✅ Cloud server is REACHABLE!");
    return true;
  } else {
    Serial.println("❌ Server not reachable - will use Telegram only");
    return false;
  }
}

// =============================================
// IMAGE CAPTURE AND PROCESSING
// =============================================
void captureAndProcessImage() {
  Serial.println("📸 Starting image capture process...");

  camera_fb_t * fb = esp_camera_fb_get();
  if(fb) {
    esp_camera_fb_return(fb);
    Serial.println("  Cleared old frame");
  }
  delay(200);

  fb = NULL;
  for(int i = 0; i < 3; i++) {
    fb = esp_camera_fb_get();
    if(fb && fb->len > 0) {
      Serial.printf("✓ Capture successful on attempt %d\n", i+1);
      break;
    }
    if(fb) esp_camera_fb_return(fb);
    Serial.printf("⚠️ Capture attempt %d failed, retrying...\n", i+1);
    delay(200);
  }

  if (!fb || fb->len == 0) {
    Serial.println("❌ Camera capture failed after 3 attempts!");
    displayMessage("CAPTURE FAILED", "Camera error", "Press to restart");
    systemError = true;
    delay(2000);

    while(1) {
      checkButtonForRestart();
      delay(100);
    }
    return;
  }

  captureCount++;
  String timestamp = getTimestamp();

  Serial.printf("✓ Image captured! Size: %d bytes (%d KB)\n", fb->len, fb->len / 1024);
  Serial.printf("  Resolution: %dx%d\n", fb->width, fb->height);
  Serial.printf("  Format: %d\n", fb->format);
  Serial.printf("📷 Camera ID: %s\n", CAMERA_ID);
  Serial.printf("🕐 Timestamp: %s\n", timestamp.c_str());

  bool uploadSuccess = false;
  if (WiFi.status() == WL_CONNECTED && serverReachable) {
    displayMessage("UPLOADING SERVER", String(fb->len / 1024) + " KB", "Please wait...");
  
    if (uploadSuccess) {
      Serial.println("✅ Server upload successful!");
      displayMessage("SERVER: SUCCESS", "Sending Telegram...");
      delay(1000);
    } else {
      Serial.println("❌ Server upload failed");
      displayMessage("SERVER: FAILED", "Sending Telegram...");
      serverReachable = false;
      delay(1000);
    }
  }

  displayMessage("SENDING TELEGRAM", "Please wait...");
  bool telegramSuccess = sendPhotoToTelegram(fb);

  if (telegramSuccess) {
    Serial.println("✅ Telegram photo sent successfully!");
    displayMessage("TELEGRAM: SUCCESS", "Plate #" + String(captureCount), "Complete!");
  } else {
    Serial.println("❌ Telegram send failed");
    displayMessage("TELEGRAM: FAILED", "Check bot settings");
  }

  esp_camera_fb_return(fb);
  fb = NULL;

  Serial.println("✓ Memory freed, ready for next capture");
  delay(2000);
}

// =============================================
// SERVER UPLOAD
// =============================================
bool uploadImageToServer(uint8_t *imageData, size_t imageLen, const String &timestamp) {
  Serial.println("🌐 Uploading image to server...");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi not connected!");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();  // disable SSL verification for simplicity
  client.setTimeout(15000);  // 15s network timeout

  if (!client.connect("web-production-23072.up.railway.app", 443)) {
    Serial.println("❌ Connection failed");
    return false;
  }

  String boundary = "----ESP32Boundary";
  String startReq =
    "POST /test HTTP/1.1\r\n"
    "Host: web-production-23072.up.railway.app\r\n"
    "User-Agent: ESP32CAM\r\n"
    "Content-Type: multipart/form-data; boundary=" + boundary + "\r\n"
    "Connection: close\r\n"
    "Content-Length: " + String(
        String("--" + boundary + "\r\n").length() +
        String("Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n").length() +
        String("Content-Type: image/jpeg\r\n\r\n").length() +
        imageLen +
        String("\r\n--" + boundary + "--\r\n").length()
      ) + "\r\n\r\n";

  client.print(startReq);
  client.print("--" + boundary + "\r\n");
  client.print("Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n");
  client.print("Content-Type: image/jpeg\r\n\r\n");

  // stream in small chunks to avoid heap exhaustion
  const size_t CHUNK = 2048;
  for (size_t i = 0; i < imageLen; i += CHUNK) {
    size_t len = (i + CHUNK < imageLen) ? CHUNK : (imageLen - i);
    client.write(imageData + i, len);
    delay(5); // yield between chunks
  }

  client.print("\r\n--" + boundary + "--\r\n");

  Serial.println("🕓 Waiting for server response...");
  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 15000) {
    String line = client.readStringUntil('\n');
    if (line.startsWith("HTTP/1.1")) Serial.println(line);
    if (line == "\r") break;
  }

  String response = client.readString();
  Serial.println("📡 Server reply: " + response);
  client.stop();

  if (response.indexOf("200") != -1 || response.indexOf("OK") != -1)
    return true;

  return false;
}

// =============================================
// TELEGRAM UPLOAD
// =============================================
bool sendPhotoToTelegram(camera_fb_t * fb) {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  Serial.println("📤 Connecting to Telegram...");

  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("✓ Connected to Telegram");

    String head = "--ESP32CAM\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + String(CHAT_ID) + "\r\n--ESP32CAM\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"" + String(CAMERA_ID) + ".jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--ESP32CAM--\r\n";

    size_t imageLen = fb->len;
    size_t extraLen = head.length() + tail.length();
    size_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + String(BOTtoken) + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=ESP32CAM");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;

    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      } else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    int waitTime = 10000;
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length()==0) state=true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    Serial.println();
    clientTCP.stop();
    Serial.println(getBody);
    return true;
  } else {
    Serial.println("❌ Failed to connect to Telegram");
    return false;
  }
}

// =============================================
// POWER MANAGEMENT
// =============================================
void powerOffSystem() {
  Serial.println("\n🔴 POWERING OFF SYSTEM...");
  displayMessage("POWERING OFF...", "Goodbye!");
  delay(2000);

  if (displayAvailable) {
    display.clearDisplay();
    display.display();
  }

  esp_camera_deinit();

  Serial.println("💤 Entering deep sleep mode");
  delay(100);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);
  esp_deep_sleep_start();
}

// =============================================
// DISPLAY FUNCTIONS
// =============================================
void displayMessage(String line1, String line2, String line3) {
  if (!displayAvailable) return;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("License Plate");
  display.println("Detector");
  display.println("================");
  display.println(line1);
  if (line2.length() > 0) {
    display.println(line2);
  }
  if (line3.length() > 0) {
    display.println(line3);
  }
  display.display();
}