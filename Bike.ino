#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_ADXL345_U.h>
#include <Firebase_ESP_Client.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

BluetoothSerial SerialBT;
uint8_t lockAddress[] = {0x04, 0x83, 0x08, 0x73, 0x66, 0x82};

#define LED_PIN 26
#define BUZZER_PIN 25      // First active buzzer
#define BUZZER2_PIN 27     // Second active buzzer (new)

// ---------- Configuration Parameters ----------
float threshold = 2.0;               // Motion threshold (m/s^2)
unsigned long sampleInterval = 500;  // Sampling interval (ms)
unsigned long monitorWindow = 3000;  // Motion detection window (ms)
int requiredTriggers = 3;            // Number of motion detections required within the window
unsigned long alarmDuration = 5000;  // Alarm duration (ms)

// ---------- State Variables ----------
unsigned long lastSampleTime = 0;
unsigned long windowStartTime = 0;
int motionCount = 0;
bool alarmActive = false;
unsigned long alarmStartTime = 0;

// ---------- New Upload Control ----------
bool isCut = false;                         // Lock is cut
unsigned long lastUploadTime = 0;
const unsigned long UPLOAD_INTERVAL = 5000; // Upload every 5 seconds

// ---------- WiFi Configuration ----------
#define WIFI_SSID "Jerry"
#define WIFI_PASSWORD "12345678"

// ----------- Firebase Configuration --------
#define API_KEY "AIzaSyAoLKK-3sx16nk85eMYeuPHZNJO3pBJM08"
#define DATABASE_URL "https://bike-anti-theft-be6a1-default-rtdb.firebaseio.com"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// ---------- Function Declarations ----------
void calibrateADXL();
void triggerAlarm();
void resetAlarm();
void uploadToFirebase(float ax, float ay, float az, bool alarm);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // SDA, SCL pins for I2C

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUZZER2_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(BUZZER2_PIN, LOW);

  SerialBT.begin("BikeReceiver", true);
  Serial.println("Bluetooth initialized as master.");

  Serial.print("Connecting to BikeLock at ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", lockAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (SerialBT.connect(lockAddress)) {
    Serial.println("✅ Connected to BikeLock!");
  }
  else {
    Serial.println("❌ Failed to connect. Make sure BikeLock is powered on and in range.");
  }

  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ WiFi connected!");

  // Setup Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Optional but safer: set a unique client name
  config.token_status_callback = tokenStatusCallback;

  // You can use anonymous authentication (no login)
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("✅ Firebase sign-up successful (anonymous).");
  } else {
    Serial.printf("❌ Firebase sign-up failed: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  unsigned long currentTime = millis();

  if (SerialBT.available()) {
    String msg = SerialBT.readStringUntil('\n');
    msg.trim();
    Serial.print("📩 Bluetooth received: ");
    Serial.println(msg);

    if (msg.equalsIgnoreCase("Cable Cut Detected")) {
      Serial.println("⚠️ Lock reported cable cut! Triggering alarm.");
      triggerAlarm();
      isCut = true;
    }
    else if (msg.equalsIgnoreCase("Cable Reconnected")) {
      Serial.println("✅ Lock reconnected normally.");
      resetAlarm();
      isCut = false;
    }
  }
  
  // ---------- Keep alarm active for 5 seconds ----------
  if (alarmActive) {
    
  }

  // Upload every 5 seconds
  if (currentTime - lastUploadTime >= UPLOAD_INTERVAL) {
    lastUploadTime = currentTime;
    sensors_event_t event;
    accel.getEvent(&event);
    uploadToFirebase(event.acceleration.x, event.acceleration.y, event.acceleration.z, isCut);
  }
}


// =======================================================
// Upload Data to Firebase (every 5 seconds)
// =======================================================
void uploadToFirebase(float ax, float ay, float az, bool alarm) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ WiFi disconnected!");
    return;
  }

  // ---- Example data (replace with actual readings later) ----
  float battery = 82.0;     // e.g., from analogRead(batteryPin)
  float lat = 27.961;       // e.g., from GPS module
  float lon = -82.442;      // e.g., from GPS module
  unsigned long timestamp = millis() / 1000; // seconds since boot, or use actual RTC time

  FirebaseJson json;

  json.set("alarm", alarm);
  json.set("accel_x", ax);
  json.set("accel_y", ay);
  json.set("accel_z", az);
  json.set("battery", battery);
  json.set("lat", lat);
  json.set("lon", lon);
  json.set("timestamp", timestamp);

  // Upload the entire object to /bike_status
  if (Firebase.RTDB.setJSON(&fbdo, "bike_status", &json)) {
    Serial.println("✅ Data uploaded to Firebase as JSON object");
  } else {
    Serial.print("❌ Upload failed: ");
    Serial.println(fbdo.errorReason());
  }
}
