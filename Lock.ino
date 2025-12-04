#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_ADXL345_U.h>
#include <Firebase_ESP_Client.h>
#include <Arduino.h>
#include <BluetoothSerial.h>
#include <TinyGPSPlus.h>
#include <esp_wpa2.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

BluetoothSerial SerialBT;
uint8_t lockAddress[] = {0x04, 0x83, 0x08, 0x73, 0x66, 0x82};

#define LED_PIN 26
#define BUZZER_PIN 25      // First active buzzer

// Create ADXL345 object
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ---------- Configuration Parameters ----------
float threshold = 1.0;               // Motion threshold (m/s^2)
unsigned long sampleInterval = 500;  // Sampling interval (ms)
unsigned long monitorWindow = 3000;  // Motion detection window (ms)
int requiredTriggers = 2;            // Number of motion detections required within the window
unsigned long alarmDuration = 5000;  // Alarm duration (ms)

// ---------- Calibration Parameters ----------
float baseAcc = 9.8;   // Expected gravity acceleration (m/s^2)
float offsetAcc = 0.0; // Calibration offset (computed automatically)

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
#define WIFI_SSID "UFDevice"
#define EAP_IDENTITY "lijunhao"
#define EAP_USERNAME "lijunhao"
#define EAP_PASSWORD "3165JHLi38714=="

// ----------- Firebase Configuration --------
#define API_KEY "AIzaSyAoLKK-3sx16nk85eMYeuPHZNJO3pBJM08"
#define DATABASE_URL "https://bike-anti-theft-be6a1-default-rtdb.firebaseio.com"

// ------------ GPS --------------
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

// ----------- Firebase -----------------
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

  // ----------- GPS INITIALIZATION -----------
  GPS_Serial.begin(9600, SERIAL_8N1, 4, 5);  // RX = 4, TX = 5
  Serial.println("GPS initialized on pins 4 (RX) and 5 (TX)");
  
  Wire.begin(21, 22); // SDA, SCL pins for I2C

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  SerialBT.begin("BikeReceiver", true);
  Serial.println("Bluetooth initialized as master.");

  Serial.print("Connecting to BikeLock at ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", lockAddress[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (SerialBT.connect(lockAddress)) {
    Serial.println("Connected to BikeLock!");
  }
  else {
    Serial.println("Failed to connect. Make sure BikeLock is powered on and in range.");
  }

  if (!accel.begin()) {
    Serial.println("ADXL345 not detected! Please check wiring.");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);
  Serial.println("ADXL345 initialized successfully.");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);

  // WPA2 Enterprise config
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));

  esp_wpa2_config_t configWPA = WPA2_CONFIG_INIT_DEFAULT();
  esp_wifi_sta_wpa2_ent_enable(&configWPA);

  WiFi.begin(WIFI_SSID);

  Serial.print("Connecting to UFDevice");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected to UFDevice!");
  Serial.println(WiFi.localIP());

  // Setup Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  // Optional but safer: set a unique client name
  config.token_status_callback = tokenStatusCallback;

  // Use anonymous authentication
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Firebase sign-up successful (anonymous).");
  } else {
    Serial.printf("Firebase sign-up failed: %s\n", config.signer.signupError.message.c_str());
  }

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // Perform Initial Calibration
  calibrateADXL();
}

void loop() {
  unsigned long currentTime = millis();

  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  if (SerialBT.available()) {
    String msg = SerialBT.readStringUntil('\n');
    msg.trim();
    Serial.print("Bluetooth received: ");
    Serial.println(msg);

    if (msg.equalsIgnoreCase("Cable Cut Detected")) {
      Serial.println("Lock reported cable cut! Triggering alarm.");
      triggerAlarm();
      isCut = true;
    }
    else if (msg.equalsIgnoreCase("Cable Reconnected")) {
      Serial.println("Lock reconnected normally.");
      resetAlarm();
      isCut = false;
    }
  }
  
  // ---------- Keep alarm active for 5 seconds ----------
  if (alarmActive) {
    if (millis() - alarmStartTime < alarmDuration) {
      // Both buzzers ON + LED ON during alarm
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      resetAlarm();
    }
  }

  // ---------- Sample acceleration every 0.5 second ----------
  if (currentTime - lastSampleTime >= sampleInterval) {
    lastSampleTime = currentTime;

    sensors_event_t event;
    accel.getEvent(&event);

    // Compute total acceleration magnitude
    float totalAcc = sqrt(event.acceleration.x * event.acceleration.x +
                          event.acceleration.y * event.acceleration.y +
                          event.acceleration.z * event.acceleration.z);

    // Apply calibration offset
    float delta = abs((totalAcc - offsetAcc) - baseAcc);

    Serial.print("Δa: ");
    Serial.print(delta, 2);
    Serial.println(" m/s^2");

    // ---------- Initialize or reset monitoring window ----------
    if (windowStartTime == 0 || currentTime - windowStartTime > monitorWindow) {
      windowStartTime = currentTime;
      motionCount = 0;
    }

    // ---------- Count motion detections above threshold ----------
    if (delta > threshold) {
      motionCount++;
      Serial.print("Motion count: ");
      Serial.println(motionCount);
    }

    // ---------- Trigger alarm if motion count condition met ----------
    if (motionCount >= requiredTriggers && (currentTime - windowStartTime <= monitorWindow)) {
      triggerAlarm();
    }
  }
  
  // Upload every 5 seconds
  if (currentTime - lastUploadTime >= UPLOAD_INTERVAL) {
    lastUploadTime = currentTime;
    sensors_event_t event;
    accel.getEvent(&event);
    uploadToFirebase(event.acceleration.x, event.acceleration.y, event.acceleration.z, alarmActive);
  }

}

// =======================================================
// Calibration Function: measure 100 samples to compute offset
// =======================================================
void calibrateADXL() {
  Serial.println("\n=== Calibrating ADXL345... Keep the module still. ===");
  float sum = 0;
  int samples = 100;

  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    float totalAcc = sqrt(event.acceleration.x * event.acceleration.x +
                          event.acceleration.y * event.acceleration.y +
                          event.acceleration.z * event.acceleration.z);
    sum += totalAcc;
    delay(20);
  }

  offsetAcc = (sum / samples) - baseAcc;
  Serial.print("Calibration complete. Offset = ");
  Serial.print(offsetAcc, 3);
  Serial.println(" m/s^2\n");
}

// =======================================================
// Alarm Trigger Function
// =======================================================
void triggerAlarm() {
  Serial.println("Movement detected! Triggering 5s alarm!");
  alarmActive = true;
  alarmStartTime = millis();
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
}

// =======================================================
// Alarm Reset Function
// =======================================================
void resetAlarm() {
  alarmActive = false;
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  motionCount = 0;
  windowStartTime = 0;
  Firebase.RTDB.setBool(&fbdo, "device/001/alarm", false);
  Serial.println("Alarm reset. Monitoring resumed.\n");
}

// =======================================================
// Upload Data to Firebase (every 5 seconds)
// =======================================================
void uploadToFirebase(float ax, float ay, float az, bool alarm) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected!");
    return;
  }

  // ----  data ----
  float battery = 82.0;     // e.g., from analogRead(batteryPin)

  float lat, lon;
  bool hasGPS = getGPSLocation(lat, lon);

  if (!hasGPS) {
    Serial.println("No GPS signal. Fetching last known location from Firebase...");
    
    if (Firebase.RTDB.getFloat(&fbdo, "bike_status/lat")) {
      if (fbdo.dataType() == "float" || fbdo.dataType() == "double") {
        lat = fbdo.floatData();
      }
    }

    if (Firebase.RTDB.getFloat(&fbdo, "bike_status/lon")) {
      if (fbdo.dataType() == "float" || fbdo.dataType() == "double") {
        lon = fbdo.floatData();
      }
    }
    
    Serial.printf("Using last known location: %.6f, %.6f\n", lat, lon);
  }

  if (!hasGPS) {
      Serial.println("GPS not fixed yet, using last known or placeholder values.");
  }

  unsigned long timestamp = millis() / 1000;

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
    Serial.println("Data uploaded to Firebase as JSON object");
    if (alarmActive || isCut) {
      Serial.println("Alarm state detected → sending alarm flag to Firebase");
      Firebase.RTDB.setBool(&fbdo, "device/001/alarm", true);
    }
  } else {
    Serial.print("Upload failed: ");
    Serial.println(fbdo.errorReason());
  }
}

// ------------ GPS --------------------
bool getGPSLocation(float &lat, float &lon) {
  if (gps.location.isValid()) {
      lat = gps.location.lat();
      lon = gps.location.lng();
      return true;
  }
  return false;
}