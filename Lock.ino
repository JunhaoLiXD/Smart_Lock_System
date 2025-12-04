#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
const int CABLE_PIN = 32;     // Continuity detection pin
const int TEST_LED = 2;       // On-board LED (or external LED on GPIO2)
bool lastState = HIGH;        // HIGH = connected, LOW = cut
unsigned long lastDebounce = 0;
const unsigned long DEBOUNCE_MS = 50;

void setup() {
  Serial.begin(115200);
  pinMode(CABLE_PIN, INPUT_PULLDOWN);  // Use internal pulldown resistor
  pinMode(TEST_LED, OUTPUT);
  digitalWrite(TEST_LED, LOW);

  if (!SerialBT.begin("BikeLock")) {
    Serial.println("Bluetooth init failed!");
    while (1);
  }
  Serial.println("Bluetooth Lock ready, waiting for hub to connect...");
  Serial.println(SerialBT.getBtAddressString());
}

void loop() {
  int val = digitalRead(CABLE_PIN); // HIGH = connected, LOW = cut

  if (val != lastState && millis() - lastDebounce > DEBOUNCE_MS) {
    lastDebounce = millis();

    if (val == LOW) {
      Serial.println("Cable Cut Detected!");
      digitalWrite(TEST_LED, HIGH);   // LED on when cable is cut

      SerialBT.println("Cable Cut Detected");
      delay(100);
    }
    else {
      Serial.println("Cable connected normally.");
      digitalWrite(TEST_LED, LOW);    // LED off when cable is okay

      SerialBT.println("Cable Reconnected");
    }
    lastState = val;
  }

  delay(20);
}
