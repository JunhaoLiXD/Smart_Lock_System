**Smart Lock System**

This project implements a bicycle smart lock system using two ESP32 devices. The system includes motion detection, Bluetooth communication, GPS tracking, Firebase database updates, and Pushover notifications. A simple webpage is also included for displaying device status.


**Project Description**

The system consists of two ESP32 modules: a Lock Device and a Bike Device.
The Lock Device detects abnormal motion and triggers an alert.
The Bike Device receives this alert through Bluetooth, collects GPS information, uploads data to Firebase, and sends notifications to the user.
A web interface is provided for real-time monitoring.


**File Descriptions**

**• Lock.ino**
Firmware for the lock-side ESP32.
Responsible for accelerometer readings, motion detection, alarm activation, and Bluetooth communication with the bike device.

**• Bike.ino**
Firmware for the bike-side ESP32.
Handles Bluetooth reception, GPS data collection, WiFi connection, Firebase uploads, and sending notification triggers.

**• index.js**
A Firebase Cloud Function that listens for alarm value changes in the database.
When the alarm value becomes true, this script automatically sends a Pushover alert to the user.
It acts as the backend automation that delivers notifications based on database updates.

**• webpage.html**
A simple webpage that displays information retrieved from the Firebase database, including device status and messages.

**• .gitignore**
Defines files and folders that should not be tracked by Git, preventing unnecessary system or temporary files from being included in the repository.


**System Summary**

The Lock Device detects unusual activity and alerts the Bike Device.
The Bike Device processes the signal, updates the database, and triggers notifications.
The Firebase Function sends a Pushover alert, and the webpage displays real-time system information.
