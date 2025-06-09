/*
 * OTA_Manager.cpp - Unified Over-The-Air Update Manager for ESP32
 * 
 * This file contains ALL OTA functionality in one place:
 * - WiFi configuration constants
 * - OTA configuration constants  
 * - All OTA function implementations
 * 
 * SETUP INSTRUCTIONS:
 * 1. Include this file in your main.cpp
 * 2. Call initOTA() in setup()
 * 3. Call handleOTA() in loop()
 * 4. Update IP address in platformio.ini to match your ESP32's IP
 * 
 * USAGE:
 * - Use 'pio run -t upload' for OTA uploads
 * - Monitor serial output to see IP address assigned to ESP32
 * - Update platformio.ini upload_port with the assigned IP
 * 
 * NETWORK REQUIREMENTS:
 * - ESP32 and computer must be on same network
 * - Port 3232 must be open for OTA communication
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

//* ************************************************************************
//* ************************ OTA CONFIGURATION ***************************
//* ************************************************************************
// WiFi Configuration
const char* WIFI_SSID = "Everwood";
const char* WIFI_PASSWORD = "Everwood-Staff";

// OTA Configuration
const char* OTA_HOSTNAME = "transfer-arm";
const char* OTA_PASSWORD = "transfer-arm-ota";
const int OTA_PORT = 3232;

// Connection timeouts
const unsigned long WIFI_TIMEOUT = 30000;  // 30 seconds
const unsigned long OTA_TIMEOUT = 10000;   // 10 seconds

//* ************************************************************************
//* ************************ WIFI CONNECTION FUNCTIONS ******************
//* ************************************************************************

void initWiFi() {
  Serial.println("\n=== ESP32 OTA Remote Upload Setup ===");
  
  //! Step 1: Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

//* ************************************************************************
//* ************************ OTA SETUP FUNCTIONS ************************
//* ************************************************************************

void initOTA() {
  // Initialize WiFi first
  initWiFi();
  
  //! Step 2: Configure OTA
  ArduinoOTA.setHostname("ESP32-Remote");
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  //! Step 3: Start OTA service
  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.println("Device ready for remote uploads!");
  Serial.print("Use IP: ");
  Serial.println(WiFi.localIP());
}

//* ************************************************************************
//* ************************ OTA RUNTIME FUNCTIONS **********************
//* ************************************************************************

void handleOTA() {
  //! Handle OTA updates
  ArduinoOTA.handle();
}

void displayIP() {
  //! Display IP every 10 seconds
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 10000) {
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    lastPrint = millis();
  }
} 