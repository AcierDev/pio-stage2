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
#include "config/Config.h"

//* ************************************************************************
//* ************************ OTA CONFIGURATION ***************************
//* ************************************************************************
// WiFi Configuration
const char* WIFI_SSID = "Everwood";
const char* WIFI_PASSWORD = "Everwood-Staff";

// OTA Configuration
const char* OTA_HOSTNAME = "transfer-arm";
const char* OTA_PASSWORD = "transfer-arm-ota";
const float OTA_PORT = 3232.0;

// Connection timeouts
const unsigned long WIFI_TIMEOUT = 30000;  // 30 seconds
const unsigned long OTA_TIMEOUT = 10000;   // 10 seconds

//* ************************************************************************
//* ************************ WIFI CONNECTION FUNCTIONS ******************
//* ************************************************************************

void initWiFi() {
  Serial.println("\n=== ESP32 OTA Remote Upload Setup ===");
  
  //! Step 1: Configure static IP
  IPAddress local_IP(192, 168, 1, 218);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Static IP configuration failed");
  }
  
  //! Step 2: Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  
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
  // Configure static IP
  IPAddress local_IP(192, 168, 1, 251);
  IPAddress gateway(192, 168, 1, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);
  IPAddress secondaryDNS(8, 8, 4, 4);

  // Try to configure static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    // Static IP configuration failed
  }

  // Connect to Wi-Fi
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  WiFi.mode(WIFI_STA);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // Configure OTA
  ArduinoOTA.setHostname("ESP32-S3-Stage2");
  ArduinoOTA.setPassword("stage2-ota");
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
  });

  ArduinoOTA.onEnd([]() {
    // OTA update completed
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Progress update
  });

  ArduinoOTA.onError([](ota_error_t error) {
    // Error handling
    if (error == OTA_AUTH_ERROR) {
      // Auth Failed
    } else if (error == OTA_BEGIN_ERROR) {
      // Begin Failed
    } else if (error == OTA_CONNECT_ERROR) {
      // Connect Failed
    } else if (error == OTA_RECEIVE_ERROR) {
      // Receive Failed
    } else if (error == OTA_END_ERROR) {
      // End Failed
    }
  });

  ArduinoOTA.begin();
}

//* ************************************************************************
//* ************************ OTA RUNTIME FUNCTIONS **********************
//* ************************************************************************

void handleOTA() {
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