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
// WiFi Configuration - Use config file settings
// OTA Configuration
const char* OTA_HOSTNAME = "ESP32-S3-Stage2";
const char* OTA_PASSWORD = "stage2-ota";
const int OTA_PORT = 3232;

// Static IP Configuration
IPAddress STATIC_IP(192, 168, 1, 251);
IPAddress GATEWAY(192, 168, 1, 1);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress PRIMARY_DNS(8, 8, 8, 8);
IPAddress SECONDARY_DNS(8, 8, 4, 4);

// Connection timeouts
const unsigned long WIFI_TIMEOUT = 30000;  // 30 seconds

//* ************************************************************************
//* ************************ OTA SETUP FUNCTIONS ************************
//* ************************************************************************

void initOTA() {
  Serial.println("\n=== ESP32-S3 OTA Setup ===");
  
  //! Step 1: Configure static IP first
  Serial.println("Configuring static IP...");
  if (!WiFi.config(STATIC_IP, GATEWAY, SUBNET, PRIMARY_DNS, SECONDARY_DNS)) {
    Serial.println("WARNING: Static IP configuration failed, using DHCP");
  } else {
    Serial.print("Static IP configured: ");
    Serial.println(STATIC_IP);
  }

  //! Step 2: Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  
  Serial.print("Connecting to WiFi network: ");
  Serial.println(Config::WIFI_SSID);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startTime) < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    return;
  }
  
  Serial.println("");
  Serial.println("WiFi connected successfully!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Subnet: ");
  Serial.println(WiFi.subnetMask());

  //! Step 3: Configure ArduinoOTA
  Serial.println("Configuring OTA...");
  
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setPort(OTA_PORT);
  
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Starting OTA update (" + type + ")...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA update completed successfully!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    // Only print progress every 500ms to avoid spam
    if (now - lastUpdate > 500) {
      Serial.printf("OTA Progress: %u%% (%u/%u bytes)\n", (progress / (total / 100)), progress, total);
      lastUpdate = now;
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Authentication Failed - Check OTA password");
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

  ArduinoOTA.begin();
  
  Serial.println("OTA initialized successfully!");
  Serial.println("=== OTA Configuration ===");
  Serial.print("Hostname: ");
  Serial.println(OTA_HOSTNAME);
  Serial.print("Port: ");
  Serial.println(OTA_PORT);
  Serial.println("Password: [PROTECTED]");
  Serial.println("Ready for OTA updates - OTA Test v1.1!");
  Serial.println("========================");
}

//* ************************************************************************
//* ************************ OTA RUNTIME FUNCTIONS **********************
//* ************************************************************************

void handleOTA() {
  ArduinoOTA.handle();
}

void displayIP() {
  //! Display IP every 30 seconds for OTA reference
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 30000) {
    Serial.print("OTA IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("OTA Hostname: ");
    Serial.println(OTA_HOSTNAME);
    lastPrint = millis();
  }
} 