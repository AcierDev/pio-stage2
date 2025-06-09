#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

//* ************************************************************************
//* ************************ OTA MANAGER HEADER **************************
//* ************************************************************************
// Header file for OTA_Manager.cpp
// Contains function declarations for Over-The-Air update functionality

// Function declarations
void initWiFi();        // Initialize WiFi connection
void initOTA();         // Initialize OTA service
void handleOTA();       // Handle OTA updates in main loop
void displayIP();       // Display IP address periodically

#endif // OTA_MANAGER_H 