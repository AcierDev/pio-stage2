#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

//* ************************************************************************
//* ************************ OTA FUNCTION DECLARATIONS ******************
//* ************************************************************************

// WiFi connection functions
void initWiFi();

// OTA setup and management functions  
void initOTA();
void handleOTA();
void displayIP();

#endif // OTA_MANAGER_H 