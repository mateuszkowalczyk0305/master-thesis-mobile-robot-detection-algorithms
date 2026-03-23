#include <Arduino.h>
#include <WiFi.h>

#include "web/web_server.h"

#ifndef LED_PIN
#define LED_PIN 8
#endif

// Defined in src/main.cpp (so you only edit credentials there)
extern const char *WIFI_SSID;
extern const char *WIFI_PASS;

static void printPageUrl(const IPAddress &ip) {
  Serial.print("Page: http://");
  Serial.print(ip);
  Serial.println("/");
}

static void printBootInfo() {
  const bool isAp = (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA);
  const IPAddress ip = isAp ? WiFi.softAPIP() : WiFi.localIP();

  Serial.println();
  Serial.println("Boot: ESP32-C3 web interface");
  Serial.print("Mode: ");
  Serial.println(isAp ? "AP" : "STA");
  Serial.print("IP: ");
  Serial.println(ip);
  printPageUrl(ip);
  Serial.println("Endpoints: GET /  GET /state  POST /toggle  GET /info");
}

static bool bootInfoPrinted = false;

static void setupWifi() {
  const bool hasCreds = (WIFI_SSID != nullptr && WIFI_SSID[0] != '\0');

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
      Serial.print("WiFi: disconnected, reason=");
      Serial.println(info.wifi_sta_disconnected.reason);
    }
  });

  if (hasCreds) {
    Serial.print("WiFi: connecting to SSID '");
    Serial.print(WIFI_SSID);
    Serial.println("'...");

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    const unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 15000) {
      delay(250);
      Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi: connected");
      return;
    }

    Serial.println("WiFi: connect failed, falling back to AP");
  }

  WiFi.mode(WIFI_AP);
  const char *apSsid = "ESP32-C3";
  Serial.print("WiFi: starting AP '");
  Serial.print(apSsid);
  Serial.println("'");
  WiFi.softAP(apSsid);
  Serial.println("WiFi: connect to that AP (ESP32-C3), then open the Page URL");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  setupWifi();
  web::begin(LED_PIN);

  // For USB-CDC Serial, wait briefly for the monitor to attach.
  const unsigned long start = millis();
  while (!Serial && (millis() - start) < 2000) {
    delay(10);
  }
  if (Serial) {
    printBootInfo();
    bootInfoPrinted = true;
  }
}

void loop() {
  web::handle();

  // If the monitor is opened after boot, print the info once when Serial becomes available.
  if (!bootInfoPrinted && Serial) {
    printBootInfo();
    bootInfoPrinted = true;
  }
}
