#include "web_server.h"

#include <WiFi.h>
#include <WebServer.h>

#include "index_html.h"

namespace web {

static WebServer server(80);
static bool ledOn = false;
static uint8_t ledPin = 255;

static void sendJson(int status, const String &json) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(status, "application/json", json);
}

static void setupRoutes() {
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/toggle", HTTP_POST, []() {
    ledOn = !ledOn;
    if (ledPin != 255) {
      digitalWrite(ledPin, ledOn ? HIGH : LOW);
    }
    Serial.print("HTTP: POST /toggle -> LED ");
    Serial.println(ledOn ? "ON" : "OFF");
    sendJson(200, String("{\"led\":") + (ledOn ? "true" : "false") + "}");
  });

  server.on("/state", HTTP_GET, []() {
    sendJson(200, String("{\"led\":") + (ledOn ? "true" : "false") + "}");
  });

  server.on("/info", HTTP_GET, []() {
    const bool isAp = (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA);
    IPAddress ip = isAp ? WiFi.softAPIP() : WiFi.localIP();

    String json = "{";
    json += String("\"mode\":\"") + (isAp ? "ap" : "sta") + "\",";
    json += "\"ip\":\"" + ip.toString() + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"freeHeap\":" + String(ESP.getFreeHeap());
    json += "}";

    sendJson(200, json);
  });

  server.onNotFound([]() {
    Serial.print("HTTP: 404 ");
    Serial.println(server.uri());
    server.send(404, "text/plain", "Not found");
  });
}

void begin(uint8_t pin) {
  ledPin = pin;
  ledOn = false;
  if (ledPin != 255) {
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
  }

  setupRoutes();
  server.begin();

  Serial.println("HTTP: server started on port 80");
}

void handle() {
  server.handleClient();
}

} // namespace web
