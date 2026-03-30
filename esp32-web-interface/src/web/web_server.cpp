#include "web_server.h"

#include <WiFi.h>
#include <WebServer.h>

#include "index_html.h"

namespace web {

static WebServer server(80);
static bool ledOn = false;
static uint8_t ledPin = 255;

static String detectionMode = "none";

struct BlinkState {
  bool active = false;
  uint8_t remainingToggles = 0;
  uint16_t intervalMs = 0;
  unsigned long nextToggleAt = 0;
};

static BlinkState blink;

static void updateBlink() {
  if (!blink.active || ledPin == 255) {
    return;
  }

  const unsigned long now = millis();
  if (now < blink.nextToggleAt) {
    return;
  }

  ledOn = !ledOn;
  digitalWrite(ledPin, ledOn ? HIGH : LOW);

  if (blink.remainingToggles > 0) {
    blink.remainingToggles--;
  }

  if (blink.remainingToggles == 0) {
    blink.active = false;
  } else {
    blink.nextToggleAt = now + blink.intervalMs;
  }
}

static void sendJson(int status, const String &json) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(status, "application/json", json);
}

static const char *frameForMode(const String &mode) {
  if (mode == "ir") return "#M:I;";
  if (mode == "ultrasonic") return "#M:U;";
  if (mode == "lidar") return "#M:L;";
  if (mode == "fusion") return "#M:F;";
  return nullptr;
}

static void setupRoutes() {
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/wifi", HTTP_GET, []() {
    const bool isAp = (WiFi.getMode() == WIFI_MODE_AP || WiFi.getMode() == WIFI_MODE_APSTA);
    const String ssid = isAp ? String("ESP32-C3") : WiFi.SSID();

    String json = "{";
    json += String("\"mode\":\"") + (isAp ? "ap" : "sta") + "\",";
    json += "\"ssid\":\"" + ssid + "\"";
    json += "}";
    sendJson(200, json);
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

  server.on("/blink", HTTP_POST, []() {
    uint16_t count = 3;
    uint16_t ms = 150;

    if (server.hasArg("count")) {
      const int v = server.arg("count").toInt();
      if (v > 0 && v <= 50) {
        count = static_cast<uint16_t>(v);
      }
    }
    if (server.hasArg("ms")) {
      const int v = server.arg("ms").toInt();
      if (v >= 30 && v <= 2000) {
        ms = static_cast<uint16_t>(v);
      }
    }

    blink.active = true;
    blink.remainingToggles = static_cast<uint8_t>(count * 2);
    blink.intervalMs = ms;
    blink.nextToggleAt = millis();

    Serial.print("HTTP: POST /blink count=");
    Serial.print(count);
    Serial.print(" ms=");
    Serial.println(ms);

    sendJson(200, String("{\"ok\":true,\"count\":") + count + ",\"ms\":" + ms + "}");
  });

  server.on("/state", HTTP_GET, []() {
    sendJson(200, String("{\"led\":") + (ledOn ? "true" : "false") + "}");
  });

  server.on("/detection", HTTP_GET, []() {
    sendJson(200, String("{\"mode\":\"") + detectionMode + "\"}");
  });

  server.on("/detection", HTTP_POST, []() {
    if (server.hasArg("mode")) {
      detectionMode = server.arg("mode");
      detectionMode.toLowerCase();
    }
    Serial.print("HTTP: POST /detection mode=");
    Serial.println(detectionMode);
    sendJson(200, String("{\"mode\":\"") + detectionMode + "\"}");
  });

  // Called by the webpage exactly when the countdown reaches "Go!".
  // Sends a short frame over UART (Serial1).
  server.on("/go", HTTP_POST, []() {
    String mode = detectionMode;
    if (server.hasArg("mode")) {
      mode = server.arg("mode");
      mode.toLowerCase();
    }

    const char *frame = frameForMode(mode);
    if (!frame) {
      Serial.print("HTTP: POST /go unknown mode=");
      Serial.println(mode);
      sendJson(400, String("{\"ok\":false,\"error\":\"unknown_mode\",\"mode\":\"") + mode + "\"}");
      return;
    }

    Serial.print("UART: TX ");
    Serial.println(frame);
    Serial1.print(frame);

    sendJson(200, String("{\"ok\":true,\"mode\":\"") + mode + "\",\"frame\":\"" + frame + "\"}");
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
  updateBlink();
}

} // namespace web
