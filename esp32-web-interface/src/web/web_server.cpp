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

static const char *responsePrefixForMode(const String &mode) {
  if (mode == "ir") return "#M:I,";
  if (mode == "ultrasonic") return "#M:U,";
  if (mode == "lidar") return "#M:L,";
  if (mode == "fusion") return "#M:F,";
  return nullptr;
}

static const char *frameForDirection(const String &direction) {
  if (direction == "front") return "#D:1;";
  if (direction == "back") return "#D:2;";
  if (direction == "right") return "#D:3;";
  if (direction == "left") return "#D:4;";
  return nullptr;
}

static void clearUartRx() {
  while (Serial1.available() > 0) {
    Serial1.read();
  }
}

static String readUartFrame(unsigned long timeoutMs) {
  String response = "";
  const unsigned long start = millis();

  while ((millis() - start) < timeoutMs) {
    while (Serial1.available() > 0) {
      const char c = static_cast<char>(Serial1.read());
      if (c == '\r' || c == '\n') {
        continue;
      }

      response += c;
      if (c == ';' || response.length() >= 64) {
        return response;
      }
    }
    delay(1);
  }

  return response;
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
    clearUartRx();
    Serial1.print(frame);
    Serial1.flush();

    const String response = readUartFrame(3000);
    const char *expectedPrefix = responsePrefixForMode(mode);
    const bool accepted = (expectedPrefix != nullptr && response.startsWith(expectedPrefix) && response.endsWith(";"));

    Serial.print("UART: RX detection ");
    Serial.println(response.length() > 0 ? response : "(timeout)");

    sendJson(accepted ? 200 : 504,
             String("{\"ok\":") + (accepted ? "true" : "false") +
               ",\"mode\":\"" + mode +
               "\",\"frame\":\"" + frame +
               "\",\"response\":\"" + response + "\"}");
  });

  server.on("/motion", HTTP_POST, []() {
    String direction = "";
    if (server.hasArg("direction")) {
      direction = server.arg("direction");
      direction.toLowerCase();
    }

    const char *frame = frameForDirection(direction);
    if (!frame) {
      Serial.print("HTTP: POST /motion unknown direction=");
      Serial.println(direction);
      sendJson(400, String("{\"ok\":false,\"error\":\"unknown_direction\",\"direction\":\"") + direction + "\"}");
      return;
    }

    Serial.print("UART: TX ");
    Serial.println(frame);
    clearUartRx();
    Serial1.print(frame);
    Serial1.flush();

    const String response = readUartFrame(500);
    const bool accepted = (response == "#D:OK;" || response == "D:OK;");

    Serial.print("UART: RX motion ");
    Serial.println(response.length() > 0 ? response : "(timeout)");

    sendJson(accepted ? 200 : 504,
             String("{\"ok\":") + (accepted ? "true" : "false") +
               ",\"direction\":\"" + direction +
               "\",\"frame\":\"" + frame +
               "\",\"response\":\"" + response + "\"}");
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
