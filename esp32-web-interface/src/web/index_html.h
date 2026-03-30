#pragma once

#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>ESP32-C3 Detection</title>
    <style>
      :root { color-scheme: light dark; }
      body { font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; margin: 0; padding: 22px; }
      .wrap { max-width: 940px; margin: 0 auto; display: flex; gap: 14px; align-items: flex-start; flex-wrap: wrap; }
      .card { flex: 1 1 560px; border: 1px solid rgba(127,127,127,.35); border-radius: 14px; padding: 18px; }
      .side { flex: 0 1 280px; border: 1px solid rgba(127,127,127,.35); border-radius: 14px; padding: 18px; }
      h1 { font-size: 18px; margin: 0 0 12px; }
      h2 { font-size: 14px; margin: 0 0 10px; opacity: .9; }
      p { margin: 8px 0; }
      .muted { opacity: .75; }

      .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
      @media (max-width: 720px) { .grid { grid-template-columns: 1fr; } }

      button {
        width: 100%;
        padding: 12px 14px;
        border-radius: 12px;
        border: 1px solid rgba(127,127,127,.45);
        background: transparent;
        cursor: pointer;
        text-align: left;
      }
      button:active { transform: translateY(1px); }
      .pill { display: inline-block; padding: 2px 8px; border-radius: 999px; border: 1px solid rgba(127,127,127,.35); font-size: 12px; }
      .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
      code { padding: 2px 6px; border-radius: 8px; border: 1px solid rgba(127,127,127,.35); }
      .status {
        margin: 14px 0 6px;
        min-height: 44px;
        font-weight: 800;
        font-size: 36px;
        letter-spacing: .5px;
        text-align: center;
        padding: 10px 12px;
        border-radius: 12px;
        border: 1px solid rgba(127,127,127,.35);
        background: rgba(127,127,127,.12);
      }
    </style>
  </head>
  <body>
    <div class="wrap">
      <div class="card">
        <h1>Detection Mode</h1>
        <p class="muted">Select a method to start. A short countdown will run: Ready → Steady → Go!</p>
        <div class="grid">
          <button data-mode="ultrasonic"><span class="pill">Ultrasonic</span><br/>Ultrasonic Detection</button>
          <button data-mode="ir"><span class="pill">IR</span><br/>IR Detection</button>
          <button data-mode="lidar"><span class="pill">LiDAR</span><br/>LiDAR Detection</button>
          <button data-mode="fusion"><span class="pill">Fusion</span><br/>Sensor Fusion Detection</button>
        </div>

        <p class="status" id="countdown">Choose a mode…</p>
        <p class="muted">Current mode: <code id="currentMode">(unknown)</code></p>
      </div>

      <div class="side">
        <h2>Connection</h2>
        <p>Connected to WiFi: <code id="wifi">(loading)</code></p>
        <p class="muted">Host: <code id="host">(loading)</code></p>

        <h2 style="margin-top:16px;">LED</h2>
        <p>State: <code id="led">(loading)</code></p>
        <div class="row">
          <button id="blink" style="width:auto;">Blink</button>
          <button id="toggle" style="width:auto;">Toggle</button>
          <button id="refresh" style="width:auto;">Refresh</button>
        </div>
      </div>
    </div>

    <script>
      const $ = (id) => document.getElementById(id);

      function setCountdownText(text) {
        $("countdown").textContent = text;
      }

      async function refreshLed() {
        const r = await fetch("/state", { cache: "no-store" });
        const j = await r.json();
        $("led").textContent = j.led ? "ON" : "OFF";
      }

      async function refreshWifi() {
        const r = await fetch("/wifi", { cache: "no-store" });
        const j = await r.json();
        $("wifi").textContent = j.ssid || "(none)";
      }

      async function refreshMode() {
        const r = await fetch("/detection", { cache: "no-store" });
        const j = await r.json();
        $("currentMode").textContent = j.mode || "(none)";
      }

      async function refreshAll() {
        $("host").textContent = location.host || "(no host)";
        await Promise.all([refreshLed(), refreshWifi(), refreshMode()]);
      }

      let tReady = 0, tSteady = 0, tGo = 0;

      function clearCountdownTimers() {
        if (tReady) clearTimeout(tReady);
        if (tSteady) clearTimeout(tSteady);
        if (tGo) clearTimeout(tGo);
        tReady = tSteady = tGo = 0;
      }

      function runCountdown(onGo) {
        clearCountdownTimers();
        setCountdownText("Ready");
        tSteady = setTimeout(() => setCountdownText("Steady"), 700);
        tGo = setTimeout(() => {
          setCountdownText("Go!");
          if (typeof onGo === "function") onGo();
        }, 1400);
      }

      async function setDetectionMode(mode) {
        await fetch("/detection", {
          method: "POST",
          headers: { "Content-Type": "application/x-www-form-urlencoded" },
          body: "mode=" + encodeURIComponent(mode)
        });
      }

      document.querySelectorAll("button[data-mode]").forEach((btn) => {
        btn.addEventListener("click", async () => {
          const mode = btn.getAttribute("data-mode");
          runCountdown(async () => {
            await fetch("/go", {
              method: "POST",
              headers: { "Content-Type": "application/x-www-form-urlencoded" },
              body: "mode=" + encodeURIComponent(mode)
            });
          });
          await setDetectionMode(mode);
          await refreshMode();
        });
      });

      $("toggle").addEventListener("click", async () => {
        await fetch("/toggle", { method: "POST" });
        await refreshLed();
      });

      $("blink").addEventListener("click", async () => {
        await fetch("/blink", {
          method: "POST",
          headers: { "Content-Type": "application/x-www-form-urlencoded" },
          body: "count=3&ms=150"
        });
      });

      $("refresh").addEventListener("click", refreshAll);

      refreshAll();
    </script>
  </body>
</html>
)rawliteral";
