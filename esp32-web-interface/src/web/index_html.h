#pragma once

#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>ESP32-C3 Web</title>
    <style>
      :root { color-scheme: light dark; }
      body { font-family: system-ui, -apple-system, Segoe UI, Roboto, Arial, sans-serif; margin: 0; padding: 24px; }
      .card { max-width: 560px; margin: 0 auto; border: 1px solid rgba(127,127,127,.35); border-radius: 14px; padding: 18px; }
      h1 { font-size: 18px; margin: 0 0 10px; }
      p { margin: 8px 0; }
      .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
      button { padding: 10px 14px; border-radius: 10px; border: 1px solid rgba(127,127,127,.45); background: transparent; cursor: pointer; }
      code { padding: 2px 6px; border-radius: 8px; border: 1px solid rgba(127,127,127,.35); }
      .muted { opacity: .75; }
    </style>
  </head>
  <body>
    <div class="card">
      <h1>ESP32-C3 Super Mini</h1>
      <p class="muted">Simple HTTP page served from the board.</p>
      <p>Host: <code id="host">(loading)</code></p>
      <p>LED: <code id="led">(loading)</code></p>
      <div class="row">
        <button id="toggle">Toggle LED</button>
        <button id="refresh">Refresh</button>
      </div>
    </div>
    <script>
      const $ = (id) => document.getElementById(id);
      $("host").textContent = location.host || "(no host)";

      async function refresh() {
        const r = await fetch("/state", { cache: "no-store" });
        const j = await r.json();
        $("led").textContent = j.led ? "ON" : "OFF";
      }

      $("toggle").addEventListener("click", async () => {
        await fetch("/toggle", { method: "POST" });
        await refresh();
      });
      $("refresh").addEventListener("click", refresh);
      refresh();
    </script>
  </body>
</html>
)rawliteral";
