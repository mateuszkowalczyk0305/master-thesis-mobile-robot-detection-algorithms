#pragma once

#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="pl">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Panel sterowania ESP32-C3</title>
    <style>
      :root {
        color-scheme: dark;
        --bg: #11100f;
        --bg-deep: #050505;
        --panel: rgba(22, 23, 24, .9);
        --panel-strong: rgba(31, 32, 34, .96);
        --line: rgba(244, 241, 234, .16);
        --text: #f4f1ea;
        --muted: #aaa39a;
        --cyan: #d9dde1;
        --green: #b9d765;
        --amber: #f2b84b;
        --rose: #d83b36;
        --violet: #b7b2a8;
        --shadow: 0 18px 42px rgba(0, 0, 0, .34);
      }

      * { box-sizing: border-box; }

      html { -webkit-text-size-adjust: 100%; }

      body {
        min-height: 100vh;
        margin: 0;
        padding: 18px;
        font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
        color: var(--text);
        background:
          linear-gradient(120deg, rgba(216, 59, 54, .22), transparent 38%),
          linear-gradient(240deg, rgba(242, 184, 75, .14), transparent 36%),
          linear-gradient(180deg, #1b1a18 0%, var(--bg) 48%, var(--bg-deep) 100%);
        background-size: 140% 140%, 140% 140%, 100% 100%;
        animation: backgroundDrift 12s ease-in-out infinite alternate;
      }

      button, code, output { font: inherit; }

      button {
        border: 1px solid var(--line);
        border-radius: 8px;
        color: inherit;
        background: rgba(255, 255, 255, .06);
        cursor: pointer;
        touch-action: manipulation;
        transition: transform .18s ease, border-color .18s ease, background .18s ease, box-shadow .18s ease;
      }

      button:active { transform: translateY(1px) scale(.98); }
      button:focus-visible { outline: 2px solid var(--amber); outline-offset: 3px; }
      button:hover { border-color: rgba(216, 59, 54, .64); background: rgba(242, 184, 75, .08); }

      code {
        display: inline-flex;
        align-items: center;
        min-height: 28px;
        max-width: 100%;
        padding: 3px 8px;
        overflow-wrap: anywhere;
        border: 1px solid rgba(255, 255, 255, .13);
        border-radius: 8px;
        color: #fff0c4;
        background: rgba(0, 0, 0, .24);
      }

      h1, h2, h3, p { margin: 0; }

      .shell {
        width: min(1080px, 100%);
        margin: 0 auto;
      }

      .topbar {
        display: flex;
        align-items: flex-end;
        justify-content: space-between;
        gap: 14px;
        margin: 4px 0 16px;
        animation: riseIn .42s ease both;
      }

      .eyebrow {
        display: inline-flex;
        align-items: center;
        gap: 8px;
        margin-bottom: 8px;
        color: var(--amber);
        font-size: 13px;
        font-weight: 700;
      }

      .eyebrow::before {
        content: "";
        width: 8px;
        height: 8px;
        border-radius: 50%;
        background: var(--rose);
        box-shadow: 0 0 18px rgba(216, 59, 54, .82);
      }

      h1 {
        font-size: 28px;
        line-height: 1.12;
        font-weight: 850;
        letter-spacing: 0;
      }

      .chips {
        display: flex;
        flex-wrap: wrap;
        justify-content: flex-end;
        gap: 8px;
      }

      .chip {
        display: inline-flex;
        align-items: center;
        min-height: 34px;
        padding: 7px 10px;
        border: 1px solid var(--line);
        border-radius: 8px;
        color: var(--muted);
        background: rgba(255, 255, 255, .06);
      }

      .layout {
        display: grid;
        grid-template-columns: minmax(0, 1.24fr) minmax(280px, .76fr);
        gap: 14px;
        align-items: start;
      }

      .stack {
        display: grid;
        gap: 14px;
      }

      .panel {
        border: 1px solid var(--line);
        border-radius: 8px;
        padding: 16px;
        background: var(--panel);
        box-shadow: var(--shadow);
        backdrop-filter: blur(16px);
        animation: riseIn .48s ease both;
      }

      .panel:nth-child(2) { animation-delay: .05s; }
      .side-panel { animation-delay: .1s; }

      .section-head {
        display: flex;
        align-items: flex-start;
        justify-content: space-between;
        gap: 12px;
        margin-bottom: 14px;
      }

      .section-head h2 {
        font-size: 18px;
        line-height: 1.2;
        letter-spacing: 0;
      }

      .section-head p {
        margin-top: 4px;
        color: var(--muted);
        font-size: 14px;
        line-height: 1.45;
      }

      .mode-grid {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 10px;
      }

      .mode-btn {
        min-height: 128px;
        padding: 13px;
        text-align: left;
        position: relative;
        overflow: hidden;
      }

      .mode-btn::after {
        content: "";
        position: absolute;
        inset: auto -26px -36px auto;
        width: 86px;
        height: 86px;
        border-radius: 50%;
        opacity: .42;
        background: var(--mode-color, var(--cyan));
        filter: blur(18px);
        transition: opacity .18s ease, transform .18s ease;
      }

      .mode-btn:hover::after,
      .mode-btn.active::after {
        opacity: .68;
        transform: scale(1.12);
      }

      .mode-btn.active {
        border-color: var(--mode-color, var(--cyan));
        background: color-mix(in srgb, var(--mode-color, var(--cyan)) 20%, transparent);
        box-shadow: 0 0 0 1px color-mix(in srgb, var(--mode-color, var(--cyan)) 40%, transparent);
      }

      .mode-btn strong,
      .mode-btn small {
        position: relative;
        z-index: 1;
        display: block;
      }

      .mode-btn strong { font-size: 17px; }
      .mode-btn small { margin-top: 5px; color: var(--muted); line-height: 1.35; }

      .mode-ultra { --mode-color: var(--rose); }
      .mode-ir { --mode-color: var(--amber); }
      .mode-lidar { --mode-color: var(--cyan); }
      .mode-fusion { --mode-color: var(--violet); }

      .status-board {
        display: grid;
        grid-template-columns: minmax(0, 1fr) minmax(150px, auto);
        gap: 10px;
        align-items: center;
        margin-top: 12px;
        padding: 12px;
        border: 1px solid rgba(255, 255, 255, .12);
        border-radius: 8px;
        background: rgba(0, 0, 0, .18);
      }

      .status-board span {
        color: var(--muted);
        font-size: 13px;
      }

      .status {
        display: block;
        min-height: 48px;
        padding: 9px 12px;
        border-radius: 8px;
        color: #081116;
        background: linear-gradient(135deg, var(--rose), var(--amber));
        font-size: 20px;
        font-weight: 900;
        text-align: center;
        animation: softPulse 2.5s ease-in-out infinite;
      }

      .meta-line {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 10px;
        margin-top: 10px;
        color: var(--muted);
        font-size: 14px;
      }

      .motion-grid {
        display: grid;
        grid-template-columns: repeat(3, minmax(72px, 1fr));
        gap: 10px;
        width: min(340px, 100%);
        margin: 0 auto;
      }

      .motion-spacer { min-height: 72px; }

      .motion-btn {
        min-height: 78px;
        padding: 10px 8px;
        display: grid;
        place-items: center;
        gap: 4px;
        text-align: center;
        background: linear-gradient(180deg, rgba(255, 255, 255, .11), rgba(255, 255, 255, .04));
      }

      .motion-btn.sent {
        border-color: var(--green);
        box-shadow: 0 0 0 1px rgba(115, 227, 109, .45), 0 0 24px rgba(115, 227, 109, .2);
      }

      .arrow {
        width: 36px;
        height: 36px;
        display: grid;
        place-items: center;
        border-radius: 8px;
        color: #081116;
        background: var(--rose);
        font-size: 26px;
        line-height: 1;
        font-weight: 900;
      }

      .motion-btn:nth-of-type(2) .arrow { background: var(--amber); }
      .motion-btn:nth-of-type(3) .arrow { background: var(--rose); }
      .motion-btn:nth-of-type(4) .arrow { background: var(--amber); }

      .motion-btn span:last-child {
        font-size: 13px;
        font-weight: 800;
      }

      .telemetry {
        display: grid;
        gap: 10px;
      }

      .readout {
        display: grid;
        gap: 8px;
        margin: 0;
        padding: 0;
        list-style: none;
      }

      .readout li,
      .uart-row {
        display: grid;
        grid-template-columns: minmax(92px, .7fr) minmax(0, 1fr);
        gap: 8px;
        align-items: center;
        padding: 10px;
        border: 1px solid rgba(255, 255, 255, .1);
        border-radius: 8px;
        background: rgba(255, 255, 255, .05);
      }

      .readout span,
      .uart-row span {
        color: var(--muted);
        font-size: 13px;
      }

      .actions {
        display: grid;
        grid-template-columns: repeat(3, minmax(0, 1fr));
        gap: 8px;
        margin-top: 12px;
      }

      .action-btn {
        min-height: 44px;
        padding: 8px;
        text-align: center;
        font-weight: 800;
      }

      .refresh-btn { background: rgba(217, 221, 225, .1); }
      .toggle-btn { background: rgba(255, 200, 87, .13); }
      .blink-btn { background: rgba(255, 107, 138, .13); }

      .service-toggle {
        display: flex;
        align-items: center;
        justify-content: space-between;
        gap: 12px;
        margin-top: 14px;
        padding: 12px;
        border: 1px solid rgba(255, 255, 255, .1);
        border-radius: 8px;
        background: rgba(255, 255, 255, .05);
      }

      .service-copy strong,
      .service-copy span {
        display: block;
      }

      .service-copy strong {
        font-size: 14px;
      }

      .service-copy span {
        margin-top: 3px;
        color: var(--muted);
        font-size: 12px;
        line-height: 1.35;
      }

      .switch {
        position: relative;
        flex: 0 0 auto;
        width: 58px;
        height: 32px;
      }

      .switch input {
        position: absolute;
        inset: 0;
        opacity: 0;
      }

      .slider {
        position: absolute;
        inset: 0;
        border: 1px solid rgba(255, 255, 255, .18);
        border-radius: 999px;
        background: rgba(0, 0, 0, .26);
        cursor: pointer;
        transition: background .18s ease, border-color .18s ease, box-shadow .18s ease;
      }

      .slider::before {
        content: "";
        position: absolute;
        top: 4px;
        left: 4px;
        width: 22px;
        height: 22px;
        border-radius: 50%;
        background: var(--muted);
        transition: transform .18s ease, background .18s ease;
      }

      .switch input:checked + .slider {
        border-color: rgba(242, 184, 75, .82);
        background: rgba(242, 184, 75, .2);
        box-shadow: 0 0 22px rgba(242, 184, 75, .16);
      }

      .switch input:checked + .slider::before {
        transform: translateX(26px);
        background: var(--amber);
      }

      .service-panel {
        display: none;
        margin-top: 12px;
        padding: 12px;
        border: 1px solid rgba(242, 184, 75, .28);
        border-radius: 8px;
        background: rgba(0, 0, 0, .22);
        animation: riseIn .28s ease both;
      }

      .service-panel.visible { display: block; }

      .service-panel h3 {
        margin-bottom: 10px;
        font-size: 15px;
      }

      .protocol-title {
        margin-top: 14px;
      }

      .uart-log-table {
        display: grid;
        grid-template-columns: repeat(2, minmax(0, 1fr));
        gap: 8px;
      }

      .uart-log-head,
      .uart-log-cell {
        min-width: 0;
        padding: 9px;
        border: 1px solid rgba(255, 255, 255, .1);
        border-radius: 8px;
        background: rgba(255, 255, 255, .05);
      }

      .uart-log-head {
        color: var(--amber);
        font-size: 12px;
        font-weight: 900;
        text-align: center;
      }

      .uart-log-cell {
        display: grid;
        gap: 6px;
        align-content: start;
        min-height: 126px;
        max-height: 220px;
        overflow-y: auto;
      }

      .uart-log-cell code {
        width: 100%;
        justify-content: center;
      }

      .protocol-table .uart-log-cell {
        max-height: none;
        overflow: visible;
      }

      .log-entry {
        border-color: rgba(242, 184, 75, .28);
        animation: riseIn .18s ease both;
      }

      .log-entry.rx {
        color: #f4f1ea;
        border-color: rgba(185, 215, 101, .34);
      }

      .log-entry.timeout {
        color: #f2b84b;
        border-color: rgba(216, 59, 54, .34);
      }

      .log-entry.empty {
        color: var(--muted);
        border-style: dashed;
      }

      @keyframes backgroundDrift {
        from { background-position: 0% 0%, 100% 0%, 0 0; }
        to { background-position: 100% 40%, 0% 70%, 0 0; }
      }

      @keyframes riseIn {
        from { opacity: 0; transform: translateY(10px); }
        to { opacity: 1; transform: translateY(0); }
      }

      @keyframes softPulse {
        0%, 100% { box-shadow: 0 0 0 rgba(242, 184, 75, 0); }
        50% { box-shadow: 0 0 22px rgba(242, 184, 75, .2); }
      }

      @media (max-width: 860px) {
        body { padding: 12px; }
        .topbar { align-items: flex-start; flex-direction: column; }
        .chips { justify-content: flex-start; }
        .layout { grid-template-columns: 1fr; }
        h1 { font-size: 24px; }
      }

      @media (max-width: 560px) {
        .panel { padding: 13px; }
        .section-head { display: block; }
        .badge { margin-top: 10px; }
        .mode-grid { grid-template-columns: 1fr; }
        .mode-btn { min-height: 112px; }
        .status-board { grid-template-columns: 1fr; }
        .motion-grid { grid-template-columns: repeat(3, minmax(64px, 1fr)); gap: 8px; }
        .motion-btn { min-height: 72px; }
        .motion-spacer { min-height: 72px; }
        .actions { grid-template-columns: 1fr; }
        .service-toggle { align-items: flex-start; }
        .readout li,
        .uart-row { grid-template-columns: 1fr; }
      }

      @media (prefers-reduced-motion: reduce) {
        *, *::before, *::after {
          animation-duration: .01ms !important;
          animation-iteration-count: 1 !important;
          scroll-behavior: auto !important;
          transition-duration: .01ms !important;
        }
      }
    </style>
  </head>
  <body>
    <div class="shell">
      <header class="topbar">
        <div>
          <p class="eyebrow">Mateusz Kowalczyk 268533</p>
          <h1>Panel sterowania ESP32-C3</h1>
        </div>
        <div class="chips" aria-label="Parametry komunikacji">
          <span class="chip">UART: Serial1</span>
          <span class="chip">115200 baud</span>
          <span class="chip">8N1</span>
          <span class="chip">TX GPIO21 / RX GPIO20</span>
        </div>
      </header>

      <main class="layout">
        <div class="stack">
          <section class="panel">
            <div class="section-head">
              <div>
                <h2>Metoda detekcji</h2>
                <p>Wybierz odpowiednią metodę detekcji.</p>
              </div>
            </div>

            <div class="mode-grid">
              <button class="mode-btn mode-ultra" data-mode="ultrasonic" title="Detekcja ultradźwiękowa">
                <strong>Ultradźwięk</strong>
                <small>Metoda wykorzystująca dwa sensory HC-SR04 umieszczone na frontowej części robota.</small>
              </button>
              <button class="mode-btn mode-ir" data-mode="ir" title="Detekcja podczerwieni">
                <strong>Podczerwień</strong>
                <small>Metoda wykorzystująca trzy sensory GP2Y0A21YK0F umieszczone na frontowej części robota.</small>
              </button>
              <button class="mode-btn mode-lidar" data-mode="lidar" title="Detekcja LiDAR">
                <strong>LiDAR</strong>
                <small>Metoda wykorzystująca sensor LiDAR RPLIDAR A1 umieszczony na górze robota.</small>
              </button>
              <button class="mode-btn mode-fusion" data-mode="fusion" title="Fuzja sensorów">
                <strong>Fuzja sensorów</strong>
                <small>Tryb łączący dane z wielu czujników.</small>
              </button>
            </div>

            <div class="status-board">
              <div>
                <span>Stan sekwencji</span>
                <output class="status" id="countdown">OCZEKIWANIE</output>
              </div>
              <div class="meta-line">
                <span>Aktywny tryb</span>
                <code id="currentMode">brak</code>
              </div>
            </div>
          </section>

          <section class="panel">
            <div class="section-head">
              <div>
                <h2>Sterowanie ruchem</h2>
                <p>Interfejs sterowania ruchem robota (nieużywany w trakcie detekcji).</p>
              </div>
            </div>

            <div class="motion-grid" aria-label="Sterowanie kierunkiem ruchu">
              <span class="motion-spacer" aria-hidden="true"></span>
              <button class="motion-btn" data-direction="front" title="Przód" aria-label="Ruch do przodu">
                <span class="arrow">&uarr;</span>
                <span>Przód</span>
              </button>
              <span class="motion-spacer" aria-hidden="true"></span>
              <button class="motion-btn" data-direction="left" title="Lewo" aria-label="Ruch w lewo">
                <span class="arrow">&larr;</span>
                <span>Lewo</span>
              </button>
              <span class="motion-spacer" aria-hidden="true"></span>
              <button class="motion-btn" data-direction="right" title="Prawo" aria-label="Ruch w prawo">
                <span class="arrow">&rarr;</span>
                <span>Prawo</span>
              </button>
              <span class="motion-spacer" aria-hidden="true"></span>
              <button class="motion-btn" data-direction="back" title="Tył" aria-label="Ruch do tyłu">
                <span class="arrow">&darr;</span>
                <span>Tył</span>
              </button>
              <span class="motion-spacer" aria-hidden="true"></span>
            </div>
          </section>
        </div>

        <aside class="panel side-panel">
          <div class="section-head">
            <div>
              <h2>Telemetria systemu</h2>
            </div>
          </div>

          <ul class="readout">
            <li><span>Sieć WiFi</span><code id="wifi">ładowanie</code></li>
            <li><span>Host HTTP</span><code id="host">ładowanie</code></li>
            <li><span>Stan LED</span><code id="led">ładowanie</code></li>
          </ul>

          <div class="actions">
            <button class="action-btn blink-btn" id="blink" title="Test LED">Test LED</button>
            <button class="action-btn toggle-btn" id="toggle" title="Przełącz LED">Przełącz</button>
            <button class="action-btn refresh-btn" id="refresh" title="Odśwież stan">Odśwież</button>
          </div>

          <div class="service-toggle">
            <div class="service-copy">
              <strong>Tryb serwisowy</strong>
              <span>Pokazuje transmisję UART na żywo i protokół komunikacji.</span>
            </div>
            <label class="switch" title="Tryb serwisowy">
              <input id="serviceMode" type="checkbox" />
              <span class="slider" aria-hidden="true"></span>
            </label>
          </div>

          <section class="service-panel" id="servicePanel" aria-label="UART logs">
            <h3>UART logs na żywo</h3>
            <div class="uart-log-table">
              <div class="uart-log-head">ESP32</div>
              <div class="uart-log-head">STM32</div>
              <div class="uart-log-cell" id="espLog">
                <code class="log-entry empty">brak transmisji</code>
              </div>
              <div class="uart-log-cell" id="stmLog">
                <code class="log-entry empty">brak transmisji</code>
              </div>
            </div>

            <h3 class="protocol-title">Protokół komunikacji</h3>
            <div class="uart-log-table protocol-table">
              <div class="uart-log-head">ESP32</div>
              <div class="uart-log-head">STM32</div>
              <div class="uart-log-cell">
                <code>#M:U;</code>
                <code>#M:I;</code>
                <code>#M:L;</code>
                <code>#M:F;</code>
                <code>#D:1;</code>
                <code>#D:2;</code>
                <code>#D:3;</code>
                <code>#D:4;</code>
              </div>
              <div class="uart-log-cell">
                <code>#M:U,time_ms;</code>
                <code>#M:I,time_ms;</code>
                <code>#M:L,time_ms;</code>
                <code>#M:F,time_ms;</code>
                <code>#D:OK;</code>
              </div>
            </div>
          </section>
        </aside>
      </main>
    </div>

    <script>
      const $ = (id) => document.getElementById(id);
      const modeLabels = {
        ultrasonic: "Ultradźwięk",
        ir: "Podczerwień",
        lidar: "LiDAR",
        fusion: "Fuzja sensorów",
        none: "brak"
      };
      const modeFrames = {
        ultrasonic: "#M:U;",
        ir: "#M:I;",
        lidar: "#M:L;",
        fusion: "#M:F;"
      };
      const directionFrames = {
        front: "#D:1;",
        back: "#D:2;",
        right: "#D:3;",
        left: "#D:4;"
      };

      function setCountdownText(text) {
        $("countdown").textContent = text;
      }

      function setActiveMode(mode) {
        document.querySelectorAll("button[data-mode]").forEach((btn) => {
          btn.classList.toggle("active", btn.getAttribute("data-mode") === mode);
        });
      }

      function pulseButton(btn) {
        btn.classList.add("sent");
        setTimeout(() => btn.classList.remove("sent"), 360);
      }

      function appendUartLog(targetId, value, state) {
        const target = $(targetId);
        const empty = target.querySelector(".empty");
        if (empty) empty.remove();

        const item = document.createElement("code");
        item.className = "log-entry" + (state ? " " + state : "");
        item.textContent = value || "timeout";
        target.prepend(item);

        while (target.children.length > 12) {
          target.removeChild(target.lastElementChild);
        }
      }

      async function refreshLed() {
        const r = await fetch("/state", { cache: "no-store" });
        const j = await r.json();
        $("led").textContent = j.led ? "WYŁ." : "WŁ.";
      }

      async function refreshWifi() {
        const r = await fetch("/wifi", { cache: "no-store" });
        const j = await r.json();
        $("wifi").textContent = j.ssid || "brak";
      }

      async function refreshMode() {
        const r = await fetch("/detection", { cache: "no-store" });
        const j = await r.json();
        const mode = j.mode || "none";
        $("currentMode").textContent = modeLabels[mode] || mode;
        setActiveMode(mode);
      }

      async function refreshAll() {
        $("host").textContent = location.host || "brak hosta";
        await Promise.all([refreshLed(), refreshWifi(), refreshMode()]);
      }

      let tReady = 0, tGo = 0;

      function clearCountdownTimers() {
        if (tReady) clearTimeout(tReady);
        if (tGo) clearTimeout(tGo);
        tReady = tGo = 0;
      }

      function runCountdown(onGo) {
        clearCountdownTimers();
        setCountdownText("GOTOWY");
        tGo = setTimeout(() => {
          setCountdownText("START");
          if (typeof onGo === "function") onGo();
        }, 900);
      }

      async function setDetectionMode(mode) {
        await fetch("/detection", {
          method: "POST",
          headers: { "Content-Type": "application/x-www-form-urlencoded" },
          body: "mode=" + encodeURIComponent(mode)
        });
      }

      async function sendDetection(mode) {
        appendUartLog("espLog", modeFrames[mode] || "błąd", "tx");
        try {
          const r = await fetch("/go", {
            method: "POST",
            headers: { "Content-Type": "application/x-www-form-urlencoded" },
            body: "mode=" + encodeURIComponent(mode)
          });
          const j = await r.json();
          appendUartLog("stmLog", j.response || "timeout", r.ok ? "rx" : "timeout");
        } catch (error) {
          appendUartLog("stmLog", "błąd HTTP", "timeout");
        }
      }

      async function sendMotion(direction) {
        appendUartLog("espLog", directionFrames[direction] || "błąd", "tx");
        try {
          const r = await fetch("/motion", {
            method: "POST",
            headers: { "Content-Type": "application/x-www-form-urlencoded" },
            body: "direction=" + encodeURIComponent(direction)
          });
          const j = await r.json();
          appendUartLog("stmLog", j.response || "timeout", r.ok ? "rx" : "timeout");
        } catch (error) {
          appendUartLog("stmLog", "błąd HTTP", "timeout");
        }
      }

      function updateServicePanel() {
        $("servicePanel").classList.toggle("visible", $("serviceMode").checked);
      }

      document.querySelectorAll("button[data-mode]").forEach((btn) => {
        btn.addEventListener("click", async () => {
          const mode = btn.getAttribute("data-mode");
          setActiveMode(mode);
          runCountdown(() => sendDetection(mode));
          await setDetectionMode(mode);
          await refreshMode();
        });
      });

      document.querySelectorAll("button[data-direction]").forEach((btn) => {
        btn.addEventListener("click", async () => {
          pulseButton(btn);
          await sendMotion(btn.getAttribute("data-direction"));
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
      $("serviceMode").addEventListener("change", updateServicePanel);

      refreshAll();
      updateServicePanel();
    </script>
  </body>
</html>
)rawliteral";
