/*
   =============================================================================
   LAM RESEARCH CHALLENGE 2026 — STAGE 3: PROCESS AUTOMATION
   ESP32 #1 (U2: Temperature Triggered Simultaneous 3L Dosing)

   * DIRECT MANUAL FLUSH SOLENOID VALVE TEST CONTROL INCLUDED *
   Pin: PIN_VALVE_FLUSH = 22 (SV2)

   LOGIC:
   - Manual Mode: You can click [FORCE ON] or [FORCE OFF] anytime from dashboard.
   - Auto Mode:
       * Flush Pump ON -> Flush Solenoid Valve is ON (HIGH)
       * Blue Pump ON  -> Flush Solenoid Valve is OFF (LOW)
       * Rest of cases -> Flush Solenoid Valve is OFF (LOW)
   =============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "driver/gpio.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ─── 1. FUNCTION PROTOTYPES ──────────────────────────────────────────────────
void shutDownAll();
void enterState(int newState);
void sendState();
void setPump(int pumpIndex, bool on, int speed = 255);
void setValve(int valvePin, bool open);
void updateFlushValve();

__attribute__((constructor)) static void earlyDisableBrownout() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
}

// ─── 2. HARDWARE PIN MAPPING ─────────────────────────────────────────────────
const int PIN_SW = 14;

// Solenoid Valves
const int PIN_VALVE_YELLOW = 23; // SV1
const int PIN_VALVE_FLUSH  = 22; // SV2 (Flush Solenoid Valve)
const int PIN_VALVE_GREEN  = 21; // SV3
const int PIN_VALVE_BLUE   = 19; // SV4
const int PIN_VALVE_5      = 18; // SV5

// Pumps
const int PIN_PUMP_GREEN  = 33; // Green Pump (starts first)
const int PIN_PUMP_BLUE   = 26; // Blue Pump
const int PIN_PUMP_YELLOW = 32; // Yellow Pump
const int PIN_PUMP_FLUSH  = 25; // Flush Tank Pump

// ─── 3. CALIBRATION & TELEMETRY ──────────────────────────────────────────────
uint32_t lastRawBlue = 0, lastRawGreen = 0, lastRawYellow = 0;
uint32_t lastBootId = 0;
bool haveBoot = false;
volatile unsigned long lastPacketTime = 0;
volatile bool u7Connected = false;
const unsigned long U7_TIMEOUT_MS = 3000;

// Volumes
float volBlue_mL   = 0.0f;
float volGreen_mL  = 0.0f;
float volYellow_mL = 0.0f;

// DS18B20 Temperatures from U7
float tempYellow    = 25.0f;
float tempGreen     = 25.0f;
float deltaT        = 0.0f;
bool  forceTempPass = false;

// Dosing flags
bool doneBlue   = false;
bool doneGreen  = false;
bool doneYellow = false;

// DIRECT MANUAL FLUSH SOLENOID OVERRIDE
bool manualFlushActive = false;
bool manualFlushVal    = false;

// Calibration factors (pulses per litre)
// FINAL teal mix from test run (real mL): Blue ~998 : Green ~1233 : Yellow ~589
// scaled to 3000 mL total -> Blue ~1061, Green ~1312, Yellow ~626.
// Dashboard still shows 1800 / 900 / 300 targets (unchanged); each factor is
// tuned so the display hits its target exactly when the real volume is reached.
// Last run with the previous values gave only ~2 L (target 3 L), colour was right,
// so all three scaled by 3/2 (x1.5).
float pplBlue   = 1174.2f;
float pplGreen  = 3975.2f;
float pplYellow = 7359.5f;

// Targets (3.0 L Batch)
const float TARGET_DELTAT =   10.0f; // Delta T >= 10.0 °C
const float TARGET_BLUE   = 1800.0f; // 60% of 3L
const float TARGET_GREEN  =  900.0f; // 30% of 3L
const float TARGET_YELLOW =  300.0f; // 10% of 3L

// PWM Speeds
const int PWM_FREQ        = 5000;
const int PWM_RESOLUTION  = 8;
const int PWM_FAST        = 255;
const int PWM_SLOW        = 215;

// ─── 4. STATE MACHINE DEFINITIONS ────────────────────────────────────────────
const int STATE_IDLE             = 0;
const int STATE_TEMP_WAIT        = 1;
const int STATE_STEP3_SIMUL      = 2;
const int STATE_COMPLETED        = 3;
const int STATE_FLUSH            = 4;   // FIRST stage: flush 2 L of clear water into new jar (then temp check, then mixing)

// ── FLUSH STAGE SETTINGS (tune these) ────────────────────────────────────────
// No flow sensor on the flush line, so the 2000 mL is timed:
//   FLUSH_TIME_MS = 2000 mL / (flush pump flow in mL per second) * 1000
// Measured: 2 L takes 200 s (~10 mL/s).
const unsigned long FLUSH_TIME_MS = 200000UL;
// Per your spec: flush pump ON while Solenoid 2 (SV2) is CLOSED.
// If the water doesn't flow into the jar, change this to true.
const bool FLUSH_SV2_OPEN_DURING_FLUSH = false;

bool flushSeqActive = false;
unsigned long flushStartMs = 0;

int currentState = STATE_IDLE;
bool plantRunning() {
  return (currentState == STATE_TEMP_WAIT || currentState == STATE_STEP3_SIMUL || currentState == STATE_FLUSH);
}

bool pBlueOn = false, pGreenOn = false, pYellowOn = false, pFlushOn = false;
bool vBlueOpen = false, vYellowOpen = false, vGreenOpen = false, vFlushOpen = false, v5Open = false;

WebServer server(80);

// ─── 5. FLUSH VALVE INTERLOCK + MANUAL OVERRIDE ──────────────────────────────
void updateFlushValve() {
  if (manualFlushActive) {
    // Direct manual override takes full priority
    setValve(PIN_VALVE_FLUSH, manualFlushVal);
    return;
  }

  // Automatic rule
  if (pFlushOn) {
    // Final flush stage uses its own SV2 setting; otherwise Flush Pump ON -> Flush Valve ON
    setValve(PIN_VALVE_FLUSH, flushSeqActive ? FLUSH_SV2_OPEN_DURING_FLUSH : true);
  } else if (pBlueOn) {
    setValve(PIN_VALVE_FLUSH, false); // Blue Pump ON  -> Flush Valve OFF
  } else {
    setValve(PIN_VALVE_FLUSH, false); // Rest all cases -> OFF
  }
}

// ─── 6. ACTUATOR CONTROLLERS ─────────────────────────────────────────────────
void setPump(int pumpIndex, bool on, int speed) {
  int pin = PIN_PUMP_BLUE;
  if (pumpIndex == 0)      { pin = PIN_PUMP_BLUE;   pBlueOn = on; }
  else if (pumpIndex == 1) { pin = PIN_PUMP_YELLOW; pYellowOn = on; }
  else if (pumpIndex == 2) { pin = PIN_PUMP_GREEN;  pGreenOn = on; }
  else if (pumpIndex == 3) { pin = PIN_PUMP_FLUSH;  pFlushOn = on; }

  ledcWrite(pin, on ? speed : 0);
  updateFlushValve();
}

void setValve(int valvePin, bool open) {
  digitalWrite(valvePin, open ? HIGH : LOW);
  if (valvePin == PIN_VALVE_BLUE)   vBlueOpen = open;
  if (valvePin == PIN_VALVE_YELLOW) vYellowOpen = open;
  if (valvePin == PIN_VALVE_GREEN)  vGreenOpen = open;
  if (valvePin == PIN_VALVE_FLUSH)  vFlushOpen = open;
  if (valvePin == PIN_VALVE_5)      v5Open = open;
}

void shutDownAll() {
  pBlueOn = false;
  pGreenOn = false;
  pYellowOn = false;
  pFlushOn = false;

  ledcWrite(PIN_PUMP_BLUE, 0);
  ledcWrite(PIN_PUMP_GREEN, 0);
  ledcWrite(PIN_PUMP_YELLOW, 0);
  ledcWrite(PIN_PUMP_FLUSH, 0);

  setValve(PIN_VALVE_BLUE, false);
  setValve(PIN_VALVE_YELLOW, false);
  setValve(PIN_VALVE_GREEN, false);
  setValve(PIN_VALVE_5, false);

  if (!manualFlushActive) {
    setValve(PIN_VALVE_FLUSH, false);
  }
}

// ─── 7. STATE TRANSITIONS ────────────────────────────────────────────────────
void enterState(int newState) {
  flushSeqActive = false;
  shutDownAll();
  delay(40);
  currentState = newState;

  switch (newState) {
    case STATE_IDLE:
      Serial.println("\n>>> PLANT STATUS: READY <<<");
      break;

    case STATE_TEMP_WAIT:
      forceTempPass = false;
      Serial.println("\n>>> STAGE 1: GREEN PUMP RUNNING FIRST (SV5 OFF) <<<");
      setValve(PIN_VALVE_5, false);
      setValve(PIN_VALVE_GREEN, true);
      delay(20);
      setPump(2, true, PWM_FAST);
      break;

    case STATE_STEP3_SIMUL:
      volBlue_mL = 0.0f; volGreen_mL = 0.0f; volYellow_mL = 0.0f;
      doneBlue = false; doneGreen = false; doneYellow = false;
      Serial.println("\n>>> STAGE 2: DELTA T REACHED! SV5 ON & ALL PUMPS START TOGETHER <<<");

      setValve(PIN_VALVE_5, true);
      setValve(PIN_VALVE_BLUE, true);
      setValve(PIN_VALVE_GREEN, true);
      setValve(PIN_VALVE_YELLOW, true);
      delay(20);

      setPump(0, true, PWM_FAST); // Blue Pump ON
      setPump(2, true, PWM_FAST); // Green Pump ON
      setPump(1, true, PWM_FAST); // Yellow Pump ON
      break;

    case STATE_FLUSH:
      Serial.println("\n>>> STAGE 0: FLUSHING 2 L CLEAR WATER FIRST (FLUSH PUMP ON, SV2 CLOSED) <<<");
      flushSeqActive = true;
      flushStartMs = millis();
      setPump(3, true, PWM_FAST); // Flush pump ON (SV2 set by updateFlushValve)
      break;

    case STATE_COMPLETED:
      Serial.println("\n>>> FLUSH + MIX COMPLETED <<<");
      break;
  }
}

// ─── 8. FAST RESPONSE JSON API ───────────────────────────────────────────────
void sendState() {
  String st = "Ready";
  if (currentState == STATE_TEMP_WAIT)        st = "Green Circulating (Wait dT >= 10C)";
  else if (currentState == STATE_STEP3_SIMUL) st = "Simultaneous Dosing (3L)";
  else if (currentState == STATE_FLUSH)       st = "Flushing 2L Clear Water";
  else if (currentState == STATE_COMPLETED)   st = "Complete (Safe State)";

  float totalTeal = volBlue_mL + volGreen_mL + volYellow_mL;

  char buf[340];
  snprintf(buf, sizeof(buf),
    "{\"state\":\"%s\",\"step\":%d,\"u7\":%s,\"ty\":%.1f,\"tg\":%.1f,\"dt\":%.1f,"
    "\"vb\":%.1f,\"vg\":%.1f,\"vy\":%.1f,\"total\":%.1f,\"sv5\":%s,\"sv_flush\":%s,"
    "\"m_act\":%s,\"m_val\":%s}",
    st.c_str(), currentState, u7Connected ? "true" : "false",
    tempYellow, tempGreen, deltaT,
    volBlue_mL, volGreen_mL, volYellow_mL, totalTeal,
    v5Open ? "true" : "false", vFlushOpen ? "true" : "false",
    manualFlushActive ? "true" : "false", manualFlushVal ? "true" : "false"
  );

  server.sendHeader("Connection", "close");
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", buf);
}

// ─── 9. WEB DASHBOARD WITH PROMINENT MANUAL CONTROL ──────────────────────────
const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Lam Research — 3L Mixer</title>
<style>
:root{--bg:#0b131b;--card:#142230;--blue:#38bdf8;--green:#4ade80;--yellow:#fde047;--teal:#14b8a6;--red:#f43f5e;--orange:#fb923c}
body{font:14px system-ui,sans-serif;background:var(--bg);color:#f1f5f9;margin:0;padding:12px}
main{max-width:680px;margin:auto}
section{background:var(--card);padding:14px;border-radius:12px;margin:10px 0;border:1px solid #1e3347}
h1{font-size:18px;margin:0 0 4px;color:var(--teal)}
.badge{padding:4px 10px;border-radius:20px;font-weight:700;font-size:11px}
.badge-ok{background:#065f46;color:#a7f3d0}
.badge-run{background:#854d0e;color:#fef08a;animation:pulse 1.2s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.5}}
.btn{font:700 13px inherit;padding:12px 14px;border:0;border-radius:8px;cursor:pointer;margin:3px;transition:transform 0.05s}
.btn:active{transform:scale(0.97)}
.btn-start{background:#059669;color:#fff;width:100%;font-size:15px}
.btn-stop{background:var(--red);color:#fff;width:100%;font-size:14px;margin-top:6px}
.btn-temp{background:var(--orange);color:#1e1b4b;font-size:12px;padding:6px 12px}
.btn-open{background:#059669;color:#fff;font-size:13px;padding:10px 14px}
.btn-close{background:#e11d48;color:#fff;font-size:13px;padding:10px 14px}
.btn-auto{background:#475569;color:#fff;font-size:13px;padding:10px 14px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.metric{background:#0b131b;padding:10px;border-radius:8px;border:1px solid #1e3347}
.metric b{font-size:17px;display:block;margin:2px 0}
.bar{background:#1e293b;border-radius:6px;height:8px;overflow:hidden;margin-top:4px}
.bar-fill{height:100%;width:0%;transition:width 0.2s}
.step-card{border-left:4px solid #64748b;padding-left:10px;margin:8px 0}
.step-active{border-left-color:var(--teal)}
</style>
</head>
<body>
<main>
  <h1>⚙ Lam Research: Automatic 3L Mixer</h1>

  <!-- DIRECT MANUAL FLUSH SOLENOID PANEL -->
  <section style="border:2px solid #38bdf8;background:#0f2438">
    <div style="font-size:15px;font-weight:700;color:#38bdf8;margin-bottom:8px">
      🚰 MANUAL FLUSH SOLENOID VALVE CONTROL (Pin 22 / SV2)
    </div>
    <div style="display:flex;gap:8px;flex-wrap:wrap">
      <button class="btn btn-open" onclick="instantReq('flush_valve?cmd=on')">🟢 TURN ON (OPEN)</button>
      <button class="btn btn-close" onclick="instantReq('flush_valve?cmd=off')">🔴 TURN OFF (CLOSE)</button>
      <button class="btn btn-auto" onclick="instantReq('flush_valve?cmd=auto')">🔄 RESET TO AUTO</button>
    </div>
    <div style="margin-top:10px;font-size:14px">
      Current Physical State: <b id="lblFlushState" style="font-size:16px;color:#f43f5e">CLOSED (LOW)</b><br>
      Active Control Mode: <b id="lblFlushMode" style="color:#94a3b8">Auto Mode</b>
    </div>
  </section>

  <section style="display:flex;justify-content:space-between;align-items:center">
    <div>Status: <span id="stateTxt" class="badge badge-ok">Ready</span></div>
    <div id="u7Status" style="font-size:12px">Checking U7...</div>
  </section>

  <section style="border-color:#0284c7">
    <button class="btn btn-start" onclick="instantReq('start')">▶ START AUTOMATION (Green Pump First)</button>
    <button class="btn btn-stop" onclick="instantReq('stop')">⏹ EMERGENCY STOP (All OFF)</button>
  </section>

  <!-- Stage 1 -->
  <section>
    <div class="step-card" id="cardStep1">
      <div style="display:flex;justify-content:space-between;align-items:center">
        <b>Stage 1: Green Pump Running (Wait ΔT ≥ 10.0 °C)</b>
        <button class="btn btn-temp" onclick="instantReq('pass_temp')">⚡ Force ΔT Trigger</button>
      </div>
      <div style="margin-top:8px">
        Yellow Tank (TS1): <b id="ty" style="color:var(--yellow)">25.0 °C</b> | 
        Green Tank (TS2): <b id="tg" style="color:var(--green)">25.0 °C</b><br>
        Measured ΔT: <b id="dt" style="font-size:18px;color:var(--orange)">0.0 °C</b> (Target: ≥ 10.0 °C)
      </div>
      <div style="margin-top:8px;font-size:13px">
        Solenoid 5 (G18): <b id="lblSV5" style="color:#f43f5e">OFF</b>
      </div>
    </div>

    <!-- Stage 2 -->
    <div class="step-card" id="cardStep2" style="margin-top:12px">
      <b>Stage 2: Simultaneous 3L Dosing (All Pumps ON + SV5 ON)</b>
      <div class="grid" style="margin-top:8px">
        <div class="metric">
          <span style="color:var(--blue)">Blue (60%) [1,800 mL]</span>
          <b id="vBlue" style="color:var(--blue)">0.0 / 1800 mL</b>
          <div class="bar"><div id="bBlue" class="bar-fill" style="background:var(--blue)"></div></div>
        </div>
        <div class="metric">
          <span style="color:var(--green)">Green (30%) [900 mL]</span>
          <b id="vGreen" style="color:var(--green)">0.0 / 900 mL</b>
          <div class="bar"><div id="bGreen" class="bar-fill" style="background:var(--green)"></div></div>
        </div>
        <div class="metric" style="grid-column: span 2">
          <span style="color:var(--yellow)">Yellow (10%) [300 mL]</span>
          <b id="vYellow" style="color:var(--yellow)">0.0 / 300 mL</b>
          <div class="bar"><div id="bYellow" class="bar-fill" style="background:var(--yellow)"></div></div>
        </div>
      </div>
      <div style="margin-top:8px;font-weight:700">Total Collected: <span id="txtTotal" style="color:#2dd4bf">0.0</span> / 3,000 mL</div>
    </div>
  </section>
</main>

<script>
const el = id => document.getElementById(id);
let isBusy = false;

function render(d){
  el('stateTxt').textContent = d.state;
  el('stateTxt').className = (d.step == 1 || d.step == 2) ? "badge badge-run" : "badge badge-ok";
  el('u7Status').innerHTML = d.u7 ? '<span style="color:#4ade80">● Telemetry Connected</span>' : '<span style="color:#f43f5e">● Waiting for U7...</span>';

  // Manual Flush Valve Status
  el('lblFlushState').textContent = d.sv_flush ? "OPEN (HIGH)" : "CLOSED (LOW)";
  el('lblFlushState').style.color = d.sv_flush ? "#4ade80" : "#f43f5e";

  el('lblFlushMode').textContent = d.m_act ? (d.m_val ? "MANUAL FORCED ON" : "MANUAL FORCED OFF") : "Auto Interlock Mode";
  el('lblFlushMode').style.color = d.m_act ? "#fb923c" : "#94a3b8";

  el('ty').textContent = d.ty.toFixed(1) + " °C";
  el('tg').textContent = d.tg.toFixed(1) + " °C";
  el('dt').textContent = d.dt.toFixed(1) + " °C";
  el('cardStep1').className = d.step == 1 ? "step-card step-active" : "step-card";

  el('vBlue').textContent   = d.vb.toFixed(1) + " / 1800 mL";
  el('vGreen').textContent  = d.vg.toFixed(1) + " / 900 mL";
  el('vYellow').textContent = d.vy.toFixed(1) + " / 300 mL";
  el('txtTotal').textContent = d.total.toFixed(1);

  el('bBlue').style.width   = Math.min(100, (d.vb/1800)*100) + "%";
  el('bGreen').style.width  = Math.min(100, (d.vg/900)*100) + "%";
  el('bYellow').style.width = Math.min(100, (d.vy/300)*100) + "%";
  el('cardStep2').className = d.step == 2 ? "step-card step-active" : "step-card";

  el('lblSV5').textContent = d.sv5 ? "ON (OPEN)" : "OFF (CLOSED)";
  el('lblSV5').style.color = d.sv5 ? "#4ade80" : "#f43f5e";
}

async function instantReq(action){
  try {
    const res = await fetch('/api/' + action, {method: 'POST'});
    if (res.ok) render(await res.json());
  } catch(e) {}
}

async function loopSync(){
  if (!isBusy) {
    isBusy = true;
    try {
      const res = await fetch('/api/state', {cache: 'no-store'});
      if (res.ok) render(await res.json());
    } catch(e) {}
    isBusy = false;
  }
  setTimeout(loopSync, 350);
}

loopSync();
</script>
</body>
</html>
)HTML";

// ─── 10. SETUP ───────────────────────────────────────────────────────────────
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  delay(200);

  Serial.begin(115200);

  // Solenoid Valves
  pinMode(PIN_VALVE_BLUE, OUTPUT);
  pinMode(PIN_VALVE_GREEN, OUTPUT);
  pinMode(PIN_VALVE_YELLOW, OUTPUT);
  pinMode(PIN_VALVE_FLUSH, OUTPUT);
  pinMode(PIN_VALVE_5, OUTPUT);

  gpio_set_drive_capability((gpio_num_t)PIN_VALVE_BLUE, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_VALVE_GREEN, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_VALVE_YELLOW, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_VALVE_FLUSH, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_VALVE_5, GPIO_DRIVE_CAP_3);

  // Pumps
  pinMode(PIN_PUMP_BLUE, OUTPUT);
  pinMode(PIN_PUMP_GREEN, OUTPUT);
  pinMode(PIN_PUMP_YELLOW, OUTPUT);
  pinMode(PIN_PUMP_FLUSH, OUTPUT);

  ledcAttach(PIN_PUMP_BLUE, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_PUMP_GREEN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_PUMP_YELLOW, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PIN_PUMP_FLUSH, PWM_FREQ, PWM_RESOLUTION);

  shutDownAll();

  pinMode(PIN_SW, INPUT_PULLUP);

  // Wi-Fi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_17dBm);
  IPAddress local_ip(192, 168, 4, 1), gateway(192, 168, 4, 1), subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP("Lam_Teal_Mixer", "CHANGE_ME_123");

  server.on("/", HTTP_GET, []{
    server.sendHeader("Connection", "close");
    server.send_P(200, "text/html", PAGE);
  });

  server.on("/api/state", HTTP_GET, sendState);

  server.on("/api/start", HTTP_POST, []{
    if (!plantRunning()) enterState(STATE_FLUSH);   // sequence now starts with flush
    sendState();
  });

  server.on("/api/stop", HTTP_POST, []{
    enterState(STATE_IDLE);
    sendState();
  });

  server.on("/api/pass_temp", HTTP_POST, []{
    forceTempPass = true;
    sendState();
  });

  // Dedicated Manual Flush Valve Control Endpoint
  server.on("/api/flush_valve", HTTP_POST, []{
    String cmd = server.arg("cmd");
    if (cmd == "on") {
      manualFlushActive = true;
      manualFlushVal = true;
      Serial.println("[MANUAL]: Flush Solenoid Valve FORCED ON");
    } else if (cmd == "off") {
      manualFlushActive = true;
      manualFlushVal = false;
      Serial.println("[MANUAL]: Flush Solenoid Valve FORCED OFF");
    } else if (cmd == "auto") {
      manualFlushActive = false;
      Serial.println("[MANUAL]: Flush Solenoid Valve RESET TO AUTO");
    }
    updateFlushValve();
    sendState();
  });

  // Flow telemetry from U7
  server.on("/api/flow", HTTP_POST, []{
    uint32_t boot = strtoul(server.arg("boot").c_str(), NULL, 10);
    uint32_t rawB = strtoul(server.arg("b").c_str(), NULL, 10);
    uint32_t rawY = strtoul(server.arg("y").c_str(), NULL, 10);
    uint32_t rawG = strtoul(server.arg("g").c_str(), NULL, 10);

    if (!haveBoot || boot != lastBootId) {
      lastBootId = boot;
      haveBoot = true;
      lastRawBlue = rawB; lastRawGreen = rawG; lastRawYellow = rawY;
    }

    uint32_t dB = rawB - lastRawBlue;
    uint32_t dG = rawG - lastRawGreen;
    uint32_t dY = rawY - lastRawYellow;
    lastRawBlue = rawB; lastRawGreen = rawG; lastRawYellow = rawY;

    if (server.hasArg("ty")) tempYellow = server.arg("ty").toFloat();
    if (server.hasArg("tg")) tempGreen  = server.arg("tg").toFloat();
    deltaT = fabsf(tempGreen - tempYellow);

    u7Connected = true;
    lastPacketTime = millis();

    if (currentState == STATE_STEP3_SIMUL) {
      if (!doneBlue)   volBlue_mL   += dB * 1000.0f / pplBlue;
      if (!doneGreen)  volGreen_mL  += dG * 1000.0f / pplGreen;
      if (!doneYellow) volYellow_mL += dY * 1000.0f / pplYellow;
    }

    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "ok");
  });

  server.begin();
  Serial.println("\nU2 CONTROLLER ONLINE (MANUAL FLUSH VALVE TEST PANEL ACTIVE)");
}

// ─── 11. MAIN AUTOMATION LOOP ────────────────────────────────────────────────
void loop() {
  server.handleClient();

  if (u7Connected && (millis() - lastPacketTime > U7_TIMEOUT_MS)) {
    u7Connected = false;
  }

  // Push button start
  static bool prevBtn = false;
  bool btnPressed = (digitalRead(PIN_SW) == LOW);
  if (btnPressed && !prevBtn && currentState == STATE_IDLE) {
    enterState(STATE_FLUSH);   // sequence now starts with flush
  }
  prevBtn = btnPressed;

  switch (currentState) {

    case STATE_TEMP_WAIT: {
      if (forceTempPass || deltaT >= TARGET_DELTAT) {
        Serial.printf("\n[TRIGGER]: Delta T = %.1f C reached! Starting 3L Batch...\n", deltaT);
        enterState(STATE_STEP3_SIMUL);
      }
      break;
    }

    case STATE_STEP3_SIMUL: {

      // Blue (1,800 mL / 60%)
      if (!doneBlue) {
        if (volBlue_mL >= 1650.0f) setPump(0, true, PWM_SLOW);
        if (volBlue_mL >= TARGET_BLUE) {
          setPump(0, false);
          setValve(PIN_VALVE_BLUE, false);
          doneBlue = true;
          Serial.println("[BATCH] Blue Done (1800 mL)");
        }
      }

      // Green (900 mL / 30%)
      if (!doneGreen) {
        if (volGreen_mL >= 800.0f) setPump(2, true, PWM_SLOW);
        if (volGreen_mL >= TARGET_GREEN) {
          setPump(2, false);
          setValve(PIN_VALVE_GREEN, false);
          doneGreen = true;
          Serial.println("[BATCH] Green Done (900 mL)");
        }
      }

      // Yellow (300 mL / 10%)
      if (!doneYellow) {
        if (volYellow_mL >= 250.0f) setPump(1, true, PWM_SLOW);
        if (volYellow_mL >= TARGET_YELLOW) {
          setPump(1, false);
          setValve(PIN_VALVE_YELLOW, false);
          doneYellow = true;
          Serial.println("[BATCH] Yellow Done (300 mL)");
        }
      }

      if (doneBlue && doneGreen && doneYellow) {
        enterState(STATE_COMPLETED);   // all colours done -> finished
      }
      break;
    }

    case STATE_FLUSH: {
      if (millis() - flushStartMs >= FLUSH_TIME_MS) {
        Serial.println("[FLUSH] 2 L flush done -> starting temperature stage");
        enterState(STATE_TEMP_WAIT);
      }
      break;
    }

    default:
      break;
  }

  yield();
}