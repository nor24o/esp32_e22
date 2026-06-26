#pragma once
#include <pgmspace.h>

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>LoRa Tank Test</title>
<style>
  :root {
    --bg:    #0d1117;
    --card:  #161b22;
    --bd:    #30363d;
    --tx:    #c9d1d9;
    --tx2:   #8b949e;
    --green: #3fb950;
    --red:   #f85149;
    --blue:  #58a6ff;
    --yel:   #d29922;
    --pur:   #bc8cff;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--tx); font-family: 'Segoe UI', system-ui, sans-serif; font-size: 14px; }

  /* ── Header ── */
  header {
    background: var(--card);
    border-bottom: 1px solid var(--bd);
    padding: 10px 18px;
    display: flex;
    align-items: center;
    gap: 12px;
    flex-wrap: wrap;
  }
  header h1 { font-size: 15px; font-weight: 600; flex: 1; }
  .ws-dot { width: 9px; height: 9px; border-radius: 50%; background: var(--red); flex-shrink: 0; }
  .ws-dot.connected { background: var(--green); }
  .badge {
    font-size: 11px; font-weight: 600; padding: 2px 8px;
    border-radius: 12px; letter-spacing: .5px;
  }
  .badge-role { background: #1f6feb33; color: var(--blue); border: 1px solid #1f6feb66; }
  .badge-err  { background: #f8514922; color: var(--red);  border: 1px solid #f8514944; display: none; }
  .badge-err.show { display: inline; }
  .hdr-stat { font-size: 12px; color: var(--tx2); }
  #last-seen { min-width: 90px; }

  /* ── Grid ── */
  main { display: grid; grid-template-columns: 1fr 1fr; gap: 14px; padding: 14px; max-width: 1100px; margin: auto; }
  @media (max-width: 680px) { main { grid-template-columns: 1fr; } }

  /* ── Card ── */
  .card {
    background: var(--card);
    border: 1px solid var(--bd);
    border-radius: 8px;
    padding: 14px;
  }
  .card h2 { font-size: 13px; font-weight: 600; color: var(--tx2); text-transform: uppercase; letter-spacing: .5px; margin-bottom: 12px; }

  /* ── Tank visual ── */
  .tank-wrap { display: flex; align-items: flex-end; gap: 18px; }
  .tank-visual {
    width: 48px; height: 120px;
    border: 2px solid var(--bd);
    border-radius: 4px 4px 0 0;
    position: relative; overflow: hidden; flex-shrink: 0;
  }
  .tank-fill {
    position: absolute; bottom: 0; left: 0; right: 0;
    background: var(--blue); opacity: .7;
    transition: height .6s ease;
    height: 0%;
  }
  .tank-label {
    position: absolute; bottom: 3px; left: 0; right: 0;
    text-align: center; font-size: 12px; font-weight: 700; color: #fff; z-index: 1;
  }
  .tank-info { flex: 1; display: flex; flex-direction: column; gap: 7px; }
  .info-row { display: flex; justify-content: space-between; align-items: center; }
  .info-key { color: var(--tx2); font-size: 12px; }
  .info-val { font-weight: 600; font-size: 13px; }

  /* pump dots */
  .pump-dots { display: flex; gap: 6px; }
  .pump-dot {
    width: 12px; height: 12px; border-radius: 50%;
    background: var(--bd); border: 1px solid var(--bd);
    position: relative;
  }
  .pump-dot.on { background: var(--green); border-color: var(--green); }
  .pump-dot span { position: absolute; top: 13px; left: -3px; font-size: 10px; color: var(--tx2); white-space: nowrap; }

  /* ── Controls ── */
  .btn-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-bottom: 10px; }
  .btn {
    padding: 7px 10px; border-radius: 6px; border: 1px solid var(--bd);
    background: transparent; color: var(--tx); font-size: 12px; font-weight: 600;
    cursor: pointer; transition: background .15s, border-color .15s;
  }
  .btn:hover { background: #ffffff11; }
  .btn.on  { background: #3fb95022; border-color: var(--green); color: var(--green); }
  .btn.off { background: #f8514922; border-color: var(--red);   color: var(--red);   }
  .btn.act { background: #58a6ff22; border-color: var(--blue);  color: var(--blue);  }
  .btn-row { display: flex; gap: 6px; margin-top: 4px; }
  .btn-row .btn { flex: 1; }

  .section-label { font-size: 11px; color: var(--tx2); text-transform: uppercase; letter-spacing: .5px; margin: 8px 0 4px; }
  .pond-state { font-size: 12px; color: var(--tx2); margin-bottom: 6px; }
  .pond-state span { font-weight: 600; color: var(--tx); }

  /* ── Tables (config / stats) ── */
  table { width: 100%; border-collapse: collapse; font-size: 12px; }
  th, td { padding: 4px 6px; text-align: left; border-bottom: 1px solid var(--bd); }
  th { color: var(--tx2); font-weight: 600; }
  td:last-child { text-align: right; font-weight: 600; color: var(--blue); }
  .empty-note { color: var(--tx2); font-size: 12px; text-align: center; padding: 16px 0; }

  /* ── Event log ── */
  #log {
    height: 200px; overflow-y: auto; font-size: 12px; font-family: monospace;
    background: #0a0e13; border: 1px solid var(--bd); border-radius: 6px;
    padding: 6px 8px; display: flex; flex-direction: column;
  }
  .log-entry { padding: 1px 0; white-space: pre-wrap; word-break: break-all; }
  .log-rx  { color: var(--green); }
  .log-tx  { color: var(--blue); }
  .log-err { color: var(--red); }
  .log-inf { color: var(--tx2); }
  .log-cmd { color: var(--pur); }
  .log-hdr { color: var(--yel); }
  .log-clear-btn { font-size: 11px; color: var(--tx2); background: transparent; border: none; cursor: pointer; margin-left: auto; }
  .log-header { display: flex; align-items: center; margin-bottom: 4px; }

  /* ── Misc ── */
  .full-col { grid-column: 1 / -1; }
  .chip {
    font-size: 11px; padding: 1px 6px; border-radius: 4px;
    background: #30363d; color: var(--tx2); display: inline-block;
  }
  .chip.ok  { background: #3fb95022; color: var(--green); }
  .chip.err { background: #f8514922; color: var(--red);   }
  .chip.unk { background: #30363d;   color: var(--tx2);   }

  /* ── Config form ── */
  .cfg-row { display: flex; align-items: center; gap: 8px; padding: 5px 0; border-bottom: 1px solid var(--bd); }
  .cfg-row:last-child { border-bottom: none; }
  .cfg-name { flex: 1; font-size: 12px; color: var(--tx2); }
  .cfg-inp {
    width: 80px; padding: 3px 6px;
    background: var(--bg); border: 1px solid var(--bd); border-radius: 4px;
    color: var(--tx); font-size: 12px; text-align: right;
  }
  .cfg-inp:focus { outline: none; border-color: var(--blue); }
  .cfg-inp.dirty { border-color: var(--yel); color: var(--yel); }
  .cfg-unit { font-size: 11px; color: var(--tx2); width: 14px; }
  .cfg-hint { font-size: 10px; color: var(--tx2); white-space: nowrap; min-width: 64px; text-align: right; }
  .cfg-bar  { display: flex; align-items: center; gap: 8px; margin-top: 10px; flex-wrap: wrap; }
  .cfg-status { font-size: 11px; color: var(--tx2); }
</style>
</head>
<body>

<header>
  <div class="ws-dot" id="wsDot"></div>
  <h1>LoRa Tank Monitor</h1>
  <span class="badge badge-role" id="roleBadge">DUAL</span>
  <span class="badge badge-err"  id="errBadge">ERR</span>
  <span class="hdr-stat">RSSI&nbsp;<strong id="hdrRssi">—</strong>&nbsp;dBm</span>
  <span class="hdr-stat">SNR&nbsp;<strong id="hdrSnr">—</strong>&nbsp;dB</span>
  <span class="hdr-stat" id="last-seen">No&nbsp;data</span>
  <span class="hdr-stat">Heap&nbsp;<strong id="hdrHeap">—</strong></span>
</header>

<main>

  <!-- ── Tank Status ── -->
  <div class="card" id="cardTank">
    <h2>Tank Status</h2>
    <div class="tank-wrap">
      <div class="tank-visual">
        <div class="tank-fill" id="tankFill"></div>
        <div class="tank-label" id="tankLvlTxt">—</div>
      </div>
      <div class="tank-info">
        <div class="info-row">
          <span class="info-key">Mode</span>
          <span class="info-val" id="tMode">—</span>
        </div>
        <div class="info-row">
          <span class="info-key">Pumps</span>
          <div class="pump-dots" id="pumpDots">
            <div class="pump-dot" id="pd1"><span>P1</span></div>
            <div class="pump-dot" id="pd2"><span>P2</span></div>
            <div class="pump-dot" id="pd3"><span>P3</span></div>
            <div class="pump-dot" id="pdP"><span>Pond</span></div>
          </div>
        </div>
        <div class="info-row">
          <span class="info-key">Temp</span>
          <span class="info-val" id="tTemp">—</span>
        </div>
        <div class="info-row">
          <span class="info-key">Humidity</span>
          <span class="info-val" id="tHum">—</span>
        </div>
        <div class="info-row">
          <span class="info-key">Error</span>
          <span class="info-val" id="tErr">—</span>
        </div>
        <div class="info-row">
          <span class="info-key">Node RSSI/SNR</span>
          <span class="info-val" id="tNodeRssi">—</span>
        </div>
        <div class="info-row">
          <span class="info-key">Link RSSI/SNR</span>
          <span class="info-val" id="tLinkRssi">—</span>
        </div>
      </div>
    </div>
  </div>

  <!-- ── Controls ── -->
  <div class="card">
    <h2>Controls</h2>

    <div class="section-label">Pump Commands <span style="font-weight:400;text-transform:none;letter-spacing:0">(gateway 0x01 · always obeyed)</span></div>
    <div class="btn-grid">
      <button class="btn on"  onclick="sendCmd('pump','p=1&a=1')">P1 ON</button>
      <button class="btn off" onclick="sendCmd('pump','p=1&a=0')">P1 OFF</button>
      <button class="btn on"  onclick="sendCmd('pump','p=2&a=1')">P2 ON</button>
      <button class="btn off" onclick="sendCmd('pump','p=2&a=0')">P2 OFF</button>
      <button class="btn on"  onclick="sendCmd('pump','p=3&a=1')">P3 ON</button>
      <button class="btn off" onclick="sendCmd('pump','p=3&a=0')">P3 OFF</button>
      <button class="btn on"  onclick="sendCmd('pump','p=4&a=1')">Pond ON</button>
      <button class="btn off" onclick="sendCmd('pump','p=4&a=0')">Pond OFF</button>
    </div>
    <div class="btn-row">
      <button class="btn act" onclick="sendCmd('cfg_get','')">Get Config</button>
      <button class="btn act" onclick="sendCmd('stats_get','')">Get Stats</button>
      <button class="btn act" onclick="sendCmd('keepalive','')">Keepalive</button>
    </div>

    <div class="section-label" style="margin-top:14px">Pond Node Simulation <span style="font-weight:400;text-transform:none;letter-spacing:0">(pond 0x03)</span></div>
    <div class="pond-state">Pond pump: <span id="pondPumpState">OFF</span></div>
    <button class="btn act" onclick="sendCmd('telemetry','')" style="width:100%">Send Telemetry Now</button>
    <div style="margin-top:6px;font-size:11px;color:var(--tx2)">Records pump cmds from tank &bull; reports state in next telemetry &bull; keepalive every 30s &bull; telemetry every 10s</div>
  </div>

  <!-- ── Config ── -->
  <div class="card">
    <div style="display:flex;align-items:center;gap:8px;margin-bottom:12px">
      <h2 style="margin:0">Config</h2>
      <span class="chip" id="cfgChip">waiting...</span>
    </div>
    <div id="cfgBody"></div>
    <div class="cfg-bar" id="cfgBar">
      <button class="btn" id="cfgResetBtn" onclick="resetCfgDefaults()">Defaults</button>
      <button class="btn act" id="cfgSetBtn" onclick="sendCfgSet()" disabled>Set Config</button>
      <span class="cfg-status" id="cfgStatus"></span>
    </div>
  </div>

  <!-- ── Stats ── -->
  <div class="card">
    <h2>Stats <span class="chip" id="statsChip">waiting...</span></h2>
    <div id="statsBody"><div class="empty-note">Send "Get Stats" to populate</div></div>
  </div>

  <!-- ── Event Log ── -->
  <div class="card full-col">
    <div class="log-header">
      <h2 style="margin:0">Event Log</h2>
      <button class="log-clear-btn" onclick="clearLog()">Clear</button>
    </div>
    <div id="log"></div>
  </div>

</main>

<script>
'use strict';

let ws = null;
let reconnectTimer = null;
let lastSeenTimer  = null;
let lastSeenMs     = 0;
const MAX_LOG = 120;

// ── WebSocket ──────────────────────────────────────────────────────────────

function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = () => {
    setDot(true);
    addLog('inf', 'WebSocket connected');
    document.getElementById('cfgSetBtn').disabled = false;
    clearTimeout(reconnectTimer);
  };
  ws.onclose = () => {
    setDot(false);
    addLog('err', 'WebSocket disconnected — reconnecting in 2 s...');
    document.getElementById('cfgSetBtn').disabled = true;
    reconnectTimer = setTimeout(connect, 2000);
  };
  ws.onerror = () => {
    addLog('err', 'WebSocket error');
  };
  ws.onmessage = e => {
    try { handleMsg(JSON.parse(e.data)); }
    catch(ex) { addLog('err', 'Bad JSON: ' + e.data); }
  };
}

function setDot(ok) {
  document.getElementById('wsDot').className = 'ws-dot' + (ok ? ' connected' : '');
}

// ── Message handler ────────────────────────────────────────────────────────

function handleMsg(d) {
  switch (d.t) {

    case 'telem': {
      lastSeenMs = Date.now();
      const lvl  = d.wl;
      const pct  = Math.round((lvl / 3) * 100);
      document.getElementById('tankFill').style.height   = pct + '%';
      document.getElementById('tankLvlTxt').textContent  = lvl + '/3';
      document.getElementById('tMode').textContent       = d.auto ? 'AUTO' : 'MANUAL';
      setPumpDot('pd1', d.p1);
      setPumpDot('pd2', d.p2);
      setPumpDot('pd3', d.p3);
      setPumpDot('pdP', d.pond);
      document.getElementById('tTemp').textContent       = (d.tmp / 10).toFixed(1) + ' °C';
      document.getElementById('tHum').textContent        = d.hum + ' %';
      const errStr = ['OK','OVERCURRENT','DRY-RUN','NO-COMMS'][d.err] || ('ERR ' + d.err);
      document.getElementById('tErr').textContent        = errStr;
      const errBadge = document.getElementById('errBadge');
      errBadge.textContent = errStr;
      errBadge.className   = 'badge badge-err' + (d.err ? ' show' : '');
      document.getElementById('tNodeRssi').textContent   = d.nrssi + ' dBm / ' + d.nsnr + ' dB';
      document.getElementById('tLinkRssi').textContent   = d.rssi  + ' dBm / ' + d.snr  + ' dB';
      document.getElementById('hdrRssi').textContent     = d.rssi;
      document.getElementById('hdrSnr').textContent      = d.snr;
      addLog('rx', `TELEM  wl=${lvl}  auto=${d.auto}  P1=${d.p1} P2=${d.p2} P3=${d.p3} Pond=${d.pond}  ${(d.tmp/10).toFixed(1)}°C  err=${d.err}  rssi=${d.rssi}`);
      break;
    }

    case 'cfg': {
      buildCfgForm(d);
      document.getElementById('cfgChip').textContent   = 'live · rssi=' + d.rssi + ' snr=' + d.snr;
      document.getElementById('cfgChip').className     = 'chip ok';
      document.getElementById('cfgSetBtn').disabled    = false;
      document.getElementById('cfgStatus').textContent = '';
      addLog('rx', `CONFIG_RESP  pmr=${d.pmr/1000}s pmc=${d.pmc/1000}s pro=${d.pro/1000}s ti=${d.ti/1000}s nt=${d.nt/1000}s crt=${d.crt/1000}s bam=${d.bam}  rssi=${d.rssi}`);
      break;
    }

    case 'stats': {
      const rows = [
        ['uptime',        fmtSec(d.up)],
        ['P1 runtime',    fmtSec(d.rt1)],
        ['P2 runtime',    fmtSec(d.rt2)],
        ['P3 runtime',    fmtSec(d.rt3)],
        ['Pond runtime',  fmtSec(d.rtp)],
        ['fill_cycles',   d.fc],
        ['boot_count',    d.bc],
        ['last_fault',    d.lf],
      ];
      document.getElementById('statsBody').innerHTML = buildTable(rows);
      document.getElementById('statsChip').textContent = 'rssi=' + d.rssi + ' snr=' + d.snr;
      document.getElementById('statsChip').className   = 'chip ok';
      addLog('rx', `STATS_RESP  uptime=${fmtSec(d.up)}  rssi=${d.rssi}`);
      break;
    }

    case 'cmd':
      addLog('rx', `COMMAND  pump=${d.pump}  action=${d.action ? 'ON' : 'OFF'}  id=${d.id}  rssi=${d.rssi}`);
      // If the command is to the pond pump, update the displayed pond state.
      if (d.pump === 1) {
        document.getElementById('pondPumpState').textContent = d.action ? 'ON' : 'OFF';
      }
      break;

    case 'tx':
      addLog('tx', `TX  type=${d.mtype}  to=0x${d.target.toString(16).padStart(2,'0')}  id=${d.id}`);
      break;

    case 'log':
      addLog(d.lvl, d.msg);
      break;

    case 'role':
      addLog('inf', 'Mode: ' + (d.role || 'dual').toUpperCase());
      break;

    case 'status':
      document.getElementById('hdrHeap').textContent = Math.round(d.heap / 1024) + 'k';
      break;
  }
}

// ── UI helpers ─────────────────────────────────────────────────────────────

function setPumpDot(id, on) {
  const el = document.getElementById(id);
  if (el) el.className = 'pump-dot' + (on ? ' on' : '');
}

function buildTable(rows) {
  let h = '<table><tr><th>Parameter</th><th>Value</th></tr>';
  for (const [k, v] of rows) h += `<tr><td>${k}</td><td>${v}</td></tr>`;
  return h + '</table>';
}

function fmtSec(s) {
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  return h ? `${h}h ${m}m` : m ? `${m}m ${sec}s` : `${sec}s`;
}

function addLog(lvl, msg) {
  const log = document.getElementById('log');
  const ts  = new Date().toLocaleTimeString();
  const div = document.createElement('div');
  div.className = 'log-entry log-' + lvl;
  div.textContent = `[${ts}] ${msg}`;
  log.insertBefore(div, log.firstChild);
  while (log.children.length > MAX_LOG) log.removeChild(log.lastChild);
}

function clearLog() {
  document.getElementById('log').innerHTML = '';
}

// last-seen ticker
setInterval(() => {
  if (!lastSeenMs) return;
  const s = Math.round((Date.now() - lastSeenMs) / 1000);
  document.getElementById('last-seen').textContent = 'Last rx ' + s + 's ago';
}, 1000);

// ── Commands ───────────────────────────────────────────────────────────────

function sendCmd(cmd, params) {
  if (!ws || ws.readyState !== WebSocket.OPEN) { addLog('err', 'Not connected'); return; }
  let msg;
  if (cmd === 'pump') {
    const p = new URLSearchParams(params);
    msg = JSON.stringify({ cmd: 'pump', p: parseInt(p.get('p')), a: parseInt(p.get('a')) });
  } else {
    msg = JSON.stringify({ cmd });
  }
  ws.send(msg);
  addLog('cmd', '→ ' + msg);
}

// ── Config editor ─────────────────────────────────────────────────────────

// Raw ms/unit values — mirrors DEF_* in config.hpp
const CFG_DEFAULTS = { pmr:30000, pmc:60000, pro:300000, ti:10000, nt:60000, crt:15000, bam:1 };

const CFG_FIELDS = [
  { key:'pmr', label:'Pump min runtime',        ms:true, min:5,   max:3600,  def:30  },
  { key:'pmc', label:'Pump min cooldown',       ms:true, min:5,   max:3600,  def:60  },
  { key:'pro', label:'Replenish run-on',        ms:true, min:30,  max:86400, def:300 },
  { key:'ti',  label:'Telemetry interval',      ms:true, min:5,   max:3600,  def:10  },
  { key:'nt',  label:'Network timeout',         ms:true, min:10,  max:3600,  def:60  },
  { key:'crt', label:'Cmd response timeout',    ms:true, min:5,   max:120,   def:15  },
  { key:'bam', label:'Boot auto mode',          bool:true,                   def:1   },
];

function buildCfgForm(d) {
  let h = '';
  for (const f of CFG_FIELDS) {
    const raw = d[f.key];
    if (f.bool) {
      h += `<div class="cfg-row">
        <span class="cfg-name">${f.label}</span>
        <select class="cfg-inp" id="cfg_${f.key}" onchange="this.classList.add('dirty')">
          <option value="1"${raw ? ' selected' : ''}>YES</option>
          <option value="0"${!raw ? ' selected' : ''}>NO</option>
        </select>
        <span class="cfg-unit"></span>
        <span class="cfg-hint">YES / NO</span>
      </div>`;
    } else {
      const disp = f.ms ? raw / 1000 : raw;
      h += `<div class="cfg-row">
        <span class="cfg-name">${f.label}</span>
        <input class="cfg-inp" type="number" id="cfg_${f.key}"
               value="${disp}" min="${f.min}" max="${f.max}" step="1"
               onchange="this.classList.add('dirty')">
        <span class="cfg-unit">${f.ms ? 's' : ''}</span>
        <span class="cfg-hint">${f.min}–${f.max}</span>
      </div>`;
    }
  }
  document.getElementById('cfgBody').innerHTML = h;
}

function sendCfgSet() {
  if (!ws || ws.readyState !== WebSocket.OPEN) { addLog('err', 'Not connected'); return; }
  if (!document.getElementById('cfg_pmr')) { addLog('err', 'Load config first (Get Config)'); return; }

  const msg = { cmd: 'cfg_set' };
  for (const f of CFG_FIELDS) {
    const el = document.getElementById('cfg_' + f.key);
    if (!el) { addLog('err', 'Missing field: ' + f.key); return; }
    if (f.bool) {
      msg[f.key] = parseInt(el.value);
    } else {
      msg[f.key] = f.ms ? Math.round(parseFloat(el.value) * 1000) : Math.round(parseFloat(el.value));
    }
  }

  // Client-side validation mirrors tank's validateConfig() in nvs_manager.hpp
  const checks = [
    [msg.pmr, 5000,   3600000,   'pump_min_runtime (5–3600 s)'],
    [msg.pmc, 5000,   3600000,   'pump_min_cooldown (5–3600 s)'],
    [msg.pro, 30000,  86400000,  'replenish_runon (30–86400 s)'],
    [msg.ti,  5000,   3600000,   'telemetry_interval (5–3600 s)'],
    [msg.nt,  10000,  3600000,   'network_timeout (10–3600 s)'],
    [msg.crt, 5000,   120000,    'cmd_response_timeout (5–120 s)'],
  ];
  for (const [v, lo, hi, name] of checks) {
    if (!Number.isFinite(v) || v < lo || v > hi) { addLog('err', name + ' out of range'); return; }
  }

  const json = JSON.stringify(msg);
  ws.send(json);
  addLog('cmd', '→ ' + json);
  document.getElementById('cfgSetBtn').disabled = true;
  document.getElementById('cfgStatus').textContent = 'Sent — waiting for CONFIG_RESP...';
}

function resetCfgDefaults() {
  buildCfgForm(CFG_DEFAULTS);
  document.getElementById('cfgChip').textContent  = 'defaults';
  document.getElementById('cfgChip').className    = 'chip unk';
  document.getElementById('cfgStatus').textContent = '';
}

// ── Boot ───────────────────────────────────────────────────────────────────
buildCfgForm(CFG_DEFAULTS);
document.getElementById('cfgChip').textContent = 'defaults';
document.getElementById('cfgChip').className   = 'chip unk';
connect();
</script>
</body>
</html>
)HTML";
