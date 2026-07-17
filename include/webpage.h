#pragma once
#include <Arduino.h>

// Single self-contained control page. Served from flash. Talks to:
//   GET  /state       current fan settings + live temp/RPM
//   GET  /set?...     change persisted settings (intake / exhaust / auto)
//   POST /update      firmware upload (HTTP basic auth, user "admin")
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Fan Controller</title>
<style>
  :root { --bg:#0e0f13; --card:#191b22; --line:#2a2d38; --fg:#e8eaf0; --mut:#8a90a2; --accent:#ff7800; }
  * { box-sizing:border-box; -webkit-tap-highlight-color:transparent; }
  body { margin:0; min-height:100vh; background:var(--bg); color:var(--fg);
         font:16px/1.5 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;
         display:flex; justify-content:center; padding:24px 16px; }
  .wrap { width:100%; max-width:420px; }
  h1 { font-size:15px; font-weight:600; letter-spacing:.14em; text-transform:uppercase;
       color:var(--mut); margin:0 0 20px; text-align:center; }
  .hero { font-variant-numeric:tabular-nums; font-weight:700; text-align:center;
          font-size:64px; letter-spacing:.02em; margin:4px 0 24px; }
  .hero small { font-size:20px; color:var(--mut); font-weight:600; margin-left:4px; }
  .card { background:var(--card); border:1px solid var(--line); border-radius:16px;
          padding:20px; margin-bottom:16px; }
  .row { display:flex; align-items:center; justify-content:space-between; gap:16px; }
  .row + .row { margin-top:16px; }
  label { color:var(--mut); font-size:14px; }
  .val { font-variant-numeric:tabular-nums; color:var(--fg); font-size:14px; min-width:34px; text-align:right; }
  input[type=range] { -webkit-appearance:none; appearance:none; width:100%; height:6px;
    background:var(--line); border-radius:999px; outline:none; }
  input[type=range]::-webkit-slider-thumb { -webkit-appearance:none; width:22px; height:22px;
    border-radius:50%; background:var(--accent); cursor:pointer; border:3px solid var(--card); }
  input[type=range]::-moz-range-thumb { width:22px; height:22px; border-radius:50%;
    background:var(--accent); cursor:pointer; border:3px solid var(--card); }
  input[type=range]:disabled { opacity:.45; }
  input[type=range]:disabled::-webkit-slider-thumb { cursor:default; }
  .brow { display:block; }
  .brow .top { display:flex; justify-content:space-between; margin-bottom:12px; }
  .rpm { display:flex; justify-content:space-between; margin-top:16px; gap:12px; }
  .rpm .cell { flex:1; background:var(--bg); border:1px solid var(--line); border-radius:10px;
    padding:10px 12px; text-align:center; }
  .rpm .num { font-variant-numeric:tabular-nums; font-weight:700; font-size:20px; color:var(--accent); }
  .rpm .cap { color:var(--mut); font-size:12px; margin-top:2px; }
  .seg { display:flex; background:var(--bg); border:1px solid var(--line); border-radius:10px; padding:3px; }
  .seg button { flex:1; border:none; background:none; color:var(--mut); font:inherit; font-size:14px;
    padding:7px 12px; border-radius:8px; cursor:pointer; }
  .seg button.on { background:var(--accent); color:#111; font-weight:600; }
  .foot { text-align:center; color:var(--mut); font-size:12px; margin-top:8px; }
  .foot code { color:var(--fg); }
  .upd { display:flex; gap:10px; align-items:center; margin-top:14px; flex-wrap:wrap; }
  .upd input[type=file] { flex:1; min-width:0; color:var(--mut); font-size:13px; }
  .upd button { border:none; background:var(--accent); color:#111; font:inherit; font-weight:600;
    padding:9px 16px; border-radius:9px; cursor:pointer; }
  .upd button:disabled { opacity:.5; cursor:default; }
  .ustat { color:var(--mut); font-size:13px; margin-top:8px; min-height:18px; }
</style>
</head>
<body>
  <div class="wrap">
    <h1>Fan Controller</h1>
    <div class="hero"><span id="temp">--</span><small>&deg;C</small></div>

    <div class="card">
      <div class="row">
        <label>Auto mode<span id="aact"></span></label>
        <div class="seg" id="auto">
          <button data-a="0">Off</button>
          <button data-a="1">On</button>
        </div>
      </div>
    </div>

    <div class="card">
      <div class="brow">
        <div class="top"><label>Intake speed</label><span class="val" id="ival">--</span></div>
        <input type="range" id="intake" min="0" max="255" step="1">
      </div>
      <div class="rpm">
        <div class="cell"><div class="num" id="rpm1">--</div><div class="cap">Fan 1 RPM</div></div>
        <div class="cell"><div class="num" id="rpm2">--</div><div class="cap">Fan 2 RPM</div></div>
      </div>
    </div>

    <div class="card">
      <div class="brow">
        <div class="top"><label>Exhaust speed</label><span class="val" id="eval">--</span></div>
        <input type="range" id="exhaust" min="0" max="255" step="1">
      </div>
      <div class="rpm">
        <div class="cell"><div class="num" id="rpm3">--</div><div class="cap">Fan 3 RPM</div></div>
        <div class="cell"><div class="num" id="rpm4">--</div><div class="cap">Fan 4 RPM</div></div>
      </div>
    </div>

    <div class="card">
      <label>Firmware update</label>
      <div class="upd">
        <input type="file" id="fw" accept=".bin">
        <button id="upbtn" type="button">Upload</button>
      </div>
      <div class="ustat" id="ustat"></div>
    </div>

    <div class="foot">Reachable at <code id="host"></code> &middot; up <span id="up">--</span>s &middot; rst <span id="rst">--</span></div>
  </div>

<script>
const $ = s => document.querySelector(s);
let ti, te;                               // debounce timers
let st = {};                              // last known state
const focused = el => document.activeElement === el;

function paint(s) {
  st = s;
  $('#temp').textContent = (s.temp === null || s.temp === undefined) ? '--' : s.temp.toFixed(1);

  // auto toggle + slider enable/disable
  document.querySelectorAll('#auto button').forEach(b => b.classList.toggle('on', +b.dataset.a === (s.auto ? 1 : 0)));
  $('#aact').textContent = s.auto ? '  ⚙ temp-controlled' : '';
  $('#intake').disabled = s.auto;
  $('#exhaust').disabled = s.auto;

  // speeds
  $('#ival').textContent = s.intake; if (!focused($('#intake'))) $('#intake').value = s.intake;
  $('#eval').textContent = s.exhaust; if (!focused($('#exhaust'))) $('#exhaust').value = s.exhaust;

  // RPM (0 or missing -> "--")
  const rpm = (v) => (v ? v : '--');
  $('#rpm1').textContent = rpm(s.rpm1);
  $('#rpm2').textContent = rpm(s.rpm2);
  $('#rpm3').textContent = rpm(s.rpm3);
  $('#rpm4').textContent = rpm(s.rpm4);

  $('#up').textContent = s.up;
  $('#rst').textContent = s.rst;
}

async function send(params) { paint(await (await fetch('/set?' + new URLSearchParams(params))).json()); }
async function refresh()    { try { paint(await (await fetch('/state')).json()); } catch(e) {} }

document.querySelectorAll('#auto button').forEach(b => b.addEventListener('click', () => send({auto: b.dataset.a})));
$('#intake').addEventListener('input', e => {
  $('#ival').textContent = e.target.value;
  clearTimeout(ti); ti = setTimeout(() => send({intake: e.target.value}), 120);
});
$('#exhaust').addEventListener('input', e => {
  $('#eval').textContent = e.target.value;
  clearTimeout(te); te = setTimeout(() => send({exhaust: e.target.value}), 120);
});

// firmware upload (multipart POST to /update with HTTP basic auth)
$('#upbtn').addEventListener('click', () => {
  const f = $('#fw').files[0];
  if (!f) { $('#ustat').textContent = 'Choose a .bin file first.'; return; }
  const pw = prompt('OTA password:');
  if (pw === null) return;
  const fd = new FormData();
  fd.append('firmware', f, f.name);
  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update');
  xhr.setRequestHeader('Authorization', 'Basic ' + btoa('admin:' + pw));
  $('#upbtn').disabled = true;
  xhr.upload.onprogress = e => {
    if (e.lengthComputable) $('#ustat').textContent = 'Uploading ' + Math.round(e.loaded / e.total * 100) + '%';
  };
  xhr.onload = () => {
    if (xhr.status === 200) { $('#ustat').textContent = 'Success — device rebooting…'; setTimeout(() => location.reload(), 7000); }
    else { $('#ustat').textContent = xhr.status === 401 ? 'Wrong password.' : 'Failed (' + xhr.status + ').'; $('#upbtn').disabled = false; }
  };
  xhr.onerror = () => { $('#ustat').textContent = 'Upload error.'; $('#upbtn').disabled = false; };
  xhr.send(fd);
});

$('#host').textContent = location.host;
refresh();
setInterval(refresh, 1000);               // keep temp + RPM live
</script>
</body>
</html>)HTML";
