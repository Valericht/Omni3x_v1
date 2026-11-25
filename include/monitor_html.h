#pragma once
#include <pgmspace.h>
// Dashboard Omni-3X. HTTP: http://192.168.4.1/  WebSocket: ws://192.168.4.1:81/ws  Kabel: Web Serial (lihat tools/monitor.html)
// Tanpa CDN / font eksternal: AP robot tidak punya internet.
// File ini dibuat dari tools/monitor.html: python tools/pack_monitor.py
const char MONITOR_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="id"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Omni-3X Monitor</title>
<style>
:root{--bg:#0e1116;--card:#151a21;--fg:#e8eaed;--mut:#8f98a3;--ln:#272d37;--soft:#1c222b;--ok:#3ddc84;--bad:#f0454b;--warn:#ffc233;--acc:#5aa2ff;--stop:#e5242b}
@media(prefers-color-scheme:light){:root{--bg:#f4f5f7;--card:#fff;--fg:#1c1e21;--mut:#5f6773;--ln:#d6d9de;--soft:#eef0f3;--ok:#15803d;--bad:#dc2626;--warn:#b45309;--acc:#2563eb;--stop:#dc2626}}
*{box-sizing:border-box}
body{margin:0 auto;max-width:1920px;padding:12px;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
.num,td,.kv b{font-variant-numeric:tabular-nums}
.card{background:var(--card);border:1px solid var(--ln);border-radius:12px;padding:12px;margin-bottom:10px}
h1{font-size:18px;margin:0}
h2{font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:var(--mut);margin:0 0 8px;font-weight:600}
.top{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.chip{padding:3px 10px;border-radius:99px;border:1px solid var(--ln);font-size:12px;background:var(--soft)}
.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}.mut{color:var(--mut)}
#banner{display:none;margin-bottom:10px;padding:10px 12px;border-radius:10px;border:1px solid var(--bad);color:var(--bad);background:var(--card)}
.topo{display:flex;align-items:stretch;gap:0}
.col{display:flex;align-items:center;min-width:0}
.col.s{gap:8px;flex:1 1 auto;flex-wrap:wrap;justify-content:flex-end}
.node{border:1.5px solid var(--ln);border-radius:9px;padding:5px 10px;background:var(--soft)}
.node b{display:block;font-size:13px}
.node small{display:block;color:var(--mut);font-size:11px;overflow-wrap:anywhere}
.node.on{border-color:var(--ok)}.node.off{border-color:var(--bad)}.node.idle{border-color:var(--warn)}
.link{flex:1 1 60px;min-width:60px;position:relative;display:flex;align-items:center;justify-content:center;text-align:center;font-size:11px;line-height:1.2;color:var(--mut)}
.link::before{content:"";position:absolute;left:0;right:0;top:50%;height:2px;margin-top:-1px;background:var(--lc,var(--ln))}
.link span{position:relative;background:var(--card);padding:1px 4px}
.link.on{--lc:var(--ok)}.link.bad{--lc:var(--bad)}.link.idle{--lc:var(--warn)}
@media(max-width:900px){.topo{flex-direction:column}.link{flex:0 0 34px}.link::before{left:50%;right:auto;top:0;bottom:0;width:2px;height:auto;margin:0 0 0 -1px}.col.s{justify-content:flex-start}}
.main{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.5fr) minmax(0,1.05fr);gap:10px;align-items:start}
@media(max-width:1250px){.main{grid-template-columns:minmax(0,1fr) minmax(0,1fr)}.main>#colM{grid-column:1/-1;order:3}}
@media(max-width:800px){.main{grid-template-columns:1fr}#colR{order:-1}}
.kv{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;align-content:start}
.kv span{color:var(--mut)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-top:8px}
.sc{overflow-x:auto}
table{width:100%;border-collapse:collapse}
th,td{padding:5px 6px;text-align:left;border-bottom:1px solid var(--ln);font-size:13px;vertical-align:middle}
th{color:var(--mut);font-weight:600;white-space:nowrap;font-size:11px;letter-spacing:.08em;text-transform:uppercase}
tr:last-child td{border-bottom:0}
tr.act td{background:var(--soft)}
.bar{height:7px;background:var(--ln);border-radius:4px;position:relative}
.bar i{position:absolute;top:0;height:7px;background:var(--acc);border-radius:4px}
#map{width:100%;height:400px;background:var(--bg);border:1px solid var(--ln);border-radius:8px;display:block}
#ch{width:100%;height:230px;display:block}
.badge{display:inline-block;min-width:18px;text-align:center;border:1px solid var(--ln);border-radius:4px;font-size:11px;color:var(--mut);margin-left:4px}
#ev{margin:0;padding:0;list-style:none;font-size:13px;display:flex;gap:6px 26px;flex-wrap:wrap}
#ev time{color:var(--mut);margin-right:8px}
button{padding:7px 12px;border:1px solid var(--ln);background:var(--soft);color:var(--fg);border-radius:9px;font:inherit;cursor:pointer;touch-action:manipulation}
button:disabled{opacity:.4;cursor:default}
button.go{background:var(--acc);border-color:var(--acc);color:#fff;font-weight:600}
button.stop{background:var(--stop);border-color:var(--stop);color:#fff;font-weight:700}
button.on{background:var(--acc);border-color:var(--acc);color:#fff}
.top button.stop{margin-left:auto;font-size:16px;padding:7px 34px}
.motors{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.motors .card{margin:0}
.big{font-size:42px;font-weight:700;line-height:1.1}
.tiles{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px}
.tile{background:var(--soft);border-radius:8px;padding:7px 10px}
.tile small{display:block;color:var(--mut);font-size:11px}
.tile b{font-size:17px}
.trio{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:10px}
.trio .card{margin:0}
@media(max-width:700px){.motors,.trio{grid-template-columns:1fr}}
.bdg{padding:3px 12px;border-radius:99px;font-size:12px;font-weight:700;color:#fff;background:var(--ln)}
.bdg.g{background:#15803d}.bdg.y{background:#ca8a04}.bdg.r{background:#dc2626}
.jog{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.jog button{padding:15px 4px;font-size:15px;touch-action:none;user-select:none;-webkit-user-select:none}
.jog button.stop{font-size:16px}
.pre{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
.pre button{padding:10px 4px;display:flex;flex-direction:column;align-items:center;gap:2px;min-height:76px}
.pre b{font-size:20px}
.pre small{font-size:11px;opacity:.85;text-align:center;line-height:1.2}
.pre button.run{background:var(--ok);border-color:var(--ok);color:#06210f}
input,select{padding:5px 7px;border:1px solid var(--ln);border-radius:7px;background:var(--bg);color:var(--fg);font:inherit}
input[type=number]{width:78px}
input.t{width:170px}
input[type=range]{width:100%;padding:0;accent-color:var(--acc)}
.alert{display:none;margin-bottom:10px;padding:10px 12px;border-radius:10px;border:1px solid var(--warn);color:var(--warn);background:var(--card)}
.alert.bad{border-color:var(--bad);color:var(--bad)}
.alert.info{border-color:var(--acc);color:var(--acc)}
.alert .row button{color:var(--fg)}
.cal{display:grid;grid-template-columns:auto 1fr;gap:2px 8px;font-size:12px;color:var(--mut)}
.seg i{display:inline-block;width:11px;height:6px;margin-right:2px;border-radius:2px;background:var(--ln)}.seg i.f{background:var(--ok)}
details{margin-top:10px}
details>summary{cursor:pointer;color:var(--mut);padding:6px 2px;font-weight:600}
.hint{font-size:12px;color:var(--mut);margin-top:8px}
.link-btn{background:none;border:0;color:var(--acc);padding:6px 0;width:100%;text-align:center}
</style></head><body>
<div class="card top">
 <h1>Omni-3X</h1>
 <span class="chip" id="cCable">Kabel: belum disambung</span>
 <span class="chip" id="cWifi">WiFi: menghubungkan</span>
 <span class="chip num" id="cLat">- ms</span>
 <span class="chip" id="cMode">-</span>
 <span class="chip" id="cPad">gamepad -</span>
 <span class="chip" id="cGrip">Gripper: -</span> <button onclick="send({cmd:'grip',on:true})">Jepit</button> <button onclick="send({cmd:'grip',on:false})">Lepas</button>
 <button id="bCable" onclick="toggleCable()">Sambung kabel</button>
 <button class="stop" onclick="doStop(false)">STOP</button>
</div>
<div id="banner">Tidak ada data dari robot lewat kabel maupun Wi-Fi. Cek kabel UART atau sambung ke Wi-Fi Omni3X; halaman ini pulih sendiri. Robot berhenti otomatis bila kedua jalur diam.</div>
<div id="aSafe" class="alert bad"></div>
<div id="aPose" class="alert"></div>

<div class="card"><h2>Sambungan</h2>
<div class="topo">
 <div class="col"><div class="node" id="nPad"><b>Gamepad</b><small id="tPad">-</small></div></div>
 <div class="link" id="lUsb"><span>USB-OTG</span></div>
 <div class="col"><div class="node" id="nMaster"><b>Master ESP32-S3</b><small id="tMaster">-</small></div></div>
 <div class="link" id="lIo"><span>PWM, encoder, I2C</span></div>
 <div class="col s" id="parts"></div>
</div></div>

<div class="main">
 <div id="colL">
  <div class="card"><h2>Posisi robot</h2>
   <canvas id="map"></canvas>
   <div class="tiles" id="pos"></div>
   <div class="row"><button class="go" style="flex:1" onclick="setZero()">Set titik 0 (posisi dan arah)</button><button onclick="trail.length=0;S&&drawMap()">Hapus jejak</button></div>
   <div class="hint" id="trailInfo">Jejak (X, Y, Yaw) direkam saat robot digerakkan manual.</div>
  </div>
  <div class="card"><h2>Koreksi dinding</h2><div id="wc"></div></div>
 </div>

 <div id="colM">
  <div class="motors" id="motors"></div>
  <div class="card" style="margin-top:10px"><h2>RPM roda · 30 detik terakhir <span style="float:right;text-transform:none;letter-spacing:0"><b style="color:#5aa2ff">M1</b> <b style="color:#3ddc84">M2</b> <b style="color:#ffc233">M3</b> garis tipis = target</span></h2><canvas id="ch"></canvas></div>
  <div class="trio">
   <div class="card"><h2>IMU BNO055</h2><div id="imu"></div></div>
   <div class="card"><h2>Ultrasonik</h2><div id="us"></div></div>
   <div class="card"><h2>Pengaman</h2>
    <div class="row" style="margin-top:0"><label><input type="checkbox" id="safeOn" style="width:auto" onchange="send({cmd:'safety',on:this.checked})"> Aktif</label><span class="chip" id="safeChip">-</span></div>
    <div class="kv" id="safeKv" style="margin-top:8px"></div>
   </div>
  </div>
  <div class="card" style="margin-top:10px"><h2>Medan · selip · PID yaw</h2><div id="asst"></div></div>
 </div>

 <div id="colR">
  <div class="card"><h2>Kontrol manual</h2>
   <div class="jog">
    <button data-jog="5">&#10226; Putar kiri</button><button data-jog="1">&#9650; Maju</button><button data-jog="6">Putar kanan &#10227;</button>
    <button data-jog="3">&#9664; Kiri</button><button class="stop" onclick="doStop(true)">STOP</button><button data-jog="4">Kanan &#9654;</button>
    <span></span><button data-jog="2">&#9660; Mundur</button><span></span>
   </div>
   <div class="row"><span class="chip" id="jgStat">diam</span><span class="mut" style="margin-left:auto;font-size:12px">Tahan = jalan, lepas = berhenti · W A S D, Q E, Spasi = STOP</span></div>
   <div style="margin-top:10px"><label for="spSl">Kecepatan manual: <b id="spVal">-</b> RPM</label>
    <input type="range" id="spSl" min="30" max="500" step="10" value="150" oninput="$('spVal').textContent=this.value" onchange="setManualSpeed(this.value)"></div>
   <div class="row"><span>Simpan sebagai</span><select id="jogWp"></select>
    <label style="margin-left:auto"><input type="checkbox" id="autoSave" checked style="width:auto"> simpan otomatis saat STOP</label></div>
   <div class="row"><button class="go" style="flex:1" onclick="doStop(true,true)">STOP dan simpan posisi</button></div>
  </div>

  <div class="card"><h2>Preset waypoint <span id="preInfo" style="float:right;text-transform:none;letter-spacing:0"></span></h2>
   <div class="pre" id="pre"></div>
   <div class="hint">Tekan sekali = langsung jalan; di tiap waypoint ber-yaw robot berputar ke yaw-nya sebelum lanjut. STOP = batal. Gamepad: A, B, X, Y = Preset A, B, C, D.</div>
  </div>

  <div class="card"><h2>Waypoint</h2>
   <div class="sc"><table id="wpm"></table></div>
   <button class="link-btn" id="wpMore" onclick="wpAll=!wpAll;wpKey='';render()"></button>
  </div>
 </div>
</div>

<div class="card"><h2>Kejadian</h2><ul id="ev"></ul></div>

<details class="card" id="adv"><summary>Pengaturan lanjutan (rute preset, edit waypoint, tes gerak, kecepatan, PID)</summary>
<h2 style="margin-top:12px">Rute tiap preset</h2>
<div class="sc"><table id="rtt"></table></div>
<div class="hint">Preset A, B, C, D = tombol A, B, X, Y di gamepad. Tiap preset menjalankan beberapa waypoint berurutan, mis. 1,2,3 (default: A = WP1, B = WP2, C = WP3, D = WP4). G1 = gripper ON (jepit), G0 = gripper OFF (lepas), mis. 1,2,G1,3,G0: robot berhenti di waypoint sebelum langkah gripper. Waypoint perantara dilewati tanpa berhenti; robot berhenti dan memutar badan di waypoint terakhir.</div>

<h2 style="margin-top:16px">Edit waypoint (x, y, yaw)</h2>
<div class="sc"><table id="wp"></table></div>
<div class="hint">Yaw kosong = robot tidak memutar badan di waypoint itu. "Simpan" memakai angka di kolom. Semua waypoint tersimpan di flash.</div>

<h2 style="margin-top:16px">Tes gerak: maju, mundur, kiri, kanan</h2>
<div class="two" style="display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:14px">
 <div>
  <div class="jog" style="max-width:300px">
   <span></span><button id="mtF" onclick="mTest(1)">Maju</button><span></span>
   <button id="mtL" onclick="mTest(3)">Kiri</button><div class="tile" id="mLive" style="text-align:center;font-weight:700">DIAM</div><button id="mtR" onclick="mTest(4)">Kanan</button>
   <span></span><button id="mtB" onclick="mTest(2)">Mundur</button><span></span>
  </div>
  <div class="row"><button class="go" id="mtAll" onclick="mTestAll()">Tes 4 arah berurutan</button><span class="chip" id="mStat">siap</span></div>
 </div>
 <div class="kv" id="mKv"></div>
</div>
<div class="sc" style="margin-top:10px"><table id="mtab"></table></div>
<div class="hint">Letakkan robot di lantai dengan ruang kosong sekitar 60 cm ke segala arah. Tes batal otomatis bila STOP ditekan, stick digerakkan, ada rintangan dekat, atau dashboard berhenti mengirim detak.</div>

<h2 style="margin-top:16px">Kecepatan lanjutan (RPM roda)</h2>
<div class="row" style="margin-top:0"><span>Turbo (tahan R1)</span><input id="spT" type="number" step="10" oninput="this.dataset.d=1"><span>Navigasi dan preset</span><input id="spN" type="number" step="10" oninput="this.dataset.d=1"><button class="go" onclick="saveSpeed()">Simpan</button></div>
<div class="hint">Batas 30 sampai 500 RPM, tersimpan di flash. Kecepatan manual diatur dari slider di panel kontrol.</div>

<h2 style="margin-top:16px">PID per roda</h2>
<div class="sc"><table><thead><tr><th>Roda</th><th>Kp</th><th>Ki</th><th>Kd</th><th></th></tr></thead><tbody id="pidt"></tbody></table></div>
<div class="row"><span class="chip" id="tune" style="display:none"></span></div>
<div class="hint">Simpan langsung ke flash. Auto-tune (step response, roda diputar 50% PWM): angkat roda dulu.</div>
</details>

<script>
const $=id=>document.getElementById(id);
const AVN=['','Berhenti sebentar (cek halangan)','Menggeser ke samping','Melewati halangan'];
const ARRV=['tanpa verifikasi','cocok','TIDAK COCOK'],arrTxt=a=>{const f=i=>a.r[i]>0?`${a.m[i]<0?'-':a.m[i]}/${a.r[i]}`:'-';return `WP${a.id} · ${ARRV[a.v]||''} · K ${f(0)} Ka ${f(1)} D ${f(2)} cm · sisa lateral ${a.rl} sepanjang ${a.rf} cm · verifikasi ${a.n}x`};
const MODES=['Manual','Otonom','Stop','Tes gerak','Jalan dashboard'],TUNE=['','berjalan','selesai','gagal'],TW=['','respons terlalu kecil: cek motor/encoder/ENC_CPR','arah encoder terbalik: balik ENC_SIGN roda ini','respons belum mantap: cek beban, roda harus terangkat','dibatalkan (STOP/koneksi putus)'],PAD=['A','B','X','Y'],COL=['#2563eb','#16a34a','#d97706'],N=300,LIM=500;
const hist=[[],[],[]],histT=[[],[],[]],bad=[0,0,0],rev=[0,0,0],evs=[];
const JOGN=['','Maju','Mundur','Kiri','Kanan','Putar kiri','Putar kanan'],SF=['','Roda berputar kencang tetapi robot tidak bergerak (menabrak atau roda selip)','Robot berputar tidak terkendali','Motor macet (beban terlalu berat atau tertahan)','Robot terangkat saat motor jalan'],
 DENY=['','posisi tidak valid: tetapkan titik 0 dulu','rute kosong atau waypoint belum disimpan','pengaman aktif: tekan Lanjut dulu','robot sedang dipakai (otonom, tes gerak, jalan, atau stick)','posisi tidak pasti (tanjakan): tunggu robot datar dan diam','gagal tiba setelah beberapa percobaan ulang','terhalang terlalu lama (rintangan di depan)','sinkron ditolak: WP tanpa referensi dinding, jarak > 30 cm, atau bukan robot diam'],SVW=['','waypoint','PID','posisi terakhir','rute','kecepatan'];
let S=null,P=null,wpKey='',lastMsg=0,gap=100,jogInit=0,wpAll=false;
const KV=a=>a.map(r=>`<span>${r[0]}</span><b>${r[1]}</b>`).join('');
const setH=(el,h)=>{if(el._h!==h){el._h=h;el.innerHTML=h}};
const send=o=>{const m=JSON.stringify(o);if(serialOpen&&(cableAlive()||!wsOpen()))serWrite(m);else if(wsOpen())ws.send(m)}; // kabel diutamakan, Wi-Fi cadangan
const sendAll=o=>{const m=JSON.stringify(o);if(wsOpen())ws.send(m);if(serialOpen)serWrite(m)}; // STOP: ke semua jalur
function log(t){evs.unshift([new Date().toLocaleTimeString('id-ID'),t]);if(evs.length>12)evs.pop();setH($('ev'),evs.slice(0,6).map(e=>`<li><time class="num">${e[0]}</time>${e[1]}</li>`).join(''))}
function chip(id,txt,cls){const e=$(id);e.textContent=txt;e.className='chip '+(cls||'')}

/* ===== dua jalur: Wi-Fi (WebSocket) + kabel (Web Serial, 460800 baud) ===== */
const HOST=(location.protocol==='file:'||!location.hostname)?'192.168.4.1':location.hostname,BAUD=460800,te=new TextEncoder();
let ws=null,lastWifi=0,lastSer=0,port=null,serW=null,rdr=null,ended=null,endRes=null,serialOpen=false,pw=false,pc=false;
const wsOpen=()=>ws&&ws.readyState===1;
const cableAlive=()=>serialOpen&&performance.now()-lastSer<1500;
const wifiAlive=()=>wsOpen()&&performance.now()-lastWifi<1500;
function connect(){
 try{ws=new WebSocket('ws://'+HOST+':81/ws')}catch(e){setTimeout(connect,1000);return}
 ws.onmessage=e=>{try{accept(JSON.parse(e.data),'wifi')}catch(x){rerr(x)}};
 ws.onclose=()=>setTimeout(connect,1000)}
function accept(j,src){
 if(!j||!j.rpm||!j.wp)return;
 const n=performance.now();if(src==='wifi')lastWifi=n;else lastSer=n;
 if(src==='wifi'&&cableAlive())return; // data kabel diutamakan selama hidup
 if(lastMsg)gap=gap*.8+(n-lastMsg)*.2;lastMsg=n;S=j;render()}
const serWrite=m=>{if(serW)serW.write(te.encode(m+'\n')).catch(()=>{})};
let lastErr='';function rerr(e){const m=(e&&e.message)||String(e);if(m!==lastErr){lastErr=m;log('Error tampilan: '+m);console.error(e)}}
function onLine(l){const i=l.indexOf('{');if(i<0)return;try{accept(JSON.parse(l.slice(i)),'kabel')}catch(e){rerr(e)}} // baris debug diabaikan
function toggleCable(){serialOpen?closeCable():openCable()}
async function openCable(){
 if(!('serial' in navigator)){alert('Kabel butuh Web Serial: pakai Chrome atau Edge di PC, dan buka file tools/monitor.html (bukan http://192.168.4.1, halaman itu bukan konteks aman).');return}
 try{
  port=await navigator.serial.requestPort();await port.open({baudRate:BAUD});
  try{await port.setSignals({dataTerminalReady:false,requestToSend:false})}catch(e){}
  serW=port.writable.getWriter();rdr=port.readable.getReader();ended=new Promise(r=>endRes=r);
  serialOpen=true;lastSer=0;$('bCable').textContent='Putuskan kabel';log('Kabel dibuka, '+BAUD+' baud');readLoop(rdr);
 }catch(e){port=null;serialOpen=false;log('Kabel gagal dibuka: '+((e&&e.message)||e))}}
async function readLoop(r){
 const dec=new TextDecoder();let buf='';
 try{for(;;){const x=await r.read();if(x.done)break;buf+=dec.decode(x.value,{stream:true});let i;
  while((i=buf.indexOf('\n'))>=0){onLine(buf.slice(0,i));buf=buf.slice(i+1)}if(buf.length>4000)buf=''}}catch(e){}
 try{r.releaseLock()}catch(e){}
 endRes();if(port)closeCable()} // kabel dicabut: bersihkan
async function closeCable(){
 if(!port)return;const p=port;port=null;serialOpen=false;lastSer=0;
 try{await rdr.cancel()}catch(e){}
 await ended;
 try{serW.releaseLock()}catch(e){}
 try{await p.close()}catch(e){}
 serW=null;rdr=null;$('bCable').textContent='Sambung kabel'}
function linkTick(){
 const w=wifiAlive(),c=cableAlive();
 chip('cWifi','WiFi: '+(w?'tersambung':wsOpen()?'menunggu data':'terputus'),w?'ok':c?'warn':'bad');
 chip('cCable','Kabel: '+(c?'tersambung':serialOpen?'menunggu data':'belum disambung'),c?'ok':serialOpen?'warn':'mut');
 if(w!==pw){log(w?'WiFi tersambung':'WiFi terputus');pw=w}
 if(c!==pc){log(c?'Kabel tersambung':'Kabel terputus');pc=c}
 $('banner').style.display=(w||c)?'none':'block'}
// detak ke SEMUA jalur terbuka: robot hanya berhenti bila keduanya diam (< 300 ms)
setInterval(()=>{const m='{"cmd":"hb"}';if(wsOpen())ws.send(m);if(serialOpen)serWrite(m)},100);
setInterval(linkTick,500);
connect();linkTick();


function diag(i){const sp=S.sp[i],r=S.rpm[i],n=performance.now();
 if(S.tune==1)return['warn','auto-tune berjalan'];
 if(Math.abs(sp)>=15&&Math.abs(r)<2){if(!bad[i])bad[i]=n;if(n-bad[i]>1500)return['bad','target ada tapi RPM 0: cek kabel motor atau encoder']}else bad[i]=0;
 if(Math.abs(sp)>=15&&Math.abs(r)>=5&&r*sp<0){if(!rev[i])rev[i]=n;if(n-rev[i]>1500)return['bad','arah terbalik: cek MOTOR_SIGN dan ENC_SIGN']}else rev[i]=0;
 return Math.abs(sp)>=1?['ok','berputar normal']:['ok','siap']}


/* ===== tes gerak (maju, mundur, kiri, kanan) ===== */
const DIRS=['','Maju','Mundur','Kiri','Kanan'],VT=['-','OK','TERBALIK','tidak bergerak','melenceng','sinyal lemah'],
 WHY=['','dibatalkan (STOP, stick, atau tombol B)','dashboard tidak mengirim detak','ada rintangan dekat','','robot sedang otonom'];
const mres={},mq=[];let mPrev=null;
const vcls=v=>v==1?'ok':v==0?'mut':(v==4||v==5)?'warn':'bad';
function mTest(d){send({cmd:'mtest',dir:d})}
function mTestAll(){mq.length=0;mq.push(2,3,4);mTest(1)}
function icalTxt(c){c=c||0;return 'SYS '+(c>>6&3)+', GYR '+(c>>4&3)+', ACC '+(c>>2&3)+', MAG '+(c&3)}
function diagnose(r){const o=r.o,i=r.i;
 if(o==3)return i==1?['bad','Robot bergerak (IMU setuju) tapi encoder diam: cek kabel encoder']:['bad','Robot tidak bergerak: cek daya baterai, kabel motor, driver'];
 if(o==4)return['warn','Arah melenceng: roda tidak seimbang, ada slip, atau tanda kinematika salah'];
 if(o==1){if(i==1)return['ok','OK, odometri dan IMU searah'];
  if(i==2)return['warn','Robot benar, IMU terbalik: ubah IMU_ACC_FWD_SIGN, IMU_ACC_RIGHT_SIGN, atau IMU_ACC_FWD_AXIS di config.h'];
  if(i==0)return['mut','OK menurut odometri (IMU mati)'];
  if(i==5)return['warn','OK menurut odometri, sinyal IMU lemah (kalibrasi akselerometer belum penuh?)'];
  return['warn','OK menurut odometri, IMU melenceng: cek pemasangan sumbu IMU']}
 if(o==2){if(i==1)return['bad','Encoder terbalik: balik ENC_SIGN di config.h (robot sebenarnya bergerak benar)'];
  if(i==2)return['bad','Robot bergerak terbalik (odometri dan IMU sepakat): balik MOTOR_SIGN dan ENC_SIGN semua roda bersamaan'];
  return['bad','Arah terbalik menurut odometri, IMU tidak bisa memastikan: cek MOTOR_SIGN dan ENC_SIGN']}
 return['mut','-']}
function mRender(){
 const M=S.mc||{s:0,dir:0},vx=S.mvx||0,vy=S.mvy||0,sp=Math.hypot(vx,vy);
 let lv='DIAM';if(sp>=3)lv=Math.abs(vy)>=Math.abs(vx)?(vy>0?'MAJU':'MUNDUR'):(vx>0?'KANAN':'KIRI');
 const L=$('mLive');L.textContent=lv;L.style.color=sp>=3?'var(--acc)':'var(--mut)';
 setH($('mKv'),`<span>Kecepatan terukur x (kanan)</span><b>${vx} cm/s</b><span>Kecepatan terukur y (depan)</span><b>${vy} cm/s</b><span>Yaw</span><b>${S.yaw}°</b>`);
 if(mPrev===null)mPrev=M.s;
 else if(M.s!==mPrev){
  if(M.s===1)log('Tes gerak '+DIRS[M.dir]+' mulai');
  else if(M.s===2){mres[M.dir]=M;log('Tes gerak '+DIRS[M.dir]+': odometri '+VT[M.o]+', IMU '+VT[M.i]);if(mq.length)setTimeout(()=>mTest(mq.shift()),1200)}
  else if(M.s===3){mres[M.dir]=M;mq.length=0;log('Tes gerak '+DIRS[M.dir]+' batal: '+(WHY[M.why]||''))}
  mPrev=M.s}
 const busy=M.s===1||S.mode===1||S.mode===4||S.sf>0;
 ['mtF','mtB','mtL','mtR','mtAll'].forEach(id=>$(id).disabled=busy);
 chip('mStat',M.s===1?'berjalan: '+DIRS[M.dir]:M.s===2?'selesai: '+DIRS[M.dir]:M.s===3?'batal':'siap',M.s===1?'warn':M.s===2?'ok':M.s===3?'bad':'mut');
 let h='<thead><tr><th>Arah</th><th>Odometri x / y (cm)</th><th>IMU x / y (cm/s)</th><th>Putar</th><th>Diagnosa</th></tr></thead><tbody>';
 for(let d=1;d<=4;d++){
  const r=(M.s===1&&M.dir===d)?M:mres[d];
  if(!r){h+=`<tr><td>${DIRS[d]}</td><td colspan="4" class="mut">belum dites</td></tr>`;continue}
  if(r.s===1){h+=`<tr class="act"><td>${DIRS[d]}</td><td class="num">${r.dx} / ${r.dy}</td><td class="num">${r.ix} / ${r.iy}</td><td></td><td class="warn">sedang berjalan</td></tr>`;continue}
  if(r.s===3){h+=`<tr><td>${DIRS[d]}</td><td colspan="3"></td><td class="bad">batal: ${WHY[r.why]||''}</td></tr>`;continue}
  const dg=diagnose(r),yw=Math.abs(r.dyaw)>10;
  h+=`<tr><td>${DIRS[d]}</td><td class="num">${r.dx} / ${r.dy}<br><b class="${vcls(r.o)}">${VT[r.o]}</b></td>`+
   `<td class="num">${r.i?r.ix+' / '+r.iy:'-'}<br><b class="${vcls(r.i)}">${r.i?VT[r.i]:'IMU mati'}</b></td>`+
   `<td class="num ${yw?'warn':''}">${r.dyaw}°${r.yi?'':'<br><span class="mut">odometri</span>'}</td><td class="${dg[0]}">${dg[1]}</td></tr>`}
 setH($('mtab'),h+'</tbody>')}


function savePid(m){const s=m<0?'A':m;(m<0?[0,1,2,'A']:[m]).forEach(w=>['kp','ki','kd'].forEach(k=>{const e=$(k+w);if(e&&w!=='A')delete e.dataset.d}));send({cmd:'pid',m,kp:+$('kp'+s).value,ki:+$('ki'+s).value,kd:+$('kd'+s).value});log('PID '+(m<0?'semua roda':'roda '+(m+1))+' dikirim')}
function autoTune(m){if(confirm('Angkat '+(m<0?'semua roda':'roda '+(m+1))+' dari lantai dulu. Motor akan berputar. Lanjut?'))send({cmd:'autotune',m})}
(function(){let h='';[0,1,2,-1].forEach(i=>{const s=i<0?'A':i;
 h+=`<tr><td>${i<0?'Semua':i+1}</td>`+['kp','ki','kd'].map(k=>`<td><input id="${k}${s}" type="number" step="any" oninput="this.dataset.d=1"></td>`).join('')+
  `<td style="white-space:nowrap"><button onclick="savePid(${i})">${i<0?'Terapkan ke semua':'Simpan'}</button> <button onclick="autoTune(${i})">Auto-tune${i<0?' semua':''}</button></td></tr>`});
 $('pidt').innerHTML=h})();


/* ===== dashboard satu halaman: kontrol manual, preset, waypoint ===== */
function node(id,cls,txt){$(id).className='node '+cls;if(txt!==undefined)$('t'+id.slice(1)).innerHTML=txt}
const PRE=['A','B','C','D'],PADB=['A','B','X','Y'],PS=['IDLE','','JALAN'],KEYD={KeyW:1,KeyS:2,KeyA:3,KeyD:4,KeyQ:5,KeyE:6};
const trail=[];let held=null,heldBy='',keysDown=[],recDirty=false,wpKey2='';
const fx=v=>Math.round(v);

// --- kontrol manual: tahan = jalan, lepas = berhenti (tombol layar dan W/A/S/D identik) ---
function jogStart(d){if(held===d)return;held=d;recDirty=true;send({cmd:'jog',d})}
function jogEnd(){if(held===null)return;held=null;heldBy='';sendAll({cmd:'jog',d:0});setTimeout(()=>{if(held===null)sendAll({cmd:'jog',d:0})},80)}
function doStop(save,force){keysDown=[];held=null;heldBy='';sendAll({cmd:'stop'});
 if(save&&S&&!S.pl&&(force||($('autoSave').checked&&recDirty))){const id=+$('jogWp').value;sendAll({cmd:'stop_save',id});log('STOP: posisi dan arah disimpan ke WP'+id)}
 recDirty=false}
function setZero(){send({cmd:'odom_reset'});trail.length=0}
function setManualSpeed(v){v=+v;send({cmd:'speed',man:v,loc:v});log('Kecepatan manual '+v+' RPM')}
function saveSpeed(){send({cmd:'speed',turbo:+$('spT').value,nav:+$('spN').value});['spT','spN'].forEach(id=>delete $(id).dataset.d);log('Kecepatan turbo dan navigasi dikirim')}
document.querySelectorAll('[data-jog]').forEach(b=>{const d=+b.dataset.jog;
 b.addEventListener('pointerdown',e=>{e.preventDefault();try{b.setPointerCapture(e.pointerId)}catch(x){}heldBy='ptr';jogStart(d)});
 ['pointerup','pointercancel','lostpointercapture'].forEach(t=>b.addEventListener(t,()=>{if(heldBy==='ptr')jogEnd()}));
 b.addEventListener('contextmenu',e=>e.preventDefault())});
addEventListener('pointerup',()=>{if(heldBy==='ptr')jogEnd()});
const typing=e=>{const t=e.target,n=t.tagName;return n==='SELECT'||n==='TEXTAREA'||(n==='INPUT'&&t.type!=='range'&&t.type!=='checkbox')};
addEventListener('keydown',e=>{if(typing(e)||e.metaKey||e.ctrlKey||e.altKey)return;
 if(e.code==='Space'){e.preventDefault();if(!e.repeat)doStop(true);return}
 const d=KEYD[e.code];if(!d)return;e.preventDefault();if(e.repeat)return;
 if(!keysDown.includes(d))keysDown.push(d);heldBy='key';jogStart(d)});
addEventListener('keyup',e=>{if(e.code==='Space'&&!typing(e)){e.preventDefault();return}
 const d=KEYD[e.code];if(!d)return;keysDown=keysDown.filter(k=>k!==d);if(heldBy!=='key')return;
 if(keysDown.length)jogStart(keysDown[keysDown.length-1]);else jogEnd()});
addEventListener('blur',()=>{keysDown=[];jogEnd()});
document.addEventListener('visibilitychange',()=>{if(document.hidden){keysDown=[];jogEnd()}});

// --- preset A..D: tombol dibuat sekali, status diperbarui tiap telemetri ---
function presetTap(i){send({cmd:'preset',i});log('Preset '+PRE[i]+': jalan')}
(function(){let h='';PRE.forEach((n,i)=>{h+=`<button id="pb${i}" onclick="presetTap(${i})"><b>${n}</b><span class="mut" style="font-size:10px">gamepad ${PADB[i]}</span><small id="pbs${i}">Idle</small></button>`});$('pre').innerHTML=h})();
function renderPre(){
 const ps=S.ps||[0,0,0,0],anyRun=ps.includes(2);
 for(let i=0;i<4;i++){const b=$('pb'+i),st=ps[i],rt=(S.rts&&S.rts[i])||[],lbl=rt.length?rt.map(rtLbl).join('→'):'kosong';
  b.className=st===2?'run':'';
  setH($('pbs'+i),st===2?(S.np===4?'Gripper '+(S.gp?'menjepit':'melepas'):S.np===2?'Memutar badan':S.np===3?'Verifikasi posisi':S.av?AVN[S.av]:'Jalan ke WP'+S.tgt):(rt.length?'Idle · '+lbl:'Idle · kosong'));
  b.disabled=S.sf>0||S.jg>0||S.mode===3||(st===0&&!rt.length)||(anyRun&&st!==2)}
 const r=ps.indexOf(2);
 setH($('preInfo'),r>=0?`<span class="ok">${S.np===2?'ROTATING: memutar ke arah waypoint':S.np===4?'GRIPPER: '+(S.gp?'menjepit':'melepas'):S.np===3?'VERIFY: mencocokkan bacaan dinding':S.av?'AVOID: '+AVN[S.av]:'MOVING: menuju titik'}</span>`:'<span class="mut">IDLE</span>')}

// --- tabel waypoint ---
function buildWpm(){const n=wpAll?S.wp.length:5;
 let h='<thead><tr><th>WP</th><th>X (cm)</th><th>Y (cm)</th><th>Yaw (°)</th><th></th></tr></thead><tbody>';
 S.wp.slice(0,n).forEach((w,i)=>{const id=i+1,used=(S.rts||[]).map((r,k)=>r.includes(id)?PRE[k]:'').filter(Boolean).join('');
  h+=`<tr class="${S.tgt==id?'act':''}"><td>WP${id}${used?`<span class="badge">${used}</span>`:''}</td><td class="num">${w[0]?w[1]:'-'}</td><td class="num">${w[0]?w[2]:'-'}</td><td class="num">${w[0]&&w[4]?w[3]:'-'}${w[0]&&!(w[5]>0||w[6]>0||w[7]>0)?' <span class="mut">tanpa referensi</span>':''}</td>`+
  `<td style="text-align:right;white-space:nowrap"><button class="go" ${w[0]?'':'disabled'} onclick="send({cmd:'goto',id:${id}})">Jalan</button> <button title="Simpan posisi dan arah robot sekarang" onclick="send({cmd:'wp_here',id:${id}})">Di sini</button> <button title="Cocokkan posisi dengan referensi dinding WP ini (robot diam, jarak <= 30 cm)" ${w[0]&&(w[5]>0||w[6]>0||w[7]>0)?'':'disabled'} onclick="send({cmd:'wp_sync',id:${id}})">Sinkron ke WP ${id}</button></td></tr>`});
 $('wpm').innerHTML=h+'</tbody>';$('wpMore').textContent=wpAll?'Sembunyikan WP6 – WP'+S.wp.length:'Tampilkan WP6 – WP'+S.wp.length+' ▾';$('wpMore').style.display=S.wp.length>5?'':'none'}
function buildWp(){$('wp').innerHTML='<thead><tr><th>WP</th><th>x (cm)</th><th>y (cm)</th><th>yaw (°)</th><th></th></tr></thead><tbody>'+S.wp.map((w,i)=>{const id=i+1;
 return `<tr><td>WP${id}</td><td><input id="wx${id}" type="number" value="${w[0]?w[1]:''}" placeholder="x"></td><td><input id="wy${id}" type="number" value="${w[0]?w[2]:''}" placeholder="y"></td><td><input id="wa${id}" type="number" value="${w[0]&&w[4]?w[3]:''}" placeholder="yaw"></td>`+
 `<td style="text-align:right;white-space:nowrap"><button onclick="saveWp(${id})">Simpan</button> <button title="Hapus" onclick="send({cmd:'wp_del',id:${id}})">&#10005;</button></td></tr>`}).join('')+'</tbody>'}
function saveWp(i){const x=parseFloat($('wx'+i).value),y=parseFloat($('wy'+i).value),a=parseFloat($('wa'+i).value);if(isNaN(x)||isNaN(y))return alert('Isi x dan y (cm).');
 send(isNaN(a)?{cmd:'wp_set',id:i,x,y,noyaw:true}:{cmd:'wp_set',id:i,x,y,yaw:a})}
const rtTok=n=>n===101?'G1':n===102?'G0':String(n),rtLbl=n=>n===101?'Gripper ON':n===102?'Gripper OFF':'WP'+n;
 function saveRoute(r){const ids=($('rt'+r).value.match(/[Gg][01]|\d+/g)||[]).map(t=>/^[Gg]1$/.test(t)?101:/^[Gg]0$/.test(t)?102:Number(t)).filter(n=>(n>=1&&n<=S.wp.length)||n===101||n===102).slice(0,16);
 send({cmd:'route_set',r,ids});delete $('rt'+r).dataset.d;log('Rute preset '+PRE[r]+' dikirim: '+(ids.map(rtTok).join(',')||'kosong'))}
(function(){let h='<thead><tr><th>Preset</th><th>Urutan waypoint</th><th>Status</th><th></th></tr></thead><tbody>';
 PRE.forEach((n,r)=>{h+=`<tr><td><b>${n}</b> <span class="mut">(gamepad ${PADB[r]})</span></td><td><input class="t" id="rt${r}" placeholder="mis. 1,2,G1,3" oninput="this.dataset.d=1"></td><td id="rs${r}" class="mut"></td><td style="text-align:right;white-space:nowrap"><button onclick="saveRoute(${r})">Simpan</button> <button class="go" id="rg${r}" onclick="presetTap(${r})">Jalankan</button></td></tr>`});
 $('rtt').innerHTML=h+'</tbody>'})();
(function(){let h='';[0,1,2,-1].forEach(i=>{const s=i<0?'A':i;
 h+=`<tr><td>${i<0?'Semua':i+1}</td>`+['kp','ki','kd'].map(k=>`<td><input id="${k}${s}" type="number" step="any" oninput="this.dataset.d=1"></td>`).join('')+
  `<td style="white-space:nowrap"><button onclick="savePid(${i})">${i<0?'Terapkan ke semua':'Simpan'}</button> <button onclick="autoTune(${i})">Auto-tune${i<0?' semua':''}</button></td></tr>`});
 $('pidt').innerHTML=h})();

function drawChart(){
 const c=$('ch'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;
 if(c.width!==Math.round(r.width*d)||c.height!==Math.round(r.height*d)){c.width=Math.round(r.width*d);c.height=Math.round(r.height*d)}
 const g=c.getContext('2d'),W=c.width,H=c.height;g.clearRect(0,0,W,H);
 const y=v=>H/2-v/LIM*(H/2-12*d);
 g.strokeStyle='#8886';g.lineWidth=1;g.font=(11*d)+'px ui-monospace,monospace';g.fillStyle='#8f98a3';
 [-LIM,-LIM/2,0,LIM/2,LIM].forEach(v=>{g.beginPath();g.setLineDash(v?[3,5]:[]);g.moveTo(0,y(v));g.lineTo(W,y(v));g.stroke();g.fillText((v>0?'+':'')+v,4,y(v)-3)});g.setLineDash([]);
 for(let i=0;i<3;i++){
  g.strokeStyle=COL[i]+'66';g.lineWidth=1.5*d;g.beginPath();histT[i].forEach((v,k)=>{k?g.lineTo(k*W/N,y(v)):g.moveTo(0,y(v))});g.stroke();
  g.strokeStyle=COL[i];g.lineWidth=2.5*d;g.beginPath();hist[i].forEach((v,k)=>{k?g.lineTo(k*W/N,y(v)):g.moveTo(0,y(v))});g.stroke()}}

function drawMap(){const c=$('map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;
 if(c.width!==Math.round(r.width*d)||c.height!==Math.round(r.height*d)){c.width=Math.round(r.width*d);c.height=Math.round(r.height*d)}
 const g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);const W=r.width,H=r.height,cs=getComputedStyle(document.documentElement);
 const mut=cs.getPropertyValue('--mut'),ln=cs.getPropertyValue('--ln'),acc=cs.getPropertyValue('--acc'),bad=cs.getPropertyValue('--bad'),fg=cs.getPropertyValue('--fg'),ok=cs.getPropertyValue('--ok');
 g.clearRect(0,0,W,H);
 let ext=100;for(const w of S.wp)if(w[0])ext=Math.max(ext,Math.abs(w[1])+30,Math.abs(w[2])+30);ext=Math.max(ext,Math.abs(S.x)+40,Math.abs(S.y)+40);
 const sc=Math.min(W,H)/2/ext,cx=W/2,cy=H/2,X=v=>cx+v*sc,Y=v=>cy-v*sc;
 const step=ext>400?100:ext>150?50:ext>60?20:10;g.lineWidth=1;g.font='11px ui-monospace,monospace';
 for(let v=-Math.ceil(ext/step)*step;v<=ext;v+=step){g.strokeStyle=v%(step*5)==0?mut:ln;g.globalAlpha=v==0?1:.6;
  g.beginPath();g.moveTo(X(v),0);g.lineTo(X(v),H);g.moveTo(0,Y(v));g.lineTo(W,Y(v));g.stroke()}
 g.globalAlpha=1;g.fillStyle=mut;g.fillText('0,0',X(0)+4,Y(0)+13);g.fillText(step+' cm / kotak',W-92,H-8);
 if(trail.length>1){g.strokeStyle=ok;g.globalAlpha=.7;g.lineWidth=2;g.beginPath();trail.forEach((p,k)=>k?g.lineTo(X(p[0]),Y(p[1])):g.moveTo(X(p[0]),Y(p[1])));g.stroke();g.globalAlpha=1;g.lineWidth=1}
 if(S.rt>=0&&S.rts&&S.rts[S.rt]){g.strokeStyle=acc;g.lineWidth=1.5;g.setLineDash([2,4]);g.beginPath();let f=1;S.rts[S.rt].forEach(id=>{const w=S.wp[id-1];if(!w||!w[0])return;if(f){g.moveTo(X(w[1]),Y(w[2]));f=0}else g.lineTo(X(w[1]),Y(w[2]))});g.stroke();g.setLineDash([]);g.lineWidth=1}
 S.wp.forEach((w,i)=>{if(!w[0])return;g.strokeStyle=acc;g.fillStyle=S.tgt==i+1?acc:'transparent';g.lineWidth=2;
  g.beginPath();g.arc(X(w[1]),Y(w[2]),6,0,7);g.fill();g.stroke();
  if(w[4]){const a=w[3]*Math.PI/180;g.beginPath();g.moveTo(X(w[1]),Y(w[2]));g.lineTo(X(w[1])+Math.cos(a)*16,Y(w[2])-Math.sin(a)*16);g.stroke()}   // arah hadap waypoint
  g.fillStyle=fg;g.fillText('WP'+(i+1),X(w[1])+9,Y(w[2])+4)});
 if(S.tgt&&S.wp[S.tgt-1]&&S.np!==2){const w=S.wp[S.tgt-1];g.setLineDash([5,5]);g.strokeStyle=acc;g.beginPath();g.moveTo(X(S.x),Y(S.y));g.lineTo(X(w[1]),Y(w[2]));g.stroke();g.setLineDash([])}
 const yaw=S.yaw*Math.PI/180;
 S.d.forEach((dd,i)=>{if(dd<0)return;const a=yaw+S.sa[i]*Math.PI/180,ox=S.x+Math.cos(a)*(dd+12),oy=S.y+Math.sin(a)*(dd+12);
  g.fillStyle=bad;g.beginPath();g.arc(X(ox),Y(oy),4,0,7);g.fill()});
 g.save();g.translate(X(S.x),Y(S.y));g.rotate(-yaw);
 g.fillStyle=fg;g.beginPath();const R=Math.max(10,12*sc);g.moveTo(0,-R);g.lineTo(R*.7,R*.7);g.lineTo(-R*.7,R*.7);g.closePath();g.fill();g.restore()}

function recTrail(){const mv=S.mode===4||(S.mode===0&&Math.hypot(S.mvx||0,S.mvy||0)>3);if(!mv)return;
 const l=trail[trail.length-1];
 if(!l||Math.hypot(S.x-l[0],S.y-l[1])>2||Math.abs(S.yaw-l[2])>3){trail.push([S.x,S.y,S.yaw]);if(trail.length>600)trail.shift()}}

function render(){
 chip('cLat',Math.round(gap)+' ms · '+(cableAlive()?'kabel':'wifi'),'num');
 chip('cMode','Mode '+MODES[S.mode]+(S.tgt?' ke WP'+S.tgt:'')+(S.mode===4?': '+JOGN[S.jg]:'')+(S.tb?' TURBO':''),S.sf?'bad':'');
 chip('cPad','Gamepad '+(S.pad?S.pt:'-'),S.pad?'ok':'bad');
 if(P){
  if(P.pad!==S.pad)log(S.pad?'Gamepad '+S.pt+' tersambung':'Gamepad terputus');
  if(P.mode!==S.mode)log('Mode: '+MODES[S.mode]);
  if(P.ps&&S.ps)for(let i=0;i<4;i++)if(P.ps[i]!==S.ps[i])log('Preset '+PRE[i]+': '+PS[S.ps[i]]);
  if(P.np!==S.np&&S.np===2)log('Tiba di titik, memutar badan ke arah waypoint');
  if(P.arr&&S.arr&&P.arr.q!==S.arr.q)log('Tiba: '+arrTxt(S.arr));   if(P.nd!==S.nd)log('Waypoint tuntas: posisi dan arah sudah sesuai');
  if(P.tune!==S.tune&&S.tune)log('Auto-tune: '+TUNE[S.tune]+(S.tune==3&&S.tw?' - '+TW[S.tw]:''));
  if(P.imu!==S.imu)log(S.imu?'IMU normal':'IMU tidak terdeteksi');
  if(P.sf!==S.sf&&S.sf)log('PENGAMAN: '+SF[S.sf]);
  if(P.stuck!==S.stuck&&S.stuck)log('Robot tertahan (stuck): berhenti');
  if(P.pose_degraded!==S.pose_degraded)log(S.pose_degraded?'Posisi tidak pasti (tanjakan): navigasi baru ditolak, kecepatan dibatasi':'Posisi kembali pasti');
  if(P.slip!==S.slip&&S.slip)log('Roda selip: kepercayaan encoder diturunkan');
  if(P.terrain!==S.terrain&&S.terrain&&S.terrain!=='FLAT')log('Medan: '+S.terrain+' (pitch '+S.pitch+'°, roll '+S.roll+'°)');
  if(P.sf&&!S.sf)log('Pengaman dibuka (Lanjut)');
  if(P.pl!==S.pl)log(S.pl?'Robot terdeteksi diangkat atau dipindah: posisi tidak valid':'Titik 0 ditetapkan');
  if(P.sv&&S.sv&&S.sv[0]&&P.sv[2]!==S.sv[2])log(S.sv[0]==1?'Tersimpan ke flash: '+SVW[S.sv[1]]:'GAGAL menyimpan ke flash: '+SVW[S.sv[1]]+'. Cek log serial [NVS]');
  if(P.dn&&S.dn&&P.dn[1]!==S.dn[1]&&S.dn[0])log('Perintah ditolak: '+DENY[S.dn[0]]);
 }
 P=S;recTrail();

 // sambungan
 node('nPad',S.pad?'on':'off',S.pad?S.pt+' tersambung':'tidak terdeteksi');
 $('lUsb').className='link '+(S.pad?'on':'bad');
 node('nMaster','on','AP Omni3X · batas '+S.spd+' RPM');
 const anyMove=S.rpm.some(v=>Math.abs(v)>=1),anyObs=S.d.some(v=>v>=0);
 $('lIo').className='link '+(S.imu?(anyMove?'on':'idle'):'bad');
 let ph='';[0,1,2].forEach(i=>{const m=Math.abs(S.rpm[i])>=1;ph+=`<div class="node ${m?'on':'idle'}"><b>Motor ${i+1}</b><small>${fx(S.rpm[i])} RPM</small></div>`});
 ph+=`<div class="node ${S.imu?'on':'off'}"><b>IMU BNO055</b><small>${S.imu?'0x'+(S.ia||0).toString(16)+' · yaw '+fx(S.yaw)+'°':'tidak terdeteksi'}</small></div>`;
 ph+=`<div class="node ${anyObs?'on':'idle'}"><b>Ultrasonik ×3</b><small>${S.d.map(v=>v<0?'-':fx(v)).join(' / ')} cm</small></div>`;
 setH($('parts'),ph);

 // posisi
 const tile=(a,b)=>`<div class="tile"><small>${a}</small><b class="num">${b}</b></div>`;
 setH($('pos'),tile('Posisi x',S.x+' cm')+tile('Posisi y',S.y+' cm')+tile('Yaw',S.yaw+'°')+
  tile('Target',S.tgt?'WP'+S.tgt+(S.np===2?' · putar':''):'-')+tile('Kecepatan badan',S.vx+' / '+S.vy)+tile('Putaran',S.wz+' rad/s'));
 $('trailInfo').textContent='Jejak X, Y, Yaw: '+trail.length+' titik (direkam saat robot digerakkan manual). Hijau = jejak, garis pendek di WP = arah hadap.';

 // motor
 let mh='';
 [0,1,2].forEach(i=>{const w=Math.min(Math.abs(S.rpm[i])/LIM*50,50),[lv,tx]=diag(i),c=COL[i];
  mh+=`<div class="card"><h2><span style="color:${c}">●</span> Motor ${i+1}</h2><div class="big num">${fx(S.rpm[i])}</div><div class="mut num">RPM · target ${fx(S.sp[i])}</div>`+
   `<div class="bar" style="margin:8px 0"><i style="left:${S.rpm[i]>=0?50:50-w}%;width:${w}%;background:${c}"></i></div><span class="${lv}">● ${tx}</span></div>`;
  hist[i].push(S.rpm[i]);histT[i].push(S.sp[i]);if(hist[i].length>N){hist[i].shift();histT[i].shift()}});
 setH($('motors'),mh);drawChart();

 // IMU
 const cal=S.ic||0,seg=v=>[1,2,3].map(k=>`<i class="${v>=k?'f':''}"></i>`).join('');
 setH($('imu'),S.imu?`<div style="display:flex;gap:14px;align-items:center"><svg width="84" height="84" viewBox="-43 -43 86 86"><circle r="40" fill="none" stroke="var(--ln)" stroke-width="2"/><g transform="rotate(${-S.yaw})"><path d="M0-34L8 10L0 4L-8 10Z" fill="var(--acc)"/></g></svg><div><div class="big num" style="font-size:30px">${fx(S.yaw)}°</div><div class="mut">yaw · mode ${S.in?'NDOF':'IMU'}</div></div></div>`+
  `<div class="cal" style="margin-top:8px"><span>SYS</span><span class="seg">${seg(cal>>6&3)}</span><span>GYR</span><span class="seg">${seg(cal>>4&3)}</span><span>ACC</span><span class="seg">${seg(cal>>2&3)}</span><span>MAG</span><span class="seg">${seg(cal&3)}</span></div>`
  :'<b class="bad">tidak terdeteksi</b><div class="mut">Cek 3V3, SDA GPIO8, SCL GPIO9. Heading lock dan putar ke arah waypoint mati.</div>');

 // ultrasonik
 const near=S.d.filter(v=>v>=0).sort((a,b)=>a-b)[0];
 setH($('us'),'<div class="kv">'+S.d.map((v,i)=>`<span>${S.sa[i]}°</span><b class="num ${v>=0&&v<25?'bad':''}">${v<0?'-':fx(v)+' cm'}</b>`).join('')+'</div>'+
  `<div class="hint">${near===undefined?'Tidak ada rintangan terdeteksi':'Rintangan terdekat '+fx(near)+' cm'}</div>`+
  `<div class="kv" style="margin-top:6px">${[['IR depan',1],['IR kiri',2],['IR kanan',4]].map(([n,m])=>`<span>${n}</span><b class="num ${(S.ir||0)&m?'bad':''}">${(S.ir||0)&m?'ada benda':'-'}</b>`).join('')}</div>`);

 // koreksi dinding (R8): median kiri/kanan vs referensi waypoint aktif
 
 const WST=['tidak dipakai','koreksi aktif','fallback encoder (sensor tidak valid / miring samping)','offset melebihi batas','menghindar (ditahan)'],wv=(m,i)=>S.dv&&S.dv[i]&&m>=0?fx(m):'-',wr=r=>r>0?fx(r):'-';
 setH($('wc'),'<div class="kv">'+`<span>Kiri ukur / ref</span><b class="num">${wv(S.wl,1)} / ${wr(S.wlr)} cm</b><span>Kanan ukur / ref</span><b class="num">${wv(S.wr,2)} / ${wr(S.wrr)} cm</b><span>Status</span><b class="${S.wstat===1?'ok':S.wstat?'bad':''}">${WST[S.wstat]||'-'}</b><span>Retry leg</span><b class="num">${S.rty}</b>${S.rst?'<span>Referensi</span><b class="bad">tidak cocok, rekam ulang</b>':''}<span>Sesi tiba / gagal / retry / koreksi</span><b class="num">${S.ss.join(' / ')}</b>`+'</div>'+
  `<div class="kv" style="margin-top:8px"><span>Kecepatan efektif</span><b class="num">batas ${S.spd} → sat ${S.sat}% → lereng ${S.slf} → hindar ${S.as} → ${S.re} RPM</b><span>Jarak sisi terkecil</span><b class="num ${S.smin>=0&&S.smin<8?'bad':''}">${S.smin<0?'-':S.smin+' cm'}</b><span>Tiba terakhir</span><b class="num">${S.arr&&S.arr.q?arrTxt(S.arr):'-'}</b></div>`);

 // waypoint
 const key=JSON.stringify(S.wp)+S.tgt+'|'+(S.rts?JSON.stringify(S.rts):'')+wpAll;
 if(key!==wpKey){wpKey=key;buildWpm()}
 const key2=JSON.stringify(S.wp);
 if(key2!==wpKey2&&!(document.activeElement&&document.activeElement.closest('#wp'))){wpKey2=key2;buildWp()}

 // PID
 const t=$('tune');t.textContent='auto-tune: '+TUNE[S.tune]+(S.tune==3&&S.tw?' - '+TW[S.tw]:'');t.className='chip '+(S.tune==2?'ok':S.tune==3?'bad':'warn');t.style.display=S.tune?'':'none';
 for(let i=0;i<3;i++)for(const k of ['kp','ki','kd']){const el=$(k+i);if(el&&document.activeElement!==el&&!el.dataset.d)el.value=S[k][i]}
 renderPre();mRender();drawMap();extra();
}

function asst(){
 if(S.terrain===undefined)return;
 const tc=S.terrain==='FLAT'?'g':S.terrain==='SLOPE'?'y':'r',tr=Math.max(0,Math.min(1,S.encoder_trust));
 const b=(txt,c)=>`<span class="bdg ${c}">${txt}</span>`;
 setH($('asst'),`<div class="row" style="margin-top:0">${b(S.terrain,tc)}${S.stuck?b('STUCK','r'):''}${S.slip?b('SLIP','r'):''}${S.pose_degraded?b('POSE TIDAK PASTI','y'):''}</div>`+
  `<div class="kv" style="margin-top:8px">${[['Pitch / Roll',S.pitch+'° / '+S.roll+'°'],['Galat yaw',S.yaw_error+'°'],['Kepercayaan encoder',Math.round(tr*100)+' %'],
   ['PID yaw',`Kp ${S.pid.kp} · Ki ${S.pid.ki} · Kd ${S.pid.kd}`],['Auto-tune',`<b class="${S.pid.status==='DONE'?'ok':'warn'}">${S.pid.status}</b>`]].map(([a,c])=>`<span>${a}</span><b class="num">${c}</b>`).join('')}</div>`+
  `<div class="bar" style="margin-top:8px"><i style="left:0;width:${tr*100}%;background:${tr<.3?'var(--bad)':tr<1?'var(--warn)':'var(--ok)'}"></i></div>`);
}
function extra(){ asst();
 const a=$('aSafe');setH(a,S.sf?`<b>ROBOT DIBERHENTIKAN PENGAMAN</b><br>${SF[S.sf]}.<div class="row"><button class="go" onclick="send({cmd:'resume'})">Lanjut (buka kunci)</button><span class="mut">Periksa robot dulu. Di gamepad: tekan Start.</span></div>`:'');a.style.display=S.sf?'block':'none';
 const b=$('aPose');b.className='alert '+(S.pl?'':'info');b.style.display=(S.pl||S.pr)?'block':'none';
 setH(b,S.pl?`<b>Posisi tidak valid.</b> Robot terdeteksi diangkat atau dipindah, atau IMU ter-reset. Letakkan robot di titik 0 dengan arah hadap seperti saat waypoint direkam, lalu tekan tombol ini. Preset dan navigasi ditolak sampai titik 0 ditetapkan.<div class="row"><button class="go" onclick="setZero()">Set titik 0 di sini</button></div>`
  :S.pr?`<b>Posisi dipulihkan dari memori.</b> Benar hanya bila robot tidak dipindah dan menghadap arah yang sama saat dimatikan. Bila ragu, letakkan di titik 0 lalu tekan tombol ini.<div class="row"><button onclick="setZero()">Set titik 0 di sini</button></div>`:'');
 if(!jogInit||$('jogWp').options.length!==S.wp.length){const sel=$('jogWp'),keep=sel.selectedIndex;sel.innerHTML=S.wp.map((w,i)=>`<option value="${i+1}">WP${i+1}${i<4?' (preset '+PRE[i]+')':''}</option>`).join('');sel.selectedIndex=Math.max(0,keep);jogInit=1}
 document.querySelectorAll('[data-jog]').forEach(e=>{const d=+e.dataset.jog;e.classList.toggle('on',S.jg===d);e.disabled=S.sf>0||S.mode===1||S.mode===3});
 chip('jgStat',S.jg?'jalan: '+JOGN[S.jg]+' ('+S.cs[3]+' RPM)':'diam',S.jg?'warn':'mut');
 const sl=$('spSl');if(document.activeElement!==sl){sl.value=S.cs[0];$('spVal').textContent=S.cs[0]}
 {const cg=$('cGrip');if(cg){cg.textContent='Gripper: '+(S.gp?'ON (jepit)':'OFF');cg.className='chip '+(S.gp?'warn':'mut')}}
 for(let r=0;r<4;r++){const el=$('rt'+r),ls=(S.rts&&S.rts[r])||[];if(el&&document.activeElement!==el&&!el.dataset.d)el.value=ls.map(rtTok).join(',');
  const rg=$('rg'+r);if(rg)rg.disabled=!ls.length||(S.ps&&S.ps.includes(2));
  const st=$('rs'+r);if(st)st.textContent=(S.ps&&S.ps[r]===2)?'jalan':(ls.length?ls.filter(n=>n<100).length+' titik'+(ls.some(n=>n>=100)?' + gripper':''):'kosong')}
 [['spT',1],['spN',2]].forEach(([id,k])=>{const el=$(id);if(document.activeElement!==el&&!el.dataset.d)el.value=S.cs[k]});
 const c=$('safeOn');if(document.activeElement!==c)c.checked=!!S.so;
 chip('safeChip',S.sf?'TERKUNCI':S.so?'normal':'MATI',S.sf?'bad':S.so?'ok':'warn');
 setH($('safeKv'),KV([['Duty terbesar',S.sfv[3]+' %'],['Pembatas daya',S.sat<100?'<b class="warn">aktif '+S.sat+' %</b>':'tidak aktif'],['Gyro',S.sfv[2]+' °/s'],['Baterai',S.bat<0?'tidak dipasang':S.bat+' V']]));
}
addEventListener('resize',()=>{if(S){drawMap();drawChart()}});
</script></body></html>
)HTML";
