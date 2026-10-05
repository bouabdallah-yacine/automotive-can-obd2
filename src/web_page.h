// Web dashboard served by the ESP32: gauges, warning lights, CAN bus sniffer, OBD-II diagnostics
#pragma once
static const char WEB_PAGE[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>CAN Network</title>
<style>
:root{--bg:#0e1214;--panel:#161c1f;--line:#263034;--ink:#e4ebe9;--mut:#83918f;--acc:#4fc3a1;--amb:#f2a33a;--red:#ef5b4c;--blue:#5aa9e6;
 --btn:#1d2427;--pre:#0b0f10;--st:#1e3b33;--stp:#3d2f17;--sto:#40201c;--warnb:#5a2a24;--warnt:#ffb3aa;color-scheme:dark}
:root[data-theme="light"]{--bg:#eef1f0;--panel:#ffffff;--line:#d5dcda;--ink:#17201f;--mut:#5f6d6b;--acc:#16876a;--amb:#c77700;--red:#c63a2b;--blue:#2b78c2;
 --btn:#f4f6f5;--pre:#f6f8f7;--st:#dff1ea;--stp:#fbecd2;--sto:#fadcd7;--warnb:#e8b4ad;--warnt:#a3281b;color-scheme:light}
@media(prefers-color-scheme:light){:root:not([data-theme="dark"]){--bg:#eef1f0;--panel:#ffffff;--line:#d5dcda;--ink:#17201f;--mut:#5f6d6b;--acc:#16876a;--amb:#c77700;--red:#c63a2b;--blue:#2b78c2;
 --btn:#f4f6f5;--pre:#f6f8f7;--st:#dff1ea;--stp:#fbecd2;--sto:#fadcd7;--warnb:#e8b4ad;--warnt:#a3281b;color-scheme:light}}
#theme{min-width:42px}
.gt{fill:var(--mut);font-family:ui-monospace,monospace}.gv{fill:var(--ink)}.gtrack{stroke:var(--line)}.gtick{stroke:var(--mut)}.gval{stroke:var(--acc)}.gred{stroke:var(--red)}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,-apple-system,"Segoe UI",sans-serif;padding:16px}
main{max-width:1100px;margin:0 auto;display:grid;gap:14px}
header{display:flex;justify-content:space-between;align-items:baseline;flex-wrap:wrap;gap:8px}
h1{font-size:20px;margin:0}h2{font-size:12px;margin:0 0 10px;color:var(--mut);text-transform:uppercase;letter-spacing:.07em;font-weight:600}
.mono{font-family:ui-monospace,"SFMono-Regular",Consolas,monospace;font-variant-numeric:tabular-nums}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;padding:14px;min-width:0}
.cockpit{display:grid;grid-template-columns:1fr 1fr minmax(0,260px);gap:14px}
@media(max-width:820px){.cockpit{grid-template-columns:1fr 1fr}.lights{grid-column:1/-1}}
@media(max-width:520px){.cockpit{grid-template-columns:1fr}}
svg.g{width:100%;max-width:340px;display:block;margin:0 auto}
.gp{display:flex;align-items:center;justify-content:center}
.lights{display:grid;gap:8px;align-content:start}
.lamp{display:flex;align-items:center;gap:10px;padding:8px 10px;border:1px solid var(--line);border-radius:8px;color:var(--mut)}
.lamp i{width:12px;height:12px;border-radius:50%;background:var(--line)}
.lamp.on{color:var(--ink)}.lamp.on.amb i{background:var(--amb);box-shadow:0 0 10px var(--amb)}
.lamp.on.red i{background:var(--red);box-shadow:0 0 10px var(--red)}.lamp.on.blue i{background:var(--blue);box-shadow:0 0 10px var(--blue)}
.cols{display:grid;grid-template-columns:minmax(0,1.6fr) minmax(0,1fr);gap:14px}@media(max-width:820px){.cols{grid-template-columns:1fr}}
.sniff{height:320px;overflow:auto;font-size:12.5px}
table{width:100%;border-collapse:collapse}td,th{padding:3px 6px;text-align:left;white-space:nowrap}th{color:var(--mut);font-weight:500;font-size:12px;position:sticky;top:0;background:var(--panel)}
tr.err td{color:var(--red)}td.id{font-weight:600}
.stats{display:flex;flex-wrap:wrap;gap:6px 18px;margin-bottom:10px;color:var(--mut)}.stats b{color:var(--ink)}
.node{display:flex;justify-content:space-between;padding:5px 0;border-bottom:1px solid var(--line)}.node:last-child{border:0}
.st{font-size:12px;padding:1px 8px;border-radius:99px;background:var(--st);color:var(--acc)}.st.passive{background:var(--stp);color:var(--amb)}.st.off{background:var(--sto);color:var(--red)}
.btns{display:flex;flex-wrap:wrap;gap:6px}
button{font:inherit;background:var(--btn);color:var(--ink);border:1px solid var(--line);border-radius:7px;padding:6px 11px;cursor:pointer}
button:hover{border-color:var(--acc)}button.warn{border-color:var(--warnb);color:var(--warnt)}button:focus-visible,input:focus-visible{outline:2px solid var(--acc)}
pre{margin:10px 0 0;padding:10px;background:var(--pre);border:1px solid var(--line);border-radius:8px;font-size:12.5px;white-space:pre-wrap;min-height:90px}
.ans{color:var(--acc);font-weight:600}
label{color:var(--mut);font-size:12px}input[type=range]{width:100%;accent-color:var(--acc)}
</style></head><body><main>
<header><h1>Automotive CAN network <span class="mono" style="color:var(--mut);font-size:13px">500 kbit/s</span></h1>
<span style="display:flex;gap:10px;align-items:center"><span class="mono" id="conn" style="color:var(--mut)">connecting…</span><button id="theme" title="Toggle theme" aria-label="Toggle theme">◐</button></span></header>

<section class="cockpit">
 <div class="panel gp"><svg class="g" viewBox="0 0 200 168" id="spd" role="img" aria-label="Speedometer"></svg></div>
 <div class="panel gp"><svg class="g" viewBox="0 0 200 168" id="rpm" role="img" aria-label="Tachometer"></svg></div>
 <div class="panel lights">
  <h2>Warning lights</h2>
  <div class="lamp amb" id="l_mil"><i></i>CHECK ENGINE <span class="mono" id="ndtc"></span></div>
  <div class="lamp amb" id="l_abs"><i></i>ABS active</div>
  <div class="lamp red" id="l_brk"><i></i>Brake</div>
  <div class="lamp red" id="l_lost"><i></i>Engine comm. lost (U0100)</div>
  <div class="lamp blue" id="l_temp"><i></i>Temperature <span class="mono" id="tmp"></span></div>
  <label for="ped">Pedal (web) <span class="mono" id="pedv">potentiometer</span></label>
  <input id="ped" type="range" min="0" max="100" value="0">
  <div class="btns"><button onclick="cmd('pedal','-1')">Pedal: potentiometer</button><button onclick="cmd('brake','1')">Brake for 2 s</button></div>
 </div>
</section>

<section class="cols">
 <div class="panel"><h2>Bus sniffer (live frames)</h2>
  <div class="stats mono"><span>load <b id="load"></b></span><span>frames <b id="ok"></b></span><span>errors <b id="err"></b></span><span>arbitrations <b id="arb"></b></span></div>
  <div class="sniff"><table class="mono"><thead><tr><th>time</th><th>ID</th><th>sender</th><th>DLC</th><th>data</th><th></th></tr></thead><tbody id="frames"></tbody></table></div>
 </div>
 <div style="display:grid;gap:14px;align-content:start">
  <div class="panel"><h2>ECUs</h2><div id="nodes"></div>
   <p class="mono" id="lastarb" style="color:var(--mut);font-size:12px;margin:8px 0 0"></p>
   <div class="btns" style="margin-top:10px"><button class="warn" onclick="cmd('error','40')">Noise on the engine (→ BUS-OFF)</button><button class="warn" onclick="cmd('fan','')">Cooling fan failure</button></div>
  </div>
  <div class="panel"><h2>Diagnostic OBD-II</h2>
   <div class="btns"><button onclick="obd('rpm')">RPM</button><button onclick="obd('speed')">Speed</button><button onclick="obd('temp')">Temperature</button><button onclick="obd('dtc')">Read DTCs</button><button onclick="obd('clear')">Clear DTCs</button><button onclick="obd('vin')">VIN (ISO-TP)</button></div>
   <pre class="mono" id="obd">Pick a request: the frames sent and received will appear here.</pre>
  </div>
 </div>
</section>
</main><script>
const $=id=>document.getElementById(id);
// Theme: follows the system by default, the button toggles light / dark (remembered in the browser)
(function(){const r=document.documentElement;let t=null;try{t=localStorage.getItem('theme')}catch(e){}
 if(t)r.dataset.theme=t;
 const cur=()=>r.dataset.theme||(matchMedia('(prefers-color-scheme: light)').matches?'light':'dark');
 const lab=()=>{$('theme').textContent=cur()=='dark'?'☀':'☾';$('theme').title=cur()=='dark'?'Switch to light mode':'Switch to dark mode'};
 $('theme').onclick=()=>{r.dataset.theme=cur()=='dark'?'light':'dark';try{localStorage.setItem('theme',r.dataset.theme)}catch(e){}lab()};lab()})();
function gauge(el,label,unit,max,step,red){
 const cx=100,cy=105,r=80,a0=-210,a1=30;let s='';
 const pt=(a,rr)=>[cx+rr*Math.cos(a*Math.PI/180),cy+rr*Math.sin(a*Math.PI/180)];
 const arc=(f,t,rr)=>{const[p,q]=pt(f,rr),[u,v]=pt(t,rr);return `M${p} ${q} A${rr} ${rr} 0 ${t-f>180?1:0} 1 ${u} ${v}`};
 s+=`<path d="${arc(a0,a1,r)}" fill="none" class="gtrack" stroke-width="10"/>`;
 if(red)s+=`<path d="${arc(a0+(a1-a0)*red/max,a1,r)}" fill="none" class="gred" stroke-width="10" opacity=".75"/>`;
 s+=`<path id="${el.id}_v" d="" fill="none" class="gval" stroke-width="10"/>`;
 for(let v=0;v<=max;v+=step){const a=a0+(a1-a0)*v/max,[x1,y1]=pt(a,r-14),[x,y]=pt(a,r-26);
  s+=`<line x1="${pt(a,r-6)[0]}" y1="${pt(a,r-6)[1]}" x2="${x1}" y2="${y1}" class="gtick"/>`;
  s+=`<text x="${x}" y="${y+3}" class="gt" font-size="9" text-anchor="middle">${max>1000?v/1000:v}</text>`}
 s+=`<line id="${el.id}_n" x1="${cx}" y1="${cy}" x2="${cx}" y2="${cy-r+16}" style="stroke:var(--ink)" stroke-width="3" stroke-linecap="round"/><circle cx="${cx}" cy="${cy}" r="5" class="gv"/>`;
 s+=`<text id="${el.id}_t" x="${cx}" y="${cy+36}" class="gv" font-size="22" font-weight="700" text-anchor="middle" font-family="ui-monospace,monospace">0</text>`;
 s+=`<text x="${cx}" y="${cy+56}" class="gt" font-size="9" text-anchor="middle">${label} · ${unit}${max>1000?' (dial ×1000)':''}</text>`;
 el.innerHTML=s;el._g={cx,cy,r,a0,a1,max,arc};
}
function setG(el,v){const g=el._g,f=Math.max(0,Math.min(1,v/g.max)),a=g.a0+(g.a1-g.a0)*f;
 $(el.id+'_n').setAttribute('transform',`rotate(${a+90} ${g.cx} ${g.cy})`);
 $(el.id+'_v').setAttribute('d',f>0.002?g.arc(g.a0,a,g.r):'');$(el.id+'_t').textContent=Math.round(v)}
gauge($('spd'),'speed','km/h',220,20,0);gauge($('rpm'),'engine speed','rpm',7000,1000,5500);
const lamp=(id,on)=>$(id).classList.toggle('on',!!on);
let since=0;const rows=[];
async function cmd(c,v){try{await fetch('/api/cmd?c='+c+'&v='+v)}catch(e){}}
$('ped').oninput=e=>{$('pedv').textContent=e.target.value+' %';cmd('pedal',e.target.value)};
async function obd(q){$('obd').textContent='request in progress…';
 try{const r=await (await fetch('/api/obd?q='+q)).json();$('obd').innerHTML=r.log.join('\n')+'\n<span class="ans">'+r.text+'</span>'}catch(e){$('obd').textContent='connection error'}}
async function refresh(){try{
 const s=await (await fetch('/api/state?since='+since)).json();$('conn').textContent='ESP32 online';
 setG($('spd'),s.speed);setG($('rpm'),s.rpm);
 lamp('l_mil',s.mil);$('ndtc').textContent=s.mil?'('+s.ndtc+')':'';lamp('l_abs',s.abs);lamp('l_brk',s.brake);lamp('l_lost',s.lost);
 lamp('l_temp',s.temp>105);$('tmp').textContent=s.temp.toFixed(0)+' °C';
 $('pedv').textContent=s.pedal<0?'potentiometer':s.pedal+' %';
 $('load').textContent=s.load.toFixed(1)+' %';$('ok').textContent=s.ok;$('err').textContent=s.err;$('arb').textContent=s.arb;
 $('nodes').innerHTML=s.nodes.map(n=>`<div class="node"><span>${n.name}</span><span class="mono" style="color:var(--mut)">TEC ${n.tec} · REC ${n.rec} <span class="st ${n.state=='BUS-OFF'?'off':n.state=='active'?'':'passive'}">${n.state}</span></span></div>`).join('');
 $('lastarb').textContent=s.lastArb||'';
 for(const f of s.frames){rows.unshift(f);since=Math.max(since,f.seq)}rows.length=Math.min(rows.length,60);
 $('frames').innerHTML=rows.map(f=>`<tr class="${f.err?'err':''}"><td>${(f.t/1000).toFixed(2)}</td><td class="id">0x${f.id.toString(16).toUpperCase().padStart(3,'0')}</td><td>${f.from}</td><td>${f.dlc}</td><td>${f.data}</td><td>${f.err?'error → retransmission':''}</td></tr>`).join('');
}catch(e){$('conn').textContent='ESP32 unreachable'}}
refresh();setInterval(refresh,300);
</script></body></html>)HTML";
