"use strict";

const NORCAL_BOUNDS = [[-125.0, 35.0], [-119.5, 40.8]];
const PHYSICS_HZ = 10;
const SEND_HZ = 2;

const State = {
  lat: 37.4611, lon: -122.1150, alt: 1500,
  // hdg is interpreted as MAGNETIC — same convention the iPad PFD displays
  // (AHRS heading is magnetic per GDL90 spec, and the PFD converts ownship
  // ground track to magnetic via the WMM grid before showing it). Position
  // physics adds the local declination to convert to true before stepping.
  hdg: 270, gs: 110, vs: 0,
  running: false,
  lastTickMs: 0,
  start: { lat: 37.4611, lon: -122.1150, alt: 1500, hdg: 270, gs: 110 },
  teleportMode: false,
  // Local magnetic declination (east positive). Updated from each /pos
  // response — server has pygeomag and computes against NOAA WMM-2025.
  // Initial 0 means the very first physics tick may drift a fraction of
  // a nautical mile before the first /pos round-trip lands; once that
  // first response arrives subsequent ticks use the live value and the
  // aircraft tracks its course centerline.
  decl: 0,
};

const Keys = {};
window.addEventListener("keydown", e => {
  if (e.repeat) return;
  if (e.code === "Space") { e.preventDefault(); togglePlay(); return; }
  if (e.code === "KeyR")  { resetToStart(); return; }
  if (e.code === "KeyT")  { State.teleportMode = !State.teleportMode; document.body.style.cursor = State.teleportMode ? "crosshair" : ""; return; }
  if (e.code === "Equal" || e.code === "NumpadAdd") { State.gs = Math.min(300, State.gs + 10); return; }
  if (e.code === "Minus" || e.code === "NumpadSubtract") { State.gs = Math.max(0, State.gs - 10); return; }
  Keys[e.code] = true;
  if (["ArrowLeft","ArrowRight","ArrowUp","ArrowDown"].includes(e.code)) e.preventDefault();
});
window.addEventListener("keyup", e => { Keys[e.code] = false; });

function setStart(opt) {
  const [lat, lon, hdg, alt] = opt.split(",").map(Number);
  State.start = { lat, lon, alt, hdg, gs: 110 };
  resetToStart();
}
function resetToStart() {
  State.lat = State.start.lat; State.lon = State.start.lon;
  State.alt = State.start.alt; State.hdg = State.start.hdg; State.gs = State.start.gs;
  State.vs = 0;
  if (map) map.jumpTo({ center: [State.lon, State.lat], zoom: 10 });
  updateMarker(); updateHud();
}

function togglePlay() {
  State.running = !State.running;
  const btn = document.getElementById("playPause");
  btn.textContent = State.running ? "Stop" : "Start";
  btn.classList.toggle("running", State.running);
  if (State.running) {
    State.lastTickMs = performance.now();
    setConn("live");
  } else {
    setConn("idle");
  }
}
function setConn(kind, msg) {
  const el = document.getElementById("connState");
  el.className = "pill " + kind;
  el.textContent = msg || kind;
}

function tickPhysics() {
  const now = performance.now();
  const dt = Math.min(0.5, (now - State.lastTickMs) / 1000);
  State.lastTickMs = now;
  if (!State.running) return;

  const shift = Keys.ShiftLeft || Keys.ShiftRight;
  const turnMult = shift ? 2 : 1;
  const climbMult = shift ? 2 : 1;

  // Heading: 3°/sec standard rate while held
  if (Keys.ArrowLeft)  State.hdg = (State.hdg - 3 * turnMult * dt + 360) % 360;
  if (Keys.ArrowRight) State.hdg = (State.hdg + 3 * turnMult * dt) % 360;

  // Altitude: 500 fpm while held
  const prevAlt = State.alt;
  if (Keys.ArrowUp)   State.alt = Math.min(45000, State.alt + 500 * climbMult * dt / 60);
  if (Keys.ArrowDown) State.alt = Math.max(0,     State.alt - 500 * climbMult * dt / 60);
  State.vs = ((State.alt - prevAlt) / dt) * 60;

  // Position integration (nm = deg * 60; ft→nm via 1/6076; gs in kt = nm/hr).
  // State.hdg is magnetic; convert to true via local declination so the
  // airplane physically moves along the bearing the iPad PFD is showing.
  const dNm = State.gs * dt / 3600;  // nm traveled in dt
  const trueHdg = (State.hdg + State.decl + 360) % 360;
  const hdgRad = trueHdg * Math.PI / 180;
  State.lat += (dNm / 60) * Math.cos(hdgRad);
  State.lon += (dNm / 60) * Math.sin(hdgRad) / Math.cos(State.lat * Math.PI / 180);
}

async function sendPosition() {
  try {
    const r = await fetch("/pos", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({
        lat: State.lat, lon: State.lon, alt: State.alt,
        hdg: State.hdg, gs: State.gs, vs: State.vs,
      }),
    });
    if (!r.ok) { setConn("error", "send err"); return; }
    // Pull back the server-side declination (NOAA WMM-2025 via pygeomag)
    // for the next physics tick. Cheap to refresh every send — declination
    // shifts ~0.1°/100 nm, so even a stale value from minutes ago is fine.
    try {
      const data = await r.json();
      if (typeof data.decl === "number") State.decl = data.decl;
    } catch (e) {}
  } catch (e) {
    setConn("error", "send err");
  }
}

let sendAccum = 0;
function loop() {
  tickPhysics();
  if (State.running) {
    sendAccum += 1 / PHYSICS_HZ;
    if (sendAccum >= 1 / SEND_HZ) {
      sendAccum = 0;
      sendPosition();
    }
  }
  updateMarker();
  updateHud();
  requestAnimationFrame(loop);
}
setInterval(() => {}, 1000 / PHYSICS_HZ);  // keep JS alive; loop via rAF

function updateHud() {
  document.getElementById("hLat").textContent = State.lat.toFixed(5);
  document.getElementById("hLon").textContent = State.lon.toFixed(5);
  document.getElementById("hAlt").textContent = Math.round(State.alt);
  document.getElementById("hHdg").textContent = Math.round(State.hdg);
  document.getElementById("hGs").textContent  = Math.round(State.gs);
  document.getElementById("hVs").textContent  = Math.round(State.vs);
}

// ---- Map & marker -----------------------------------------------------------

let map = null;
let marker = null;

async function initMap() {
  let vfrCycle = null;
  try {
    const r = await fetch("/config");
    const cfg = await r.json();
    if (cfg.vfr_cycle) vfrCycle = cfg.vfr_cycle;
    if (cfg.target) document.getElementById("target").value = cfg.target;
  } catch (e) { /* /config unavailable — render OSM only */ }

  const sources = {
    "osm": {
      type: "raster",
      tiles: ["https://tile.openstreetmap.org/{z}/{x}/{y}.png"],
      tileSize: 256, minzoom: 0, maxzoom: 19,
      attribution: "© OpenStreetMap",
    },
  };
  const layers = [{ id: "osm", type: "raster", source: "osm", maxzoom: 6 }];
  if (vfrCycle) {
    sources["vfr"] = {
      type: "raster",
      tiles: [`https://cdn.tachbase.com/tile-cache/faa-charts/vfr/${vfrCycle}/{z}/{x}/{y}.png`],
      tileSize: 256, minzoom: 5, maxzoom: 11,
      attribution: "FAA VFR sectional (via Tachbase CDN)",
    };
    layers.push({ id: "vfr", type: "raster", source: "vfr", minzoom: 5 });
  }

  map = new maplibregl.Map({
    container: "map",
    style: {
      version: 8,
      sources,
      layers,
    },
    center: [State.lon, State.lat],
    zoom: 10,
    maxBounds: NORCAL_BOUNDS,
  });

  map.addControl(new maplibregl.NavigationControl({ showCompass: false }), "bottom-right");

  // MapLibre's default keyboard handler pans the map with arrow keys and
  // steals them from our piloting handler once the canvas has focus. Off.
  map.keyboard.disable();

  const el = document.createElement("div");
  el.className = "ownship-marker";
  el.innerHTML = `
    <svg viewBox="0 0 32 32" width="32" height="32">
      <polygon points="16,2 28,28 16,22 4,28"
               fill="#ff00ff" stroke="#000" stroke-width="1.5" stroke-linejoin="round"/>
    </svg>`;
  marker = new maplibregl.Marker({ element: el, rotationAlignment: "map" })
    .setLngLat([State.lon, State.lat])
    .addTo(map);

  map.on("click", e => {
    if (!State.teleportMode) return;
    State.lat = e.lngLat.lat; State.lon = e.lngLat.lng;
    State.teleportMode = false;
    document.body.style.cursor = "";
    updateMarker();
  });
}

function updateMarker() {
  if (!marker) return;
  marker.setLngLat([State.lon, State.lat]);
  marker.setRotation(State.hdg);
}

// ---- Wire up UI -------------------------------------------------------------

document.getElementById("playPause").addEventListener("click", togglePlay);
document.getElementById("startAt").addEventListener("change", e => setStart(e.target.value));
document.getElementById("target").addEventListener("change", async e => {
  // Accept `[v6]:port` and `v4:port` (URL bracketed-host form). Rsplit on
  // colon for v4 so we don't slice an unbracketed v6 literal in half.
  const v = e.target.value.trim();
  const bracketed = v.match(/^\[(.+)\]:(\d+)$/);
  let host, port;
  if (bracketed) {
    host = bracketed[1]; port = bracketed[2];
  } else {
    const i = v.lastIndexOf(":");
    host = i >= 0 ? v.slice(0, i) : v;
    port = i >= 0 ? v.slice(i + 1) : "4000";
  }
  await fetch("/target", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({ host, port: Number(port) }),
  });
});

// ---- Traffic scenarios ------------------------------------------------------
//
// Each scenario is a /traffic/spawn body relative to current ownship.
// Tuned against ownship default 110 kt + receiver TA thresholds:
//   caution: range ≤ 6 nm AND |dAlt| ≤ 1200 AND closing AND tau ≤ 35 s
//   warning: tau ≤ 20 s AND range ≤ 3 nm AND |dAlt| ≤ 600
//
// Numbers picked so each scenario fires in ≤ 5 s after spawn — slow-developing
// crossers / overtakers are aviation-realistic but a pain for testing.
const SCENARIOS = {
  // Head-on 1 nm, target 150 kt → closure 260 kt, tau 13.8 s → red warning.
  warning:  { clock: 12, range_nm: 1.0, alt_diff_ft: 0,     track: "head_on",    gs_kt: 150, callsign: "TGT-WARN" },
  // Head-on 2.5 nm, target 150 kt → closure 260 kt, tau 34.6 s → amber
  // (just inside the 35 s caution gate, well outside the 20 s warning gate).
  caution:  { clock: 12, range_nm: 2.5, alt_diff_ft: 0,     track: "head_on",    gs_kt: 150, callsign: "TGT-CAUT" },
  // 3 o'clock, 1 nm, target heads ownship_hdg - 90 (crosses toward our path
  // from the right). Closure 150 kt, tau 24 s → caution.
  cross_r:  { clock: 3,  range_nm: 1.0, alt_diff_ft: 0,     track: "cross_left", gs_kt: 150, callsign: "TGT-CRS"  },
  // 6 o'clock 1 nm parallel, target 250 kt vs ownship 110 → closure 140 kt,
  // tau 25.7 s → caution.
  overtake: { clock: 6,  range_nm: 1.0, alt_diff_ft: 0,     track: "parallel",   gs_kt: 250, callsign: "TGT-OVR"  },
  // 12 o'clock 1.5 nm, target 1500 ft below → outside ±1200 ft band → NO
  // alert. Control test that the vertical gate is wired correctly.
  below:    { clock: 12, range_nm: 1.5, alt_diff_ft: -1500, track: "head_on",    gs_kt: 150, callsign: "TGT-LO"   },
  // 12 o'clock 1.5 nm, target 800 ft below climbing 2000 fpm. Caution
  // immediately (tau ~21 s, dAlt 800 inside ±1200 outside ±600), escalates
  // to warning as alt closes.
  climbing: { clock: 12, range_nm: 1.5, alt_diff_ft: -800,  track: "head_on",    gs_kt: 150, vspeed_fpm: 2000, callsign: "TGT-CLM" },
};

document.querySelectorAll(".scenario-buttons button").forEach(btn => {
  btn.addEventListener("click", async () => {
    const scn = SCENARIOS[btn.dataset.scn];
    if (!scn) return;
    btn.disabled = true;
    try {
      const r = await fetch("/traffic/spawn", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(scn),
      });
      const data = await r.json();
      if (data.error) console.warn("[traffic] spawn:", data.error);
    } finally {
      setTimeout(() => { btn.disabled = false; }, 250);
    }
    refreshTrafficList();
  });
});

document.getElementById("trafficClear").addEventListener("click", async () => {
  await fetch("/traffic/clear", { method: "POST" });
  refreshTrafficList();
});

async function refreshTrafficList() {
  try {
    const r = await fetch("/traffic");
    const data = await r.json();
    const ul = document.getElementById("trafficList");
    ul.innerHTML = "";
    for (const t of data.targets) {
      const li = document.createElement("li");
      li.textContent = `${t.callsign}  ${t.alt} ft  ${t.track}° / ${t.gs} kt`;
      ul.appendChild(li);
    }
  } catch (e) { /* ignore */ }
}
setInterval(refreshTrafficList, 1500);

initMap();
loop();
