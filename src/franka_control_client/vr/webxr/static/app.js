// WebXR teleop: streams the right controller grip pose + buttons to the Python server over /ws.
// Renders nothing (transparent clear -> passthrough in immersive-ar).
"use strict";

const $ = (id) => document.getElementById(id);
const logLine = (msg) => { $("log").textContent = `${new Date().toLocaleTimeString()} ${msg}\n` + $("log").textContent.slice(0, 2000); };

// ---- WebSocket with auto-reconnect and ping/pong ----
// After the headset sleeps (taken off), the socket can be dead without onclose ever firing
// (server heartbeat closed it, close frame lost over adb reverse). The pong watchdog catches that.
const PONG_TIMEOUT_MS = 3000;
let ws = null;
let rttMs = null;
let pingId = 0;
let lastPong = 0;
let reconnectTimer = null;

function scheduleReconnect() {
  if (reconnectTimer === null) reconnectTimer = setTimeout(() => { reconnectTimer = null; connect(); }, 1000);
}

function connect() {
  const sock = new WebSocket(`ws://${location.host}/ws`);
  ws = sock;
  lastPong = performance.now();
  sock.onopen = () => {
    if (sock !== ws) return;
    lastPong = performance.now();
    $("ws").textContent = "connected"; $("ws").className = "ok";
    logLine("WebSocket connected");
    buzz([[0.6, 80]]);
  };
  sock.onclose = () => {
    if (sock !== ws) return; // a socket the watchdog already replaced
    $("ws").textContent = "disconnected, retrying"; $("ws").className = "bad";
    rttMs = null;
    scheduleReconnect();
  };
  sock.onerror = () => sock.close();
  sock.onmessage = (ev) => {
    if (sock !== ws) return;
    const msg = JSON.parse(ev.data);
    if (msg.type === "pong") { lastPong = performance.now(); rttMs = lastPong - msg.t; }
  };
}

// drop a socket that stopped answering and open a new one
function dropConnection(why) {
  const old = ws;
  ws = null;
  rttMs = null;
  if (old) { try { old.close(); } catch (_) { /* already dead */ } }
  $("ws").textContent = `disconnected (${why}), retrying`; $("ws").className = "bad";
  logLine(`WebSocket ${why}, reconnecting`);
  buzz([[1.0, 150], [0, 100], [1.0, 150]]);
  scheduleReconnect();
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

setInterval(() => {
  if (ws && ws.readyState === WebSocket.OPEN && performance.now() - lastPong > PONG_TIMEOUT_MS) {
    dropConnection("no pong");
    return;
  }
  send({ type: "ping", id: pingId++, t: performance.now(), rtt: rttMs });
  $("rtt").textContent = rttMs === null ? "–" : `${rttMs.toFixed(1)} ms`;
}, 1000);

// back from sleep / tab switch: check the link right away instead of waiting for the watchdog
document.addEventListener("visibilitychange", () => {
  if (document.visibilityState === "visible" && ws && ws.readyState === WebSocket.OPEN &&
      performance.now() - lastPong > PONG_TIMEOUT_MS) dropConnection("stale after sleep");
});

// ---- haptics on the right controller: nothing is rendered in XR, so this is the only in-headset feedback ----
// one short pulse: connected; two strong pulses: connection lost
let hapticActuator = null;
async function buzz(pattern) {
  if (!hapticActuator) return;
  for (const [intensity, ms] of pattern) {
    try { if (intensity > 0) await hapticActuator.pulse(intensity, ms); else await new Promise((r) => setTimeout(r, ms)); }
    catch (_) { return; }
  }
}

// ---- send rate ----
let sentSinceTick = 0;
setInterval(() => { $("hz").textContent = `${sentSinceTick} Hz`; sentSinceTick = 0; }, 1000);

// ---- XR ----
let session = null;
let refSpace = null;
let gl = null;
let seq = 0;
let stickPrev = false;

async function pickMode() {
  if (!navigator.xr) return null;
  if (await navigator.xr.isSessionSupported("immersive-ar")) return "immersive-ar";
  if (await navigator.xr.isSessionSupported("immersive-vr")) return "immersive-vr";
  return null;
}

async function enterXR() {
  let mode = "immersive-ar";
  try {
    session = await navigator.xr.requestSession(mode, { requiredFeatures: ["local-floor"] });
  } catch (e) {
    logLine(`immersive-ar failed (${e}), trying immersive-vr`);
    mode = "immersive-vr";
    session = await navigator.xr.requestSession(mode, { requiredFeatures: ["local-floor"] });
  }
  const canvas = document.createElement("canvas");
  gl = canvas.getContext("webgl", { xrCompatible: true, alpha: true });
  // frames are not delivered without a base layer
  session.updateRenderState({ baseLayer: new XRWebGLLayer(session, gl) });
  refSpace = await session.requestReferenceSpace("local-floor");
  refSpace.addEventListener("reset", () => { send({ type: "recenter" }); logLine("recentered"); });
  session.addEventListener("visibilitychange", () => {
    $("xr").textContent = `${mode} (${session.visibilityState})`;
    if (session.visibilityState !== "visible") send({ type: "reset" });
    else if (ws && ws.readyState === WebSocket.OPEN && performance.now() - lastPong > PONG_TIMEOUT_MS) {
      dropConnection("stale after sleep");
    }
  });
  session.addEventListener("end", () => {
    send({ type: "session_end" });
    session = null;
    hapticActuator = null;
    $("xr").textContent = "none";
    $("enter").disabled = false;
    logLine("XR session ended");
  });
  $("xr").textContent = `${mode} (visible)`;
  $("enter").disabled = true;
  logLine(`entered ${mode}`);
  session.requestAnimationFrame(onFrame);
}

function onFrame(t, frame) {
  const s = frame.session;
  s.requestAnimationFrame(onFrame);

  const layer = s.renderState.baseLayer;
  gl.bindFramebuffer(gl.FRAMEBUFFER, layer.framebuffer);
  gl.clearColor(0, 0, 0, 0);
  gl.clear(gl.COLOR_BUFFER_BIT);

  // headset orientation, for the realign button (thumbstick press)
  const viewer = frame.getViewerPose(refSpace);
  const hq = viewer ? viewer.transform.orientation : null;

  for (const src of s.inputSources) {
    // skip hand tracking (no gamepad) and the left controller
    if (src.handedness !== "right" || !src.gamepad || !src.gripSpace) continue;
    // re-read every frame: the controller object is replaced after it sleeps/reconnects
    hapticActuator = (src.gamepad.hapticActuators && src.gamepad.hapticActuators[0]) || null;
    const pose = frame.getPose(src.gripSpace, refSpace);
    if (!pose) continue;
    const p = pose.transform.position;
    const q = pose.transform.orientation;
    const b = src.gamepad.buttons;
    const msg = {
      type: "pose",
      seq: seq++,
      t: performance.now(),
      pos: [p.x, p.y, p.z],
      rot: [q.x, q.y, q.z, q.w],
      trigger: b[0] ? b[0].value : 0,
      grip: b[1] ? b[1].value : 0,
      a: !!(b[4] && b[4].pressed),
      b: !!(b[5] && b[5].pressed),
      stick: !!(b[3] && b[3].pressed), // xr-standard: 3 = thumbstick press
      head: hq ? [hq.x, hq.y, hq.z, hq.w] : null,
    };
    if (msg.stick && !stickPrev) { buzz([[0.8, 40], [0, 60], [0.8, 40]]); logLine("realign"); }
    stickPrev = msg.stick;
    send(msg);
    sentSinceTick++;
    if (seq % 10 === 0) {
      $("ctrl").textContent =
        `pos ${msg.pos.map((v) => v.toFixed(2)).join(" ")}  trig ${msg.trigger.toFixed(2)}  grip ${msg.grip.toFixed(2)}` +
        `${msg.a ? "  A" : ""}${msg.b ? "  B" : ""}`;
    }
  }
}

(async () => {
  connect();
  const mode = await pickMode();
  if (!mode) {
    $("xr").textContent = "WebXR not available (needs localhost or https)";
    return;
  }
  $("enter").disabled = false;
  $("enter").addEventListener("click", () => enterXR().catch((e) => logLine(`XR failed: ${e}`)));
})();
