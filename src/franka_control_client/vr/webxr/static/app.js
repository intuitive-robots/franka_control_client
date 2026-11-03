// WebXR teleop: streams the right controller grip pose + buttons to the Python server over /ws.
// Renders nothing (transparent clear -> passthrough in immersive-ar).
"use strict";

const $ = (id) => document.getElementById(id);
const logLine = (msg) => { $("log").textContent = `${new Date().toLocaleTimeString()} ${msg}\n` + $("log").textContent.slice(0, 2000); };

// ---- WebSocket with auto-reconnect and ping/pong ----
let ws = null;
let rttMs = null;
let pingId = 0;

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { $("ws").textContent = "connected"; $("ws").className = "ok"; };
  ws.onclose = () => {
    $("ws").textContent = "disconnected, retrying"; $("ws").className = "bad";
    rttMs = null;
    setTimeout(connect, 1000);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "pong") rttMs = performance.now() - msg.t;
  };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

setInterval(() => {
  send({ type: "ping", id: pingId++, t: performance.now(), rtt: rttMs });
  $("rtt").textContent = rttMs === null ? "–" : `${rttMs.toFixed(1)} ms`;
}, 1000);

// ---- send rate ----
let sentSinceTick = 0;
setInterval(() => { $("hz").textContent = `${sentSinceTick} Hz`; sentSinceTick = 0; }, 1000);

// ---- XR ----
let session = null;
let refSpace = null;
let gl = null;
let seq = 0;

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
  refSpace.addEventListener("reset", () => { send({ type: "reset" }); logLine("recentered"); });
  session.addEventListener("visibilitychange", () => {
    $("xr").textContent = `${mode} (${session.visibilityState})`;
    if (session.visibilityState !== "visible") send({ type: "reset" });
  });
  session.addEventListener("end", () => {
    send({ type: "session_end" });
    session = null;
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

  for (const src of s.inputSources) {
    // skip hand tracking (no gamepad) and the left controller
    if (src.handedness !== "right" || !src.gamepad || !src.gripSpace) continue;
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
    };
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
