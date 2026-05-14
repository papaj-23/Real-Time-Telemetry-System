const canvas = document.querySelector("#scene");
const statusDot = document.querySelector("#status-dot");
const statusText = document.querySelector("#status-text");
const wsUrlInput = document.querySelector("#ws-url");
const connectButton = document.querySelector("#connect-button");

const values = {
  roll: document.querySelector("#roll-value"),
  pitch: document.querySelector("#pitch-value"),
  yaw: document.querySelector("#yaw-value"),
  temp: document.querySelector("#temp-value"),
  sample: document.querySelector("#sample-value"),
  accelX: document.querySelector("#accel-x"),
  accelY: document.querySelector("#accel-y"),
  accelZ: document.querySelector("#accel-z"),
  gyroX: document.querySelector("#gyro-x"),
  gyroY: document.querySelector("#gyro-y"),
  gyroZ: document.querySelector("#gyro-z"),
};

let websocket = null;
let previousTimestamp = null;
let previousBrowserTime = performance.now();
let yawDeg = 0;
let hasOrientation = false;

const COMPLEMENTARY_ALPHA = 0.9;
const VISUAL_Y_ROTATION_RAD = Math.PI / 2;
const GYRO_CALIBRATION_SAMPLES = 120;

const gyroBias = {
  x: -1.235,
  y: 0.258,
  z: 0.365,
};

const levelOffset = {
  roll: 0.7,
  pitch: -3.0,
};

const gyroCalibration = {
  active: true,
  count: 0,
  sumX: 0,
  sumY: 0,
  sumZ: 0,
  sumRoll: 0,
  sumPitch: 0,
};

const orientation = {
  roll: 0,
  pitch: 0,
  yaw: 0,
};

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101214);

const camera = new THREE.PerspectiveCamera(45, 1, 0.1, 100);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

const orbit = {
  azimuth: THREE.MathUtils.degToRad(40),
  elevation: THREE.MathUtils.degToRad(28),
  distance: 7.6,
  target: new THREE.Vector3(0, 0, 0),
};

let isOrbiting = false;
let lastPointerX = 0;
let lastPointerY = 0;

const ambientLight = new THREE.HemisphereLight(0xd8f4ff, 0x202529, 1.5);
scene.add(ambientLight);

const keyLight = new THREE.DirectionalLight(0xffffff, 2.4);
keyLight.position.set(4, 6, 5);
scene.add(keyLight);

const model = new THREE.Group();
scene.add(model);

const visualModel = new THREE.Group();
visualModel.rotation.y = VISUAL_Y_ROTATION_RAD;
model.add(visualModel);

const boardGeometry = new THREE.BoxGeometry(2.9, 0.16, 1.72);
const boardMaterial = new THREE.MeshStandardMaterial({
  color: 0x334743,
  roughness: 0.7,
  metalness: 0.04,
});
const board = new THREE.Mesh(boardGeometry, boardMaterial);
visualModel.add(board);

const chipGeometry = new THREE.BoxGeometry(0.76, 0.12, 0.58);
const chipMaterial = new THREE.MeshStandardMaterial({
  color: 0x22272a,
  roughness: 0.78,
  metalness: 0.03,
});
const chip = new THREE.Mesh(chipGeometry, chipMaterial);
chip.position.y = 0.14;
visualModel.add(chip);

const traceMaterial = new THREE.MeshStandardMaterial({
  color: 0x7d8275,
  roughness: 0.64,
  metalness: 0.1,
});

const railMaterial = new THREE.MeshStandardMaterial({
  color: 0x5f6f6b,
  roughness: 0.62,
  metalness: 0.06,
});

const padMaterial = new THREE.MeshStandardMaterial({
  color: 0x8f876f,
  roughness: 0.48,
  metalness: 0.24,
});

for (const x of [-1.18, 1.18]) {
  const rail = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.08, 1.36), railMaterial);
  rail.position.set(x, 0.14, 0);
  visualModel.add(rail);
}

for (const x of [-0.94, 0.94]) {
  for (const z of [-0.58, 0.58]) {
    const pad = new THREE.Mesh(new THREE.BoxGeometry(0.26, 0.026, 0.22), padMaterial);
    pad.position.set(x, 0.106, z);
    visualModel.add(pad);
  }
}

for (const z of [-0.36, 0.36]) {
  const trace = new THREE.Mesh(new THREE.BoxGeometry(1.62, 0.018, 0.04), traceMaterial);
  trace.position.set(0, 0.102, z);
  visualModel.add(trace);
}

for (const x of [-0.42, 0.42]) {
  const trace = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.018, 0.72), traceMaterial);
  trace.position.set(x, 0.104, 0);
  visualModel.add(trace);
}

const grid = new THREE.GridHelper(8, 16, 0x2b3439, 0x20272c);
scene.add(grid);

const axes = new THREE.AxesHelper(2.3);
scene.add(axes);

centerObjectAtOrigin(visualModel);

function setStatus(text, isConnected) {
  statusText.textContent = text;
  statusDot.classList.toggle("connected", isConnected);
  connectButton.textContent = isConnected ? "Rozłącz" : "Połącz";
}

function formatDeg(value) {
  return `${value.toFixed(1)}°`;
}

function formatFloat(value, unit, digits = 3) {
  return `${Number(value).toFixed(digits)} ${unit}`;
}

function normalizeDeg(value) {
  return ((value + 180) % 360 + 360) % 360 - 180;
}

function centerObjectAtOrigin(object) {
  const box = new THREE.Box3().setFromObject(object);
  const center = box.getCenter(new THREE.Vector3());
  object.children.forEach((child) => {
    child.position.sub(center);
  });
}

function resetGyroCalibration() {
  gyroCalibration.active = true;
  gyroCalibration.count = 0;
  gyroCalibration.sumX = 0;
  gyroCalibration.sumY = 0;
  gyroCalibration.sumZ = 0;
  gyroCalibration.sumRoll = 0;
  gyroCalibration.sumPitch = 0;
}

function updateGyroCalibration(gx, gy, gz, rollFromAccel, pitchFromAccel) {
  if (!gyroCalibration.active) {
    return false;
  }

  gyroCalibration.count += 1;
  gyroCalibration.sumX += gx;
  gyroCalibration.sumY += gy;
  gyroCalibration.sumZ += gz;
  gyroCalibration.sumRoll += rollFromAccel;
  gyroCalibration.sumPitch += pitchFromAccel;

  setStatus(
    `Kalibracja poziomu ${gyroCalibration.count}/${GYRO_CALIBRATION_SAMPLES}`,
    true,
  );

  if (gyroCalibration.count < GYRO_CALIBRATION_SAMPLES) {
    return true;
  }

  gyroBias.x = gyroCalibration.sumX / gyroCalibration.count;
  gyroBias.y = gyroCalibration.sumY / gyroCalibration.count;
  gyroBias.z = gyroCalibration.sumZ / gyroCalibration.count;
  levelOffset.roll = gyroCalibration.sumRoll / gyroCalibration.count;
  levelOffset.pitch = gyroCalibration.sumPitch / gyroCalibration.count;
  gyroCalibration.active = false;
  previousTimestamp = null;
  yawDeg = 0;
  hasOrientation = false;
  setStatus("Połączony", true);
  console.info("Gyro bias", { ...gyroBias });
  console.info("Level offset", { ...levelOffset });
  return true;
}

function estimateTimestampDt(timestamp) {
  if (previousTimestamp === null || timestamp === previousTimestamp) {
    previousTimestamp = timestamp;
    previousBrowserTime = performance.now();
    return 0;
  }

  const delta = timestamp - previousTimestamp;
  previousTimestamp = timestamp;

  if (delta > 0 && delta < 10_000) {
    return delta / 1000;
  }

  if (delta > 0 && delta < 10_000_000) {
    return delta / 1_000_000;
  }

  const now = performance.now();
  const browserDt = (now - previousBrowserTime) / 1000;
  previousBrowserTime = now;
  return browserDt;
}

function updateFromSample(payload) {
  const data = payload.data;
  if (!data) {
    return;
  }

  const ax = Number(data.accel_x);
  const ay = Number(data.accel_y);
  const az = Number(data.accel_z);
  const gx = Number(data.gyro_x);
  const gy = Number(data.gyro_y);
  const gz = Number(data.gyro_z);
  const timestamp = Number(data.timestamp);

  values.temp.textContent = `${Number(data.temp).toFixed(1)}°C`;
  values.sample.textContent = payload.sample ?? "0";
  values.accelX.textContent = formatFloat(ax, "g");
  values.accelY.textContent = formatFloat(ay, "g");
  values.accelZ.textContent = formatFloat(az, "g");

  const rawRollFromAccel = THREE.MathUtils.radToDeg(Math.atan2(ay, az));
  const rawPitchFromAccel = THREE.MathUtils.radToDeg(
    Math.atan2(-ax, Math.sqrt(ay * ay + az * az)),
  );

  if (updateGyroCalibration(gx, gy, gz, rawRollFromAccel, rawPitchFromAccel)) {
    values.gyroX.textContent = formatFloat(gx - gyroBias.x, "deg/s");
    values.gyroY.textContent = formatFloat(gy - gyroBias.y, "deg/s");
    values.gyroZ.textContent = formatFloat(gz - gyroBias.z, "deg/s");
    return;
  }

  const correctedGx = gx - gyroBias.x;
  const correctedGy = gy - gyroBias.y;
  const correctedGz = gz - gyroBias.z;

  const rollFromAccel = rawRollFromAccel - levelOffset.roll;
  const pitchFromAccel = rawPitchFromAccel - levelOffset.pitch;
  const dt = estimateTimestampDt(timestamp);

  if (!hasOrientation || dt <= 0) {
    orientation.roll = rollFromAccel;
    orientation.pitch = pitchFromAccel;
    yawDeg = 0;
    hasOrientation = true;
  } else {
    const predictedRoll = orientation.roll + correctedGx * dt;
    const predictedPitch = orientation.pitch + correctedGy * dt;
    orientation.roll =
      COMPLEMENTARY_ALPHA * predictedRoll +
      (1 - COMPLEMENTARY_ALPHA) * rollFromAccel;
    orientation.pitch =
      COMPLEMENTARY_ALPHA * predictedPitch +
      (1 - COMPLEMENTARY_ALPHA) * pitchFromAccel;
    yawDeg += correctedGz * dt;
  }

  orientation.roll = normalizeDeg(orientation.roll);
  orientation.pitch = normalizeDeg(orientation.pitch);
  orientation.yaw = yawDeg;

  values.roll.textContent = formatDeg(orientation.roll);
  values.pitch.textContent = formatDeg(orientation.pitch);
  values.yaw.textContent = formatDeg(orientation.yaw);
  values.temp.textContent = `${Number(data.temp).toFixed(1)}°C`;
  values.sample.textContent = payload.sample ?? "0";
  values.accelX.textContent = formatFloat(ax, "g");
  values.accelY.textContent = formatFloat(ay, "g");
  values.accelZ.textContent = formatFloat(az, "g");
  values.gyroX.textContent = formatFloat(correctedGx, "deg/s");
  values.gyroY.textContent = formatFloat(correctedGy, "deg/s");
  values.gyroZ.textContent = formatFloat(correctedGz, "deg/s");
}

function connect() {
  if (websocket) {
    websocket.close();
    return;
  }

  previousTimestamp = null;
  yawDeg = 0;
  hasOrientation = false;
  resetGyroCalibration();
  setStatus("Łączenie", false);

  websocket = new WebSocket(wsUrlInput.value.trim());

  websocket.addEventListener("open", () => {
    setStatus("Połączony", true);
  });

  websocket.addEventListener("message", (event) => {
    try {
      updateFromSample(JSON.parse(event.data));
    } catch (error) {
      console.warn("Nieprawidłowa wiadomość WebSocket", error);
    }
  });

  websocket.addEventListener("close", () => {
    websocket = null;
    previousTimestamp = null;
    hasOrientation = false;
    setStatus("Rozłączony", false);
  });

  websocket.addEventListener("error", () => {
    setStatus("Błąd połączenia", false);
  });
}

function resizeRenderer() {
  const { clientWidth, clientHeight } = canvas.parentElement;
  renderer.setSize(clientWidth, clientHeight, false);
  camera.aspect = clientWidth / Math.max(clientHeight, 1);
  camera.updateProjectionMatrix();
  updateCamera();
}

function updateCamera() {
  const radiusXZ = Math.cos(orbit.elevation) * orbit.distance;
  camera.position.set(
    Math.sin(orbit.azimuth) * radiusXZ,
    Math.sin(orbit.elevation) * orbit.distance,
    Math.cos(orbit.azimuth) * radiusXZ,
  );
  camera.lookAt(orbit.target);
}

function animate() {
  requestAnimationFrame(animate);

  model.rotation.order = "YXZ";
  model.rotation.y = THREE.MathUtils.degToRad(orientation.yaw);
  model.rotation.x = THREE.MathUtils.degToRad(orientation.pitch);
  model.rotation.z = THREE.MathUtils.degToRad(-orientation.roll);

  renderer.render(scene, camera);
}

function startOrbit(event) {
  isOrbiting = true;
  lastPointerX = event.clientX;
  lastPointerY = event.clientY;
  canvas.setPointerCapture(event.pointerId);
}

function moveOrbit(event) {
  if (!isOrbiting) {
    return;
  }

  const dx = event.clientX - lastPointerX;
  const dy = event.clientY - lastPointerY;
  lastPointerX = event.clientX;
  lastPointerY = event.clientY;

  orbit.azimuth -= dx * 0.008;
  orbit.elevation = THREE.MathUtils.clamp(
    orbit.elevation + dy * 0.008,
    THREE.MathUtils.degToRad(-78),
    THREE.MathUtils.degToRad(78),
  );
  updateCamera();
}

function stopOrbit(event) {
  isOrbiting = false;
  if (canvas.hasPointerCapture(event.pointerId)) {
    canvas.releasePointerCapture(event.pointerId);
  }
}

function zoomOrbit(event) {
  event.preventDefault();
  orbit.distance = THREE.MathUtils.clamp(
    orbit.distance + event.deltaY * 0.006,
    3.2,
    14,
  );
  updateCamera();
}

connectButton.addEventListener("click", connect);
canvas.addEventListener("pointerdown", startOrbit);
canvas.addEventListener("pointermove", moveOrbit);
canvas.addEventListener("pointerup", stopOrbit);
canvas.addEventListener("pointercancel", stopOrbit);
canvas.addEventListener("wheel", zoomOrbit, { passive: false });
window.addEventListener("resize", resizeRenderer);

resizeRenderer();
animate();
connect();
