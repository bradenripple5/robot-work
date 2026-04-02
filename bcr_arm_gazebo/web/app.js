const poseButtons = document.getElementById("pose-buttons");
const jointForm = document.getElementById("joint-form");
const statusOutput = document.getElementById("status-output");
const presetDuration = document.getElementById("preset-duration");
const customDuration = document.getElementById("custom-duration");
const sendCustomButton = document.getElementById("send-custom");
const copyCurrentButton = document.getElementById("copy-current");
const refreshButton = document.getElementById("refresh");
const resetCameraButton = document.getElementById("reset-camera");
const viewerCanvas = document.getElementById("viewer");

let lastStatus = null;

const ARM_CONFIG = {
  baseHeight: 0.05,
  chain: [
    { offset: [0, 0, 0.025], axis: "z", drawFrom: [0, 0, 0], drawTo: [0, 0, 0.2], radius: 18 },
    { offset: [0, 0, 0.2], axis: "x", drawFrom: [0, 0, 0], drawTo: [0.065, 0, 0], radius: 16 },
    { offset: [0.065, 0, 0], axis: "z", drawFrom: [0, 0, 0], drawTo: [0, 0, 0.41], radius: 14 },
    { offset: [0, 0, 0.41], axis: "x", drawFrom: [0, 0, 0], drawTo: [-0.065, 0, 0], radius: 12 },
    { offset: [-0.065, 0, 0], axis: "z", drawFrom: [0, 0, 0], drawTo: [0, 0, 0.31], radius: 11 },
    { offset: [0, 0, 0.31], axis: "x", drawFrom: [0, 0, 0], drawTo: [0.06, 0, 0], radius: 9 },
    { offset: [0.06, 0, 0], axis: "z", drawFrom: [0, 0, 0], drawTo: [0, 0, 0.105], radius: 8 },
  ],
};

const camera = {
  yaw: 0.9,
  pitch: 0.55,
  distance: 2.4,
  target: { x: 0, y: 0, z: 0.45 },
};

const pointerState = {
  dragging: false,
  mode: "orbit",
  x: 0,
  y: 0,
};

let engine;
let scene;
let viewerCamera;
const segmentState = [];

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function identity() {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

function multiplyMatrix(a, b) {
  const out = identity();
  for (let row = 0; row < 4; row += 1) {
    for (let col = 0; col < 4; col += 1) {
      out[row][col] = 0;
      for (let i = 0; i < 4; i += 1) {
        out[row][col] += a[row][i] * b[i][col];
      }
    }
  }
  return out;
}

function translationMatrix(x, y, z) {
  const matrix = identity();
  matrix[0][3] = x;
  matrix[1][3] = y;
  matrix[2][3] = z;
  return matrix;
}

function rotationMatrix(axis, angle) {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  if (axis === "x") {
    return [
      [1, 0, 0, 0],
      [0, c, -s, 0],
      [0, s, c, 0],
      [0, 0, 0, 1],
    ];
  }
  return [
    [c, -s, 0, 0],
    [s, c, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

function applyMatrix(matrix, point) {
  return {
    x: matrix[0][0] * point.x + matrix[0][1] * point.y + matrix[0][2] * point.z + matrix[0][3],
    y: matrix[1][0] * point.x + matrix[1][1] * point.y + matrix[1][2] * point.z + matrix[1][3],
    z: matrix[2][0] * point.x + matrix[2][1] * point.y + matrix[2][2] * point.z + matrix[2][3],
  };
}

function subtract(a, b) {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

function dot(a, b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

function cross(a, b) {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  };
}

function normalize(vector) {
  const length = Math.hypot(vector.x, vector.y, vector.z) || 1;
  return {
    x: vector.x / length,
    y: vector.y / length,
    z: vector.z / length,
  };
}

function getCameraPosition() {
  return {
    x: camera.target.x + camera.distance * Math.cos(camera.pitch) * Math.sin(camera.yaw),
    y: camera.target.y + camera.distance * Math.cos(camera.pitch) * Math.cos(camera.yaw),
    z: camera.target.z + camera.distance * Math.sin(camera.pitch),
  };
}

function toBabylonVector(point) {
  return new BABYLON.Vector3(point.x, point.z, point.y);
}

function buildArmSegments(positions) {
  let transform = identity();
  return ARM_CONFIG.chain.map((joint, index) => {
    transform = multiplyMatrix(transform, translationMatrix(...joint.offset));
    transform = multiplyMatrix(transform, rotationMatrix(joint.axis, positions[index] || 0));
    return {
      start: applyMatrix(transform, {
        x: joint.drawFrom[0],
        y: joint.drawFrom[1],
        z: joint.drawFrom[2],
      }),
      end: applyMatrix(transform, {
        x: joint.drawTo[0],
        y: joint.drawTo[1],
        z: joint.drawTo[2],
      }),
      radius: joint.radius,
    };
  });
}

function orientCylinderBetweenPoints(mesh, start, end) {
  const delta = end.subtract(start);
  const length = delta.length();
  if (length < 1e-5) {
    return;
  }

  const dir = delta.scale(1 / length);
  mesh.position = start.add(end).scale(0.5);
  mesh.scaling.y = length;

  const up = BABYLON.Axis.Y;
  const axis = BABYLON.Vector3.Cross(up, dir);
  const dotVal = clamp(BABYLON.Vector3.Dot(up, dir), -1, 1);

  if (axis.lengthSquared() < 1e-8) {
    if (dotVal < 0) {
      mesh.rotationQuaternion = BABYLON.Quaternion.RotationAxis(BABYLON.Axis.X, Math.PI);
    } else {
      mesh.rotationQuaternion = BABYLON.Quaternion.Identity();
    }
    return;
  }

  const angle = Math.acos(dotVal);
  mesh.rotationQuaternion = BABYLON.Quaternion.RotationAxis(axis.normalize(), angle);
}

function createGrid() {
  const linePoints = [];
  for (let i = -6; i <= 6; i += 1) {
    linePoints.push([toBabylonVector({ x: -0.9, y: i * 0.15, z: 0 }), toBabylonVector({ x: 0.9, y: i * 0.15, z: 0 })]);
    linePoints.push([toBabylonVector({ x: i * 0.15, y: -0.9, z: 0 }), toBabylonVector({ x: i * 0.15, y: 0.9, z: 0 })]);
  }
  const grid = BABYLON.MeshBuilder.CreateLineSystem("grid", { lines: linePoints, updatable: false }, scene);
  grid.color = new BABYLON.Color3(0.2, 0.35, 0.45);
}

function createAxes() {
  const xAxis = BABYLON.MeshBuilder.CreateLines("axisX", {
    points: [toBabylonVector({ x: 0, y: 0, z: 0 }), toBabylonVector({ x: 0.22, y: 0, z: 0 })],
  }, scene);
  xAxis.color = new BABYLON.Color3(1.0, 0.42, 0.42);

  const yAxis = BABYLON.MeshBuilder.CreateLines("axisY", {
    points: [toBabylonVector({ x: 0, y: 0, z: 0 }), toBabylonVector({ x: 0, y: 0.22, z: 0 })],
  }, scene);
  yAxis.color = new BABYLON.Color3(0.49, 1.0, 0.7);

  const zAxis = BABYLON.MeshBuilder.CreateLines("axisZ", {
    points: [toBabylonVector({ x: 0, y: 0, z: 0 }), toBabylonVector({ x: 0, y: 0, z: 0.22 })],
  }, scene);
  zAxis.color = new BABYLON.Color3(0.49, 0.82, 1.0);
}

function initScene() {
  engine = new BABYLON.Engine(viewerCanvas, true, { preserveDrawingBuffer: true, stencil: true });
  scene = new BABYLON.Scene(engine);
  scene.clearColor = new BABYLON.Color4(0.06, 0.09, 0.12, 1.0);

  viewerCamera = new BABYLON.FreeCamera("viewerCamera", new BABYLON.Vector3(0, 0, 0), scene);
  viewerCamera.minZ = 0.01;
  viewerCamera.maxZ = 25;

  const hemi = new BABYLON.HemisphericLight("hemi", new BABYLON.Vector3(0.2, 1, 0.3), scene);
  hemi.intensity = 0.9;
  const key = new BABYLON.DirectionalLight("key", new BABYLON.Vector3(-0.6, -1, -0.5), scene);
  key.intensity = 0.7;

  createGrid();
  createAxes();

  const baseMat = new BABYLON.StandardMaterial("baseMat", scene);
  baseMat.diffuseColor = new BABYLON.Color3(0.15, 0.22, 0.27);
  const base = BABYLON.MeshBuilder.CreateCylinder("base", { diameter: 0.16, height: ARM_CONFIG.baseHeight }, scene);
  base.material = baseMat;
  base.position = toBabylonVector({ x: 0, y: 0, z: ARM_CONFIG.baseHeight * 0.5 });

  ARM_CONFIG.chain.forEach((joint, index) => {
    const segmentMat = new BABYLON.StandardMaterial(`segmentMat${index}`, scene);
    const evenColor = new BABYLON.Color3(0.84, 1.0, 0.34);
    const oddColor = new BABYLON.Color3(0.49, 0.82, 1.0);
    segmentMat.diffuseColor = index % 2 === 0 ? evenColor : oddColor;

    const segmentMesh = BABYLON.MeshBuilder.CreateCylinder(`segment${index}`, {
      diameter: Math.max(0.012, joint.radius * 0.0022),
      height: 1,
      tessellation: 20,
    }, scene);
    segmentMesh.material = segmentMat;

    const jointMesh = BABYLON.MeshBuilder.CreateSphere(`joint${index}`, {
      diameter: Math.max(0.015, joint.radius * 0.0028),
      segments: 12,
    }, scene);
    const jointMat = new BABYLON.StandardMaterial(`jointMat${index}`, scene);
    jointMat.diffuseColor = new BABYLON.Color3(0.96, 0.98, 1.0);
    jointMesh.material = jointMat;

    segmentState.push({ segmentMesh, jointMesh });
  });

  window.addEventListener("resize", () => engine.resize());

  engine.runRenderLoop(() => {
    updateViewer();
    scene.render();
  });
}

function updateViewer() {
  const eye = toBabylonVector(getCameraPosition());
  const target = toBabylonVector(camera.target);
  viewerCamera.position.copyFrom(eye);
  viewerCamera.setTarget(target);

  const positions = lastStatus?.joint_state?.positions || [0, 0, 0, 0, 0, 0, 0];
  const segments = buildArmSegments(positions);

  segments.forEach((segment, index) => {
    const state = segmentState[index];
    if (!state) return;
    const start = toBabylonVector(segment.start);
    const end = toBabylonVector(segment.end);
    state.jointMesh.position.copyFrom(start);
    orientCylinderBetweenPoints(state.segmentMesh, start, end);
  });
}

function formatPositions(positions) {
  return positions.map((value) => Number(value).toFixed(3)).join(", ");
}

function renderJointInputs(names) {
  jointForm.innerHTML = "";
  names.forEach((jointName, index) => {
    const wrapper = document.createElement("label");
    wrapper.className = "joint-field";
    wrapper.innerHTML = `
      <span>${jointName}</span>
      <input data-index="${index}" type="number" step="0.05" value="0">
    `;
    jointForm.appendChild(wrapper);
  });
}

function getJointValues() {
  return [...jointForm.querySelectorAll("input")].map((input) => Number(input.value));
}

function setJointValues(values) {
  [...jointForm.querySelectorAll("input")].forEach((input, index) => {
    input.value = values[index] ?? 0;
  });
}

async function fetchStatus() {
  const response = await fetch("/api/status");
  const status = await response.json();
  lastStatus = status;
  renderStatus(status);
  if (!jointForm.children.length) {
    renderJointInputs(status.joint_state.names);
  }
  if (!poseButtons.children.length) {
    renderPoseButtons(status.predefined_poses);
  }
}

function renderPoseButtons(poses) {
  poseButtons.innerHTML = "";
  Object.keys(poses).forEach((poseName) => {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = poseName;
    button.addEventListener("click", async () => {
      await fetch("/api/pose", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          pose: poseName,
          duration: Number(presetDuration.value || 5),
        }),
      });
      await fetchStatus();
    });
    poseButtons.appendChild(button);
  });
}

function renderStatus(status) {
  const positions = status.joint_state.positions;
  const lastCommand = status.last_command;
  statusOutput.textContent = [
    `Joint positions: ${formatPositions(positions)}`,
    `Joint state age: ${(Date.now() / 1000 - status.joint_state.stamp).toFixed(2)}s`,
    `Last command type: ${lastCommand.type}`,
    `Last command duration: ${lastCommand.duration ?? "n/a"}`,
    `Last command target: ${
      Array.isArray(lastCommand.target) ? formatPositions(lastCommand.target) : "n/a"
    }`,
  ].join("\n");
}

sendCustomButton.addEventListener("click", async () => {
  await fetch("/api/joints", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({
      positions: getJointValues(),
      duration: Number(customDuration.value || 5),
    }),
  });
  await fetchStatus();
});

copyCurrentButton.addEventListener("click", () => {
  if (lastStatus) {
    setJointValues(lastStatus.joint_state.positions);
  }
});

refreshButton.addEventListener("click", fetchStatus);

resetCameraButton.addEventListener("click", () => {
  camera.yaw = 0.9;
  camera.pitch = 0.55;
  camera.distance = 2.4;
  camera.target = { x: 0, y: 0, z: 0.45 };
});

viewerCanvas.addEventListener("mousedown", (event) => {
  pointerState.dragging = true;
  pointerState.mode = event.shiftKey ? "pan" : "orbit";
  pointerState.x = event.clientX;
  pointerState.y = event.clientY;
});

window.addEventListener("mouseup", () => {
  pointerState.dragging = false;
});

window.addEventListener("mousemove", (event) => {
  if (!pointerState.dragging) {
    return;
  }

  const dx = event.clientX - pointerState.x;
  const dy = event.clientY - pointerState.y;
  pointerState.x = event.clientX;
  pointerState.y = event.clientY;

  if (pointerState.mode === "pan") {
    const panScale = camera.distance * 0.0015;
    camera.target.x -= dx * panScale * Math.cos(camera.yaw);
    camera.target.y += dx * panScale * Math.sin(camera.yaw);
    camera.target.z += dy * panScale;
    return;
  }

  camera.yaw -= dx * 0.008;
  camera.pitch = clamp(camera.pitch + dy * 0.008, -1.35, 1.35);
});

viewerCanvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  camera.distance = clamp(camera.distance * (1 + event.deltaY * 0.001), 0.7, 5.5);
});

initScene();
fetchStatus();
setInterval(fetchStatus, 1500);
