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
const viewerContext = viewerCanvas.getContext("2d");

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

function projectPoint(point) {
  const eye = getCameraPosition();
  const forward = normalize(subtract(camera.target, eye));
  const right = normalize(cross(forward, { x: 0, y: 0, z: 1 }));
  const up = cross(right, forward);
  const relative = subtract(point, eye);
  const xCam = dot(relative, right);
  const yCam = dot(relative, up);
  const zCam = dot(relative, forward);
  if (zCam <= 0.01) {
    return null;
  }

  const focal = Math.min(viewerCanvas.width, viewerCanvas.height) * 0.95;
  return {
    x: viewerCanvas.width * 0.5 + (xCam / zCam) * focal,
    y: viewerCanvas.height * 0.52 - (yCam / zCam) * focal,
    depth: zCam,
  };
}

function drawLine3d(start, end, color, width = 1) {
  const a = projectPoint(start);
  const b = projectPoint(end);
  if (!a || !b) {
    return null;
  }
  viewerContext.strokeStyle = color;
  viewerContext.lineWidth = width;
  viewerContext.beginPath();
  viewerContext.moveTo(a.x, a.y);
  viewerContext.lineTo(b.x, b.y);
  viewerContext.stroke();
  return { a, b };
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

function renderViewer() {
  const dpr = window.devicePixelRatio || 1;
  const bounds = viewerCanvas.getBoundingClientRect();
  const width = Math.max(1, Math.floor(bounds.width * dpr));
  const height = Math.max(1, Math.floor(bounds.height * dpr));
  if (viewerCanvas.width !== width || viewerCanvas.height !== height) {
    viewerCanvas.width = width;
    viewerCanvas.height = height;
  }

  viewerContext.clearRect(0, 0, viewerCanvas.width, viewerCanvas.height);

  for (let i = -6; i <= 6; i += 1) {
    drawLine3d(
      { x: -0.9, y: i * 0.15, z: 0 },
      { x: 0.9, y: i * 0.15, z: 0 },
      "rgba(124, 209, 255, 0.12)"
    );
    drawLine3d(
      { x: i * 0.15, y: -0.9, z: 0 },
      { x: i * 0.15, y: 0.9, z: 0 },
      "rgba(124, 209, 255, 0.12)"
    );
  }

  drawLine3d({ x: 0, y: 0, z: 0 }, { x: 0.22, y: 0, z: 0 }, "#ff6b6b", 2);
  drawLine3d({ x: 0, y: 0, z: 0 }, { x: 0, y: 0.22, z: 0 }, "#7dffb3", 2);
  drawLine3d({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0.22 }, "#7cd1ff", 2);

  const baseBottom = projectPoint({ x: 0, y: 0, z: 0 });
  const baseTop = projectPoint({ x: 0, y: 0, z: ARM_CONFIG.baseHeight });
  if (baseBottom && baseTop) {
    viewerContext.strokeStyle = "#253746";
    viewerContext.lineCap = "round";
    viewerContext.lineWidth = 42 * ((baseBottom.depth + baseTop.depth) * 0.5) ** -0.7;
    viewerContext.beginPath();
    viewerContext.moveTo(baseBottom.x, baseBottom.y);
    viewerContext.lineTo(baseTop.x, baseTop.y);
    viewerContext.stroke();
  }

  const positions = lastStatus?.joint_state?.positions || [0, 0, 0, 0, 0, 0, 0];
  const segments = buildArmSegments(positions)
    .map((segment) => {
      const start2d = projectPoint(segment.start);
      const end2d = projectPoint(segment.end);
      if (!start2d || !end2d) {
        return null;
      }
      return {
        ...segment,
        start2d,
        end2d,
        sortDepth: (start2d.depth + end2d.depth) * 0.5,
      };
    })
    .filter(Boolean)
    .sort((a, b) => b.sortDepth - a.sortDepth);

  segments.forEach((segment, index) => {
    viewerContext.strokeStyle = index % 2 === 0 ? "#d6ff57" : "#7cd1ff";
    viewerContext.lineCap = "round";
    viewerContext.lineWidth = segment.radius * segment.sortDepth ** -0.72;
    viewerContext.beginPath();
    viewerContext.moveTo(segment.start2d.x, segment.start2d.y);
    viewerContext.lineTo(segment.end2d.x, segment.end2d.y);
    viewerContext.stroke();

    viewerContext.fillStyle = "#f5fbff";
    viewerContext.beginPath();
    viewerContext.arc(
      segment.start2d.x,
      segment.start2d.y,
      Math.max(2, 7 * segment.start2d.depth ** -0.7),
      0,
      Math.PI * 2
    );
    viewerContext.fill();
  });

  requestAnimationFrame(renderViewer);
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

fetchStatus();
setInterval(fetchStatus, 1500);
renderViewer();
