import "./styles.css";

import RAPIER from "@dimforge/rapier3d-compat";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

const poseButtons = document.getElementById("pose-buttons");
const jointForm = document.getElementById("joint-form");
const statusOutput = document.getElementById("status-output");
const presetDuration = document.getElementById("preset-duration");
const customDuration = document.getElementById("custom-duration");
const directDuration = document.getElementById("direct-duration");
const sendCustomButton = document.getElementById("send-custom");
const sendDirectButton = document.getElementById("send-direct");
const sequenceButtons = document.getElementById("sequence-buttons");
const copyCurrentButton = document.getElementById("copy-current");
const refreshButton = document.getElementById("refresh");
const syncRobotButton = document.getElementById("sync-robot");
const syncStatus = document.getElementById("sync-status");
const resetCameraButton = document.getElementById("reset-camera");
const clearCommandsButton = document.getElementById("clear-commands");
const commandLog = document.getElementById("command-log");
const sceneItemSelect = document.getElementById("scene-item");
const placeItemButton = document.getElementById("place-item");
const removeItemButton = document.getElementById("remove-item");
const itemStatus = document.getElementById("item-status");
const directCommandInput = document.getElementById("direct-command");
const viewerCanvas = document.getElementById("viewer");
const viewerPanel = viewerCanvas.parentElement;

let lastStatus = null;
let commandHistory = [];
let backendConnected = false;
let localJointPositions = [0, 0, 0, 0, 0, 0, 0];
let targetJointPositions = [0, 0, 0, 0, 0, 0, 0];
let activeMotion = null;
let collisionState = {
  active: false,
  messages: [],
};
const JOINT_STATE_FRESHNESS_SEC = 1.0;

const ARM_CONFIG = {
  chain: [
    { offset: [0, 0, 0.025], axis: "z", drawTo: [0, 0, 0.2], radius: 0.04, color: 0xd6ff57 },
    { offset: [0, 0, 0.2], axis: "x", drawTo: [0.065, 0, 0], radius: 0.034, color: 0x7cd1ff },
    { offset: [0.065, 0, 0], axis: "z", drawTo: [0, 0, 0.41], radius: 0.032, color: 0xd6ff57 },
    { offset: [0, 0, 0.41], axis: "x", drawTo: [-0.065, 0, 0], radius: 0.028, color: 0x7cd1ff },
    { offset: [-0.065, 0, 0], axis: "z", drawTo: [0, 0, 0.31], radius: 0.026, color: 0xd6ff57 },
    { offset: [0, 0, 0.31], axis: "x", drawTo: [0.06, 0, 0], radius: 0.022, color: 0x7cd1ff },
    { offset: [0.06, 0, 0], axis: "z", drawTo: [0, 0, 0.105], radius: 0.02, color: 0xd6ff57 },
  ],
};

const ENVIRONMENT = {
  floorZ: 0,
  tableCenter: new THREE.Vector3(0.43, 0, 0.12),
  tableSize: new THREE.Vector3(0.5, 0.64, 0.24),
};

const PREDEFINED_POSES = {
  home: [0.0, -1.57, 0.0, 3.14, 0.0, 3.14, 0.0],
  ready: [0.0, -0.785, 0.0, -1.57, 0.0, 0.785, 0.0],
  stretch_up: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  forward_low: [0.0, -1.57, 0.0, 0.0, 0.0, 0.0, 0.0],
};
let currentPredefinedPoses = { ...PREDEFINED_POSES };
let currentPresetSignature = JSON.stringify(currentPredefinedPoses);

const JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"];

let scene;
let renderer;
let camera;
let controls;
let armRoot;
let tableMesh;
let boxMesh;
let linkStates = [];
let world;
let floorCollider;
let tableCollider;
let colliderHandles = [];
let rigidBodies = [];
let useCanvasFallback = false;
let fallbackContext = null;
let viewerNotice = null;
let webglMode = "unknown";
const sceneItems = {
  arm: true,
  table: false,
  box: false,
};

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function normalizeAngle(angle) {
  let value = Number(angle);
  while (value <= -Math.PI) {
    value += Math.PI * 2;
  }
  while (value > Math.PI) {
    value -= Math.PI * 2;
  }
  return value;
}

function radiansToDegrees(angle) {
  return normalizeAngle(angle) * (180 / Math.PI);
}

function degreesToRadians(angle) {
  return normalizeAngle(Number(angle) * (Math.PI / 180));
}

function shortestAngleDelta(from, to) {
  return normalizeAngle(to - from);
}

function formatPositions(positions) {
  return positions.map((value) => Number(value).toFixed(3)).join(", ");
}

function renderCommandHistory() {
  if (!commandHistory.length) {
    commandLog.textContent = "No commands sent yet.";
    return;
  }

  commandLog.innerHTML = "";
  [...commandHistory].reverse().forEach((entry) => {
    const item = document.createElement("div");
    item.className = "command-entry";
    item.innerHTML = `
      <strong>${entry.label}${entry.source ? ` · ${entry.source}` : ""}</strong>
      <span>Duration: ${entry.duration.toFixed(2)}s</span>
      <span>Target: ${formatPositions(entry.target)}</span>
      <span>Sent: ${new Date(entry.stamp).toLocaleTimeString()}</span>
    `;
    commandLog.appendChild(item);
  });
}

function renderItemStatus() {
  itemStatus.innerHTML = "";
  Object.entries(sceneItems).forEach(([key, present]) => {
    const row = document.createElement("div");
    row.className = "item-row";
    row.innerHTML = `
      <strong>${key}</strong>
      <span>${present ? "Present ✓" : "Absent"}</span>
    `;
    itemStatus.appendChild(row);
  });
}

function recordCommand(label, target, duration) {
  commandHistory.push({
    label,
    target: [...target],
    duration: Number(duration),
    stamp: Date.now(),
    source: "local-ui",
  });
  if (commandHistory.length > 24) {
    commandHistory = commandHistory.slice(-24);
  }
  renderCommandHistory();
}

function renderJointInputs(names) {
  jointForm.innerHTML = "";
  names.forEach((jointName, index) => {
    const wrapper = document.createElement("label");
    wrapper.className = "joint-field";
    wrapper.innerHTML = `
      <div class="joint-row">
        <span>${jointName}</span>
        <input data-role="slider" data-index="${index}" type="range" min="-180" max="180" step="1" value="0">
        <input data-role="number" data-index="${index}" type="number" min="-180" max="180" step="1" value="0">
      </div>
    `;
    jointForm.appendChild(wrapper);
  });

  jointForm.querySelectorAll('input[data-role="slider"]').forEach((input) => {
    input.addEventListener("input", onLiveJointInput);
  });
  jointForm.querySelectorAll('input[data-role="number"]').forEach((input) => {
    input.addEventListener("input", onLiveJointInput);
  });
}

function getJointValues() {
  return JOINT_NAMES.map((_, index) => {
    const input = jointForm.querySelector(`input[data-role="number"][data-index="${index}"]`);
    return degreesToRadians(input?.value ?? 0);
  });
}

function setJointValues(values) {
  JOINT_NAMES.forEach((_, index) => {
    const degrees = Math.round(radiansToDegrees(values[index] ?? 0));
    const numberInput = jointForm.querySelector(`input[data-role="number"][data-index="${index}"]`);
    const sliderInput = jointForm.querySelector(`input[data-role="slider"][data-index="${index}"]`);
    if (numberInput) {
      numberInput.value = String(degrees);
    }
    if (sliderInput) {
      sliderInput.value = String(degrees);
    }
  });
}

function getDisplayedPositions() {
  if (hasFreshBackendState()) {
    return lastStatus.joint_state.positions;
  }
  return localJointPositions;
}

function hasFreshBackendState() {
  if (!backendConnected || !lastStatus?.joint_state?.positions || !lastStatus?.joint_state?.stamp) {
    return false;
  }
  const ageSec = Date.now() / 1000 - Number(lastStatus.joint_state.stamp);
  return Number.isFinite(ageSec) && ageSec >= 0 && ageSec < JOINT_STATE_FRESHNESS_SEC;
}

function setSyncStatus(tone, message) {
  syncStatus.className = `sync-status ${tone}`;
  syncStatus.textContent = message;
}

function renderPoseButtons(poses) {
  poseButtons.innerHTML = "";
  Object.keys(poses).forEach((poseName) => {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = poseName;
    button.addEventListener("click", async () => {
      const duration = Number(presetDuration.value || 1);
      setMotionTarget(poses[poseName], duration);
      updateArmKinematics(poses[poseName]);
      updateCollisionState();

      if (collisionState.active) {
        recordCommand(`${poseName}-blocked`, poses[poseName], duration);
        updateStatus({
          type: "blocked-by-collision-check",
          duration,
          target: poses[poseName],
        });
        return;
      }

      if (!collisionState.active) {
        try {
          await fetch("/api/pose", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ pose: poseName, duration, source: "vite-ui" }),
          });
        } catch (_error) {
          backendConnected = false;
        }
      }
      recordCommand(poseName, poses[poseName], duration);
      await fetchStatus();
    });
    poseButtons.appendChild(button);
  });
}

function renderSequenceButtons() {
  sequenceButtons.innerHTML = "";
  const sequences = {
    "Deploy Cycle": [
      currentPredefinedPoses.home,
      currentPredefinedPoses.ready,
      currentPredefinedPoses.forward_low,
      currentPredefinedPoses.stretch_up,
      currentPredefinedPoses.home,
    ],
    "Inspection Sweep": [
      currentPredefinedPoses.ready,
      [0.6, -1.0, 0.0, -0.7, 0.0, 0.5, 0.0],
      [-0.6, -1.0, 0.0, -0.7, 0.0, -0.5, 0.0],
      currentPredefinedPoses.ready,
    ],
    "Fold And Extend": [
      currentPredefinedPoses.home,
      currentPredefinedPoses.forward_low,
      currentPredefinedPoses.home,
    ],
  };
  Object.entries(sequences).forEach(([label, sequence]) => {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = label;
    button.addEventListener("click", async () => {
      const duration = Number(directDuration.value || 1);
      for (const positions of sequence) {
        setJointValues(positions);
        setMotionTarget(positions, duration);
        updateArmKinematics(positions);
        updateCollisionState();
        if (collisionState.active) {
          recordCommand(`${label}-blocked`, positions, duration);
          updateStatus({
            type: "blocked-by-collision-check",
            duration,
            target: positions,
          });
          return;
        }
        try {
          await fetch("/api/joints", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ positions, duration, label, source: "vite-ui" }),
          });
        } catch (_error) {
          backendConnected = false;
        }
        recordCommand(label, positions, duration);
        await new Promise((resolve) => window.setTimeout(resolve, duration * 1000));
      }
      await fetchStatus();
    });
    sequenceButtons.appendChild(button);
  });
}

function updateStatus(lastCommand) {
  const positions = getDisplayedPositions();
  const backendStateMode = hasFreshBackendState() ? "live joint state" : "bridge only / local motion";

  if (hasFreshBackendState()) {
    setSyncStatus("sync-ok", "Live robot state available");
  } else if (backendConnected) {
    setSyncStatus("sync-warn", "Bridge connected but no live joint state");
  } else {
    setSyncStatus("sync-error", "Bridge unavailable");
  }

  statusOutput.textContent = [
    `Backend: ${backendConnected ? "connected" : "offline demo mode"}`,
    `Backend state mode: ${backendStateMode}`,
    `Rapier collision state: ${collisionState.active ? collisionState.messages.join(" | ") : "clear"}`,
    `Joint positions: ${formatPositions(positions)}`,
    `Last command type: ${lastCommand.type}`,
    `Last command duration: ${lastCommand.duration ?? "n/a"}`,
    `Last command target: ${
      Array.isArray(lastCommand.target) ? formatPositions(lastCommand.target) : "n/a"
    }`,
  ].join("\n");
}

function setSceneItemPresence(item, present) {
  sceneItems[item] = present;
  if (item === "arm" && armRoot) {
    armRoot.visible = present;
  }
  if (item === "table" && tableMesh) {
    tableMesh.visible = present;
  }
  if (item === "box" && boxMesh) {
    boxMesh.visible = present;
  }
  renderItemStatus();
}

async function fetchStatus() {
  try {
    const response = await fetch("/api/status");
    if (!response.ok) {
      throw new Error(`status ${response.status}`);
    }
    const status = await response.json();
    backendConnected = true;
    lastStatus = status;
    if (Array.isArray(status.command_history)) {
      commandHistory = status.command_history.map((entry) => ({
        ...entry,
        stamp: (entry.stamp || 0) * 1000,
      }));
      renderCommandHistory();
    }
    if (status.predefined_poses) {
      const nextSignature = JSON.stringify(status.predefined_poses);
      if (nextSignature !== currentPresetSignature) {
        currentPredefinedPoses = status.predefined_poses;
        currentPresetSignature = nextSignature;
        renderPoseButtons(currentPredefinedPoses);
        renderSequenceButtons();
      }
    }
    if (!jointForm.children.length) {
      renderJointInputs(status.joint_state.names);
    }
    if (!poseButtons.children.length) {
      renderPoseButtons(currentPredefinedPoses);
    }
    if (!sequenceButtons.children.length) {
      renderSequenceButtons();
    }
    updateStatus(status.last_command);
  } catch (_error) {
    backendConnected = false;
    if (!jointForm.children.length) {
      renderJointInputs(JOINT_NAMES);
    }
    if (!poseButtons.children.length) {
      renderPoseButtons(currentPredefinedPoses);
    }
    if (!sequenceButtons.children.length) {
      renderSequenceButtons();
    }
    updateStatus({
      type: "offline-demo",
      duration: activeMotion?.duration ?? null,
      target: targetJointPositions,
    });
  }
}

function setMotionTarget(positions, durationSec) {
  targetJointPositions = positions.map((value) => normalizeAngle(value));
  activeMotion = {
    start: localJointPositions.map((value) => normalizeAngle(value)),
    target: [...targetJointPositions],
    startedAt: performance.now(),
    duration: Math.max(0.2, Number(durationSec)) * 1000,
  };
}

function onLiveJointInput(event) {
  const index = Number(event.target.dataset.index);
  const role = event.target.dataset.role;
  const value = Number(event.target.value);
  const peerRole = role === "slider" ? "number" : "slider";
  const peer = jointForm.querySelector(`input[data-role="${peerRole}"][data-index="${index}"]`);
  if (peer) {
    peer.value = String(value);
  }

  const positions = getJointValues();
  const duration = Number(customDuration.value || 1);
  setMotionTarget(positions, duration);
  updateArmKinematics(positions);
  updateCollisionState();
  updateStatus({
    type: "live-joint-edit",
    duration,
    target: positions,
  });
}

function advanceOfflineMotion(now) {
  if (hasFreshBackendState()) {
    localJointPositions = [...lastStatus.joint_state.positions];
    return;
  }

  if (!activeMotion) {
    return;
  }

  const progress = clamp((now - activeMotion.startedAt) / activeMotion.duration, 0, 1);
  const eased = 1 - Math.pow(1 - progress, 3);
  localJointPositions = activeMotion.start.map((start, index) => {
    const delta = shortestAngleDelta(start, activeMotion.target[index]);
    return normalizeAngle(start + delta * eased);
  });

  if (progress >= 1) {
    activeMotion = null;
  }
}

function resizeRenderer() {
  if (useCanvasFallback) {
    const bounds = viewerCanvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const width = Math.max(1, Math.floor(bounds.width * dpr));
    const height = Math.max(1, Math.floor(bounds.height * dpr));
    if (viewerCanvas.width !== width || viewerCanvas.height !== height) {
      viewerCanvas.width = width;
      viewerCanvas.height = height;
    }
    return;
  }

  if (!renderer || !camera) {
    return;
  }
  const bounds = viewerCanvas.getBoundingClientRect();
  const width = Math.max(1, Math.floor(bounds.width));
  const height = Math.max(1, Math.floor(bounds.height));
  if (viewerCanvas.width !== width || viewerCanvas.height !== height) {
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  }
}

function updateArmKinematics(positions) {
  if (!armRoot) {
    return;
  }
  linkStates.forEach((state, index) => {
    state.pivot.rotation.set(0, 0, 0);
    if (state.axis === "x") {
      state.pivot.rotation.x = positions[index] || 0;
    } else {
      state.pivot.rotation.z = positions[index] || 0;
    }
  });

  armRoot.updateMatrixWorld(true);

  linkStates.forEach((state, index) => {
    state.worldStart.setFromMatrixPosition(state.pivot.matrixWorld);
    state.worldEnd.copy(state.direction).applyMatrix4(state.pivot.matrixWorld);

    const center = state.worldStart.clone().lerp(state.worldEnd, 0.5);
    const quaternion = new THREE.Quaternion().setFromRotationMatrix(state.mesh.matrixWorld);

    rigidBodies[index].setNextKinematicTranslation(center);
    rigidBodies[index].setNextKinematicRotation({
      x: quaternion.x,
      y: quaternion.y,
      z: quaternion.z,
      w: quaternion.w,
    });
  });
}

function updateCollisionState() {
  if (!world) {
    return;
  }
  world.step();
  const messages = [];

  colliderHandles.forEach((handle, index) => {
    const touchesFloor = world.intersectionPair(handle, floorCollider.handle);
    const touchesTable = world.intersectionPair(handle, tableCollider.handle);

    if (touchesFloor) {
      messages.push(`Link ${index + 1} intersects the floor`);
    }
    if (touchesTable) {
      messages.push(`Link ${index + 1} intersects the table`);
    }
  });

  for (let i = 0; i < colliderHandles.length; i += 1) {
    for (let j = i + 2; j < colliderHandles.length; j += 1) {
      if (world.intersectionPair(colliderHandles[i], colliderHandles[j])) {
        messages.push(`Links ${i + 1} and ${j + 1} overlap`);
      }
    }
  }

  collisionState = {
    active: messages.length > 0,
    messages: [...new Set(messages)],
  };

  linkStates.forEach((state) => {
    state.mesh.material.color.setHex(collisionState.active ? 0xff7a59 : state.color);
  });
}

function ensureViewerNotice() {
  if (viewerNotice) {
    return viewerNotice;
  }
  viewerNotice = document.createElement("p");
  viewerNotice.className = "subtle";
  viewerNotice.style.marginBottom = "12px";
  viewerPanel.insertBefore(viewerNotice, viewerCanvas);
  return viewerNotice;
}

function drawFallbackArm() {
  const ctx = fallbackContext;
  if (!ctx) {
    return;
  }

  resizeRenderer();
  ctx.clearRect(0, 0, viewerCanvas.width, viewerCanvas.height);

  const positions = getDisplayedPositions();
  const centerX = viewerCanvas.width * 0.5;
  const centerY = viewerCanvas.height * 0.78;
  const scale = Math.min(viewerCanvas.width, viewerCanvas.height) * 0.22;

  ctx.strokeStyle = "rgba(124, 209, 255, 0.18)";
  ctx.lineWidth = 1.5;
  for (let i = -5; i <= 5; i += 1) {
    ctx.beginPath();
    ctx.moveTo(centerX - scale * 1.4, centerY + i * scale * 0.16);
    ctx.lineTo(centerX + scale * 1.4, centerY + i * scale * 0.16);
    ctx.stroke();
  }

  ctx.strokeStyle = "#253746";
  ctx.lineCap = "round";
  ctx.lineWidth = 26;
  ctx.beginPath();
  ctx.moveTo(centerX, centerY + 14);
  ctx.lineTo(centerX, centerY - scale * 0.22);
  ctx.stroke();

  const lengths = [0.55, 0.18, 0.58, 0.2, 0.45, 0.16, 0.22];
  const thickness = [18, 16, 14, 12, 10, 9, 8];
  const colors = ["#d6ff57", "#7cd1ff", "#d6ff57", "#7cd1ff", "#d6ff57", "#7cd1ff", "#d6ff57"];
  let angle = -Math.PI / 2;
  let x = centerX;
  let y = centerY - scale * 0.22;

  positions.forEach((joint, index) => {
    angle += joint;
    const nx = x + Math.cos(angle) * scale * lengths[index];
    const ny = y + Math.sin(angle) * scale * lengths[index];
    ctx.strokeStyle = collisionState.active ? "#ff7a59" : colors[index];
    ctx.lineWidth = thickness[index];
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(nx, ny);
    ctx.stroke();

    ctx.fillStyle = "#f5fbff";
    ctx.beginPath();
    ctx.arc(x, y, 4.5, 0, Math.PI * 2);
    ctx.fill();

    x = nx;
    y = ny;
  });

  ctx.fillStyle = collisionState.active ? "#ff7a59" : "#f5fbff";
  ctx.beginPath();
  ctx.arc(x, y, 5, 0, Math.PI * 2);
  ctx.fill();
}

function animate(now) {
  resizeRenderer();
  advanceOfflineMotion(now);
  if (useCanvasFallback) {
    drawFallbackArm();
    requestAnimationFrame(animate);
    return;
  }

  updateArmKinematics(getDisplayedPositions());
  updateCollisionState();
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}

sendCustomButton.addEventListener("click", async () => {
  const positions = getJointValues().map((value) => normalizeAngle(value));
  const duration = Number(customDuration.value || 1);
  setMotionTarget(positions, duration);
  updateArmKinematics(positions);
  updateCollisionState();

  if (collisionState.active) {
    recordCommand("custom-blocked", positions, duration);
    updateStatus({
      type: "blocked-by-collision-check",
      duration,
      target: positions,
    });
    return;
  }

  try {
    await fetch("/api/joints", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ positions, duration, label: "custom", source: "vite-ui" }),
    });
  } catch (_error) {
    backendConnected = false;
  }
  recordCommand("custom", positions, duration);
  await fetchStatus();
});

sendDirectButton.addEventListener("click", async () => {
  const duration = Number(directDuration.value || 1);
  const commandSets = directCommandInput.value
    .split(";")
    .map((chunk) => chunk.trim())
    .filter(Boolean);

  if (!commandSets.length) {
    updateStatus({
      type: "invalid-direct-command",
      duration: null,
      target: null,
    });
    return;
  }

  for (const commandSet of commandSets) {
    const values = commandSet
      .split(",")
      .map((part) => part.trim())
      .filter(Boolean);

    if (values.length !== 7) {
      updateStatus({
        type: "invalid-direct-command",
        duration: null,
        target: null,
      });
      return;
    }

    const positions = values.map((value) => degreesToRadians(value));
    setJointValues(positions);
    setMotionTarget(positions, duration);
    updateArmKinematics(positions);
    updateCollisionState();

    if (collisionState.active) {
      recordCommand("direct-blocked", positions, duration);
      updateStatus({
        type: "blocked-by-collision-check",
        duration,
        target: positions,
      });
      return;
    }

    try {
      await fetch("/api/joints", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ positions, duration, label: "direct", source: "vite-ui" }),
      });
    } catch (_error) {
      backendConnected = false;
    }

    recordCommand("direct", positions, duration);
    await new Promise((resolve) => window.setTimeout(resolve, duration * 1000));
  }

  await fetchStatus();
});

copyCurrentButton.addEventListener("click", () => {
  setJointValues(getDisplayedPositions());
});

refreshButton.addEventListener("click", fetchStatus);

syncRobotButton.addEventListener("click", () => {
  if (hasFreshBackendState()) {
    const positions = [...lastStatus.joint_state.positions];
    localJointPositions = [...positions];
    targetJointPositions = [...positions];
    activeMotion = null;
    setJointValues(positions);
    updateArmKinematics(positions);
    updateCollisionState();
    setSyncStatus("sync-ok", "Synced from live robot state");
    updateStatus({
      type: "synced-from-robot",
      duration: null,
      target: positions,
    });
    return;
  }

  if (backendConnected) {
    setSyncStatus("sync-warn", "Cannot sync: no live joint state. Start Gazebo/controllers.");
  } else {
    setSyncStatus("sync-error", "Cannot sync: ROS bridge is unavailable.");
  }
});

resetCameraButton.addEventListener("click", () => {
  if (!camera || !controls) {
    return;
  }
  camera.position.set(2.1, 1.8, 1.45);
  controls.target.set(0, 0, 0.45);
  controls.update();
});

clearCommandsButton.addEventListener("click", () => {
  commandHistory = [];
  renderCommandHistory();
});

window.addEventListener("resize", resizeRenderer);

function initializeScene() {
  const gl =
    viewerCanvas.getContext("webgl", {
      alpha: true,
      antialias: false,
      depth: true,
      stencil: false,
      powerPreference: "low-power",
      premultipliedAlpha: false,
    }) ||
    viewerCanvas.getContext("experimental-webgl", {
      alpha: true,
      antialias: false,
      depth: true,
      stencil: false,
      powerPreference: "low-power",
      premultipliedAlpha: false,
    });

  if (!gl) {
    throw new Error("WebGL context unavailable");
  }

  webglMode = "webgl1";
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0f1419);

  renderer = new THREE.WebGLRenderer({
    canvas: viewerCanvas,
    context: gl,
    antialias: false,
    alpha: true,
    powerPreference: "low-power",
  });
  renderer.setPixelRatio(1);
  renderer.shadowMap.enabled = false;

  camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
  camera.up.set(0, 0, 1);
  camera.position.set(2.1, 1.8, 1.45);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.45);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 0.6;
  controls.maxDistance = 5;
  controls.maxPolarAngle = Math.PI * 0.48;

  scene.add(new THREE.HemisphereLight(0xf5fbff, 0x0b1014, 1.0));

  const keyLight = new THREE.DirectionalLight(0xffffff, 1.45);
  keyLight.position.set(2.6, 1.8, 3.4);
  scene.add(keyLight);

  const fillLight = new THREE.PointLight(0x7cd1ff, 20, 8);
  fillLight.position.set(-1.6, -1.6, 1.8);
  scene.add(fillLight);

  const grid = new THREE.GridHelper(2.4, 16, 0x7cd1ff, 0x26404f);
  grid.rotation.x = Math.PI / 2;
  grid.material.opacity = 0.3;
  grid.material.transparent = true;
  scene.add(grid);

  const floor = new THREE.Mesh(
    new THREE.CircleGeometry(1.7, 64),
    new THREE.MeshLambertMaterial({
      color: 0x121b23,
    })
  );
  scene.add(floor);

  const table = new THREE.Mesh(
    new THREE.BoxGeometry(ENVIRONMENT.tableSize.x, ENVIRONMENT.tableSize.y, ENVIRONMENT.tableSize.z),
    new THREE.MeshLambertMaterial({
      color: 0x6a3f22,
    })
  );
  table.position.copy(ENVIRONMENT.tableCenter);
  scene.add(table);
  tableMesh = table;
  tableMesh.visible = sceneItems.table;

  boxMesh = new THREE.Mesh(
    new THREE.BoxGeometry(0.12, 0.12, 0.12),
    new THREE.MeshLambertMaterial({ color: 0xffa45d })
  );
  boxMesh.position.set(0.38, 0, ENVIRONMENT.tableCenter.z + ENVIRONMENT.tableSize.z / 2 + 0.06);
  scene.add(boxMesh);
  boxMesh.visible = sceneItems.box;

  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.08, 0.08, 0.05, 36),
    new THREE.MeshLambertMaterial({
      color: 0x253746,
    })
  );
  base.position.z = 0.025;
  scene.add(base);

  scene.add(new THREE.AxesHelper(0.22));

  armRoot = new THREE.Group();
  scene.add(armRoot);

  linkStates = [];
  let parentGroup = armRoot;
  ARM_CONFIG.chain.forEach((joint) => {
    const pivot = new THREE.Group();
    pivot.position.set(...joint.offset);
    parentGroup.add(pivot);

    const direction = new THREE.Vector3(...joint.drawTo);
    const length = direction.length();
    const geometry = new THREE.CapsuleGeometry(
      joint.radius,
      Math.max(0.001, length - joint.radius * 2),
      10,
      20
    );
    const material = new THREE.MeshLambertMaterial({
      color: joint.color,
    });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.copy(direction.clone().multiplyScalar(0.5));
    mesh.quaternion.copy(
      new THREE.Quaternion().setFromUnitVectors(
        new THREE.Vector3(0, 1, 0),
        direction.clone().normalize()
      )
    );
    pivot.add(mesh);

    linkStates.push({
      axis: joint.axis,
      pivot,
      mesh,
      direction,
      length,
      color: joint.color,
      radius: joint.radius,
      worldStart: new THREE.Vector3(),
      worldEnd: new THREE.Vector3(),
    });

    parentGroup = pivot;
  });
}

function initializePhysics() {
  world = new RAPIER.World({ x: 0, y: 0, z: 0 });
  const fixedFloor = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  floorCollider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(1.8, 1.8, 0.02).setTranslation(0, 0, -0.02),
    fixedFloor
  );

  const fixedTable = world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
  tableCollider = world.createCollider(
    RAPIER.ColliderDesc.cuboid(
      ENVIRONMENT.tableSize.x / 2,
      ENVIRONMENT.tableSize.y / 2,
      ENVIRONMENT.tableSize.z / 2
    ).setTranslation(
      ENVIRONMENT.tableCenter.x,
      ENVIRONMENT.tableCenter.y,
      ENVIRONMENT.tableCenter.z
    ),
    fixedTable
  );

  colliderHandles = [];
  rigidBodies = [];
  linkStates.forEach((state) => {
    const rigidBody = world.createRigidBody(RAPIER.RigidBodyDesc.kinematicPositionBased());
    const halfBody = Math.max(0.01, state.length / 2 - state.radius);
    const collider = world.createCollider(RAPIER.ColliderDesc.capsule(halfBody, state.radius), rigidBody);
    rigidBodies.push(rigidBody);
    colliderHandles.push(collider.handle);
  });
}

async function bootstrap() {
  await RAPIER.init();
  try {
    initializeScene();
    initializePhysics();
    ensureViewerNotice().textContent = `Three.js + Rapier mode (${webglMode})`;
  } catch (error) {
    useCanvasFallback = true;
    fallbackContext = viewerCanvas.getContext("2d");
    ensureViewerNotice().textContent =
      "WebGL renderer unavailable on this machine. Using simplified canvas fallback.";
    console.error(error);
  }
  renderJointInputs(JOINT_NAMES);
  renderPoseButtons(currentPredefinedPoses);
  renderSequenceButtons();
  renderCommandHistory();
  renderItemStatus();
  fetchStatus();
  setInterval(fetchStatus, 1500);
  requestAnimationFrame(animate);
}

bootstrap();

placeItemButton.addEventListener("click", () => {
  setSceneItemPresence(sceneItemSelect.value, true);
});

removeItemButton.addEventListener("click", () => {
  setSceneItemPresence(sceneItemSelect.value, false);
});
