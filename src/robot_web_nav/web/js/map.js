// Map rendering: pixel-processed map, grid overlay, smooth robot icon, path visualization, initial pose.

let mapCanvas, mapCtx;
let mapImg = null;            // original PGM-derived image (raw grayscale)
let processedMapCanvas = null; // pre-rendered colored map (offscreen)
let viewOffset = { x: 0, y: 0 };
let viewScale = 1;
let isDragging = false;
let dragStart = { x: 0, y: 0 };
let dragOffsetStart = { x: 0, y: 0 };
let lastPinchDist = 0;
let lastPinchCenter = { x: 0, y: 0 };
let touchStart = null;
let touchMode = 'none';
let lastTap = null;

const TAP_MAX_MS = 300;
const TAP_MOVE_PX = 14;
const DOUBLE_TAP_MS = 380;
const DOUBLE_TAP_PX = 32;

// Robot pose smoothing
let smoothPose = null;       // {x, y, yaw} in world coords

// Planned path from Nav2
state.plannedPath = null;

// Initial pose setting mode
let setPoseMode = false;
let setPoseStart = null;     // {sx, sy} screen coords for drag-to-yaw
let setPoseCurrent = null;   // current drag position

window.addEventListener('load', () => {
  mapCanvas = document.getElementById('map-canvas');
  mapCtx = mapCanvas.getContext('2d');
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  mapCanvas.addEventListener('mousedown', onPointerDown);
  mapCanvas.addEventListener('mousemove', onPointerMove);
  mapCanvas.addEventListener('mouseup', onPointerUp);
  mapCanvas.addEventListener('mouseleave', onPointerUp);
  mapCanvas.addEventListener('wheel', onWheel, { passive: false });

  mapCanvas.addEventListener('touchstart', onTouchStart, { passive: false });
  mapCanvas.addEventListener('touchmove', onTouchMove, { passive: false });
  mapCanvas.addEventListener('touchend', onTouchEnd, { passive: false });
  mapCanvas.addEventListener('touchcancel', onTouchCancel, { passive: false });

  mapCanvas.addEventListener('dblclick', onMapDblClick);

  document.querySelectorAll('.zoom-btn').forEach(btn => {
    btn.addEventListener('click', () => {
      const z = btn.getAttribute('data-zoom');
      if (z === 'fit') fitMap();
      else zoomAtCenter(z === 'in' ? 1.25 : 0.8);
    });
  });

  // Set initial pose button
  document.getElementById('btn-set-pose').addEventListener('click', () => {
    setPoseMode = !setPoseMode;
    document.getElementById('set-pose-hint').classList.toggle('hidden', !setPoseMode);
    mapCanvas.style.cursor = setPoseMode ? 'cell' : 'crosshair';
  });

  document.getElementById('btn-cancel-set-pose').addEventListener('click', () => {
    setPoseMode = false;
    setPoseStart = null;
    setPoseCurrent = null;
    document.getElementById('set-pose-hint').classList.add('hidden');
    mapCanvas.style.cursor = 'crosshair';
  });

  // Smooth animation loop
  requestAnimationFrame(animLoop);
});

function resizeCanvas() {
  const container = document.getElementById('map-container');
  const dpr = window.devicePixelRatio || 1;
  mapCanvas.width = container.clientWidth * dpr;
  mapCanvas.height = container.clientHeight * dpr;
  mapCanvas.style.width = container.clientWidth + 'px';
  mapCanvas.style.height = container.clientHeight + 'px';
  mapCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
  renderMap();
}

// ===== Handler registration =====

registerHandler('map_data', (msg) => {
  state.liveMapActive = false;
  state.liveMapStats = null;
  if (typeof updateLiveMapStats === 'function') updateLiveMapStats();
  state.currentMap = msg.map_name;
  hideMapLoading();
  state.mapMeta = {
    width: msg.width,
    height: msg.height,
    resolution: msg.resolution,
    origin: msg.origin,
    mapName: msg.map_name,
    displayName: msg.display_name || msg.map_name,
  };

  const img = new Image();
  img.onload = () => {
    mapImg = img;
    processedMapCanvas = preprocessMap(img, msg.width, msg.height);
    fitMap();
    if (state.mapMeta.displayName) {
      const dn = document.getElementById('map-display-name');
      if (dn) dn.textContent = `显示名: ${state.mapMeta.displayName}`;
    }
  };
  img.src = 'data:image/png;base64,' + msg.image;
});

registerHandler('live_map_data', (msg) => {
  hideMapLoading();
  const prevMeta = state.mapMeta;
  const shouldFit = !state.liveMapActive
    || !prevMeta
    || prevMeta.width !== msg.width
    || prevMeta.height !== msg.height
    || prevMeta.resolution !== msg.resolution
    || prevMeta.origin[0] !== msg.origin[0]
    || prevMeta.origin[1] !== msg.origin[1];

  state.liveMapActive = true;
  state.liveMapStats = {
    knownPercent: msg.known_percent,
    freeCells: msg.free_cells,
    occupiedCells: msg.occupied_cells,
    stamp: msg.stamp,
  };
  state.mapMeta = {
    width: msg.width,
    height: msg.height,
    resolution: msg.resolution,
    origin: msg.origin,
    mapName: msg.map_name,
    displayName: msg.display_name || 'Live mapping',
  };

  const img = new Image();
  img.onload = () => {
    mapImg = img;
    processedMapCanvas = preprocessMap(img, msg.width, msg.height);
    if (shouldFit) fitMap();
    else renderMap();
    if (typeof updateLiveMapStats === 'function') updateLiveMapStats();
  };
  img.src = 'data:image/png;base64,' + msg.image;
});

registerHandler('live_map_cleared', () => {
  state.liveMapActive = false;
  state.liveMapStats = null;
  if (typeof updateLiveMapStats === 'function') updateLiveMapStats();
});

registerHandler('planned_path', (msg) => {
  state.plannedPath = msg.path || null;
  renderMap();
});

registerHandler('initial_pose_set', () => {
  setPoseMode = false;
  setPoseStart = null;
  setPoseCurrent = null;
  document.getElementById('set-pose-hint').classList.add('hidden');
  mapCanvas.style.cursor = 'crosshair';
});

// ===== Map processing =====

function preprocessMap(img, w, h) {
  const off = document.createElement('canvas');
  off.width = w;
  off.height = h;
  const c = off.getContext('2d');
  c.drawImage(img, 0, 0, w, h);
  const data = c.getImageData(0, 0, w, h);
  const px = data.data;
  for (let i = 0; i < px.length; i += 4) {
    const v = px[i];
    let r, g, b, a;
    if (v < 50) {
      r = 11; g = 13; b = 11; a = 255;
    } else if (v > 220) {
      r = 49; g = 64; b = 53; a = 255;
    } else {
      r = 23; g = 28; b = 23; a = 180;
    }
    px[i] = r; px[i+1] = g; px[i+2] = b; px[i+3] = a;
  }
  c.putImageData(data, 0, 0);
  return off;
}

function fitMap() {
  if (!mapImg || !state.mapMeta) return;
  const cw = mapCanvas.clientWidth;
  const ch = mapCanvas.clientHeight;
  viewScale = Math.min(cw / state.mapMeta.width, ch / state.mapMeta.height) * 0.9;
  viewOffset.x = (cw - state.mapMeta.width * viewScale) / 2;
  viewOffset.y = (ch - state.mapMeta.height * viewScale) / 2;
  renderMap();
}

function zoomAtCenter(factor) {
  const cw = mapCanvas.clientWidth;
  const ch = mapCanvas.clientHeight;
  const cx = cw / 2, cy = ch / 2;
  const newScale = Math.max(0.1, Math.min(20, viewScale * factor));
  viewOffset.x = cx - (cx - viewOffset.x) * (newScale / viewScale);
  viewOffset.y = cy - (cy - viewOffset.y) * (newScale / viewScale);
  viewScale = newScale;
  renderMap();
}

function renderMap() {
  if (!mapCtx) return;
  const cw = mapCanvas.clientWidth;
  const ch = mapCanvas.clientHeight;
  mapCtx.clearRect(0, 0, cw, ch);
  mapCtx.fillStyle = '#131713';
  mapCtx.fillRect(0, 0, cw, ch);

  if (!state.mapMeta || !processedMapCanvas) return;

  mapCtx.save();
  mapCtx.translate(viewOffset.x, viewOffset.y);
  mapCtx.scale(viewScale, viewScale);
  mapCtx.imageSmoothingEnabled = false;
  mapCtx.drawImage(processedMapCanvas, 0, 0);
  drawGrid();
  mapCtx.restore();

  drawPlannedPath();
  drawWaypoints();
  if (state.robotPose) drawRobot();
  drawPendingClick();
  drawSetPosePreview();
}

function drawGrid() {
  if (!state.mapMeta) return;
  const metersPerPx = state.mapMeta.resolution;
  const pxPerMeter = 1 / metersPerPx;
  if (pxPerMeter * viewScale < 30) return;
  const w = state.mapMeta.width;
  const h = state.mapMeta.height;
  mapCtx.strokeStyle = 'rgba(255,255,255,0.06)';
  mapCtx.lineWidth = 1 / viewScale;
  mapCtx.beginPath();
  for (let x = 0; x <= w; x += pxPerMeter) {
    mapCtx.moveTo(x, 0);
    mapCtx.lineTo(x, h);
  }
  for (let y = 0; y <= h; y += pxPerMeter) {
    mapCtx.moveTo(0, y);
    mapCtx.lineTo(w, y);
  }
  mapCtx.stroke();
}

function drawPlannedPath() {
  if (!state.plannedPath || state.plannedPath.length < 2 || !state.mapMeta) return;
  mapCtx.save();
  mapCtx.strokeStyle = '#6ee7c8';
  mapCtx.lineWidth = 2;
  mapCtx.setLineDash([6, 4]);
  mapCtx.globalAlpha = 0.7;
  mapCtx.beginPath();
  const [sx0, sy0] = worldToScreen(state.plannedPath[0][0], state.plannedPath[0][1]);
  mapCtx.moveTo(sx0, sy0);
  for (let i = 1; i < state.plannedPath.length; i++) {
    const [sx, sy] = worldToScreen(state.plannedPath[i][0], state.plannedPath[i][1]);
    mapCtx.lineTo(sx, sy);
  }
  mapCtx.stroke();
  mapCtx.restore();
}

function drawWaypoints() {
  if (!state.mapMeta) return;
  state.waypoints.forEach(wp => {
    const [sx, sy] = worldToScreen(wp.x, wp.y);
    mapCtx.save();
    mapCtx.beginPath();
    mapCtx.arc(sx, sy, 14, 0, Math.PI * 2);
    mapCtx.fillStyle = 'rgba(52,179,143,0.16)';
    mapCtx.fill();
    mapCtx.beginPath();
    mapCtx.arc(sx, sy, 8, 0, Math.PI * 2);
    mapCtx.fillStyle = '#34b38f';
    mapCtx.fill();
    mapCtx.strokeStyle = '#fff';
    mapCtx.lineWidth = 2;
    mapCtx.stroke();
    mapCtx.fillStyle = '#fff';
    mapCtx.font = '500 12px -apple-system, sans-serif';
    mapCtx.textAlign = 'center';
    mapCtx.fillText(wp.name, sx, sy - 14);
    mapCtx.restore();
  });
}

function drawRobot() {
  if (!smoothPose || !state.mapMeta) return;
  const [sx, sy] = worldToScreen(smoothPose.x, smoothPose.y);
  const footprintPx = (0.25 / state.mapMeta.resolution) * viewScale;
  mapCtx.save();
  mapCtx.translate(sx, sy);
  mapCtx.rotate(-smoothPose.yaw);
  mapCtx.beginPath();
  mapCtx.arc(0, 0, footprintPx, 0, Math.PI * 2);
  mapCtx.fillStyle = 'rgba(74,255,138,0.12)';
  mapCtx.fill();
  const r = 10;
  mapCtx.beginPath();
  mapCtx.arc(0, 0, r, 0, Math.PI * 2);
  mapCtx.fillStyle = 'rgba(74,255,138,0.4)';
  mapCtx.fill();
  mapCtx.strokeStyle = '#4aff8a';
  mapCtx.lineWidth = 2;
  mapCtx.stroke();
  mapCtx.beginPath();
  mapCtx.moveTo(r * 1.4, 0);
  mapCtx.lineTo(-r * 0.4, -r * 0.7);
  mapCtx.lineTo(-r * 0.4, r * 0.7);
  mapCtx.closePath();
  mapCtx.fillStyle = '#4aff8a';
  mapCtx.fill();
  mapCtx.restore();
}

function drawPendingClick() {
  if (!state.pendingClick) return;
  const [sx, sy] = pixelToScreen(state.pendingClick.px, state.pendingClick.py);
  mapCtx.save();
  mapCtx.beginPath();
  mapCtx.arc(sx, sy, 12, 0, Math.PI * 2);
  mapCtx.strokeStyle = '#ffb347';
  mapCtx.lineWidth = 2;
  mapCtx.setLineDash([4, 4]);
  mapCtx.stroke();
  mapCtx.restore();
}

function drawSetPosePreview() {
  if (!setPoseMode || !setPoseStart || !state.mapMeta) return;
  const [sx, sy] = [setPoseStart.sx, setPoseStart.sy];
  mapCtx.save();
  // Draw position dot
  mapCtx.beginPath();
  mapCtx.arc(sx, sy, 8, 0, Math.PI * 2);
  mapCtx.fillStyle = 'rgba(255,179,71,0.6)';
  mapCtx.fill();
  mapCtx.strokeStyle = '#ffb347';
  mapCtx.lineWidth = 2;
  mapCtx.stroke();

  // Draw heading arrow if dragging
  if (setPoseCurrent) {
    const dx = setPoseCurrent.sx - sx;
    const dy = setPoseCurrent.sy - sy;
    const angle = Math.atan2(-dy, dx); // screen Y is inverted
    const arrowLen = 30;
    mapCtx.beginPath();
    mapCtx.moveTo(sx, sy);
    mapCtx.lineTo(sx + Math.cos(angle) * arrowLen, sy - Math.sin(angle) * arrowLen);
    mapCtx.strokeStyle = '#ffb347';
    mapCtx.lineWidth = 3;
    mapCtx.setLineDash([]);
    mapCtx.stroke();
    // Arrowhead
    const tipX = sx + Math.cos(angle) * arrowLen;
    const tipY = sy - Math.sin(angle) * arrowLen;
    mapCtx.beginPath();
    mapCtx.arc(tipX, tipY, 4, 0, Math.PI * 2);
    mapCtx.fillStyle = '#ffb347';
    mapCtx.fill();
  }
  mapCtx.restore();
}

// Smooth interpolation loop (~60fps)
function animLoop() {
  if (state.robotPose) {
    if (!smoothPose) {
      smoothPose = { x: state.robotPose.x, y: state.robotPose.y, yaw: state.robotPose.yaw };
    } else {
      const k = 0.25;
      smoothPose.x += (state.robotPose.x - smoothPose.x) * k;
      smoothPose.y += (state.robotPose.y - smoothPose.y) * k;
      let dy = state.robotPose.yaw - smoothPose.yaw;
      while (dy > Math.PI) dy -= 2 * Math.PI;
      while (dy < -Math.PI) dy += 2 * Math.PI;
      smoothPose.yaw += dy * k;
    }
    renderMap();
  }
  requestAnimationFrame(animLoop);
}

// ===== Coordinate conversions =====
function worldToScreen(wx, wy) {
  const m = state.mapMeta;
  const px = (wx - m.origin[0]) / m.resolution;
  const py = m.height - (wy - m.origin[1]) / m.resolution;
  return [px * viewScale + viewOffset.x, py * viewScale + viewOffset.y];
}

function screenToWorld(sx, sy) {
  const m = state.mapMeta;
  const px = (sx - viewOffset.x) / viewScale;
  const py = (sy - viewOffset.y) / viewScale;
  return [m.origin[0] + px * m.resolution, m.origin[1] + (m.height - py) * m.resolution];
}

function screenToPixel(sx, sy) {
  return [Math.round((sx - viewOffset.x) / viewScale),
          Math.round((sy - viewOffset.y) / viewScale)];
}

function pixelToScreen(px, py) {
  return [px * viewScale + viewOffset.x, py * viewScale + viewOffset.y];
}

function isPixelInsideMap(px, py) {
  return state.mapMeta
    && px >= 0
    && py >= 0
    && px < state.mapMeta.width
    && py < state.mapMeta.height;
}

// ===== Pointer / touch =====
function getCanvasPos(e) {
  const rect = mapCanvas.getBoundingClientRect();
  return { x: e.clientX - rect.left, y: e.clientY - rect.top };
}

function onPointerDown(e) {
  // Initial pose mode: single click sets position, drag sets yaw
  if (setPoseMode && state.mapMeta) {
    const pos = getCanvasPos(e);
    setPoseStart = { sx: pos.x, sy: pos.y };
    setPoseCurrent = { sx: pos.x, sy: pos.y };
    return;
  }
  isDragging = true;
  dragStart = { x: e.clientX, y: e.clientY };
  dragOffsetStart = { ...viewOffset };
  mapCanvas.style.cursor = 'grabbing';
}

function onPointerMove(e) {
  // Initial pose drag for heading
  if (setPoseMode && setPoseStart) {
    const pos = getCanvasPos(e);
    setPoseCurrent = { sx: pos.x, sy: pos.y };
    renderMap();
    return;
  }
  if (!isDragging) return;
  viewOffset.x = dragOffsetStart.x + (e.clientX - dragStart.x);
  viewOffset.y = dragOffsetStart.y + (e.clientY - dragStart.y);
  renderMap();
}

function onPointerUp(e) {
  // Initial pose: release to confirm
  if (setPoseMode && setPoseStart) {
    const pos = getCanvasPos(e);
    finishSetPoseAtScreen(pos.x, pos.y);
    return;
  }
  isDragging = false;
  mapCanvas.style.cursor = setPoseMode ? 'cell' : 'crosshair';
}

function onWheel(e) {
  e.preventDefault();
  const pos = getCanvasPos(e);
  const factor = e.deltaY < 0 ? 1.1 : 0.9;
  const newScale = Math.max(0.1, Math.min(20, viewScale * factor));
  viewOffset.x = pos.x - (pos.x - viewOffset.x) * (newScale / viewScale);
  viewOffset.y = pos.y - (pos.y - viewOffset.y) * (newScale / viewScale);
  viewScale = newScale;
  renderMap();
}

function onMapDblClick(e) {
  e.preventDefault();
  e.stopPropagation();
  const pos = getCanvasPos(e);
  openWaypointOverlayAtScreen(pos.x, pos.y);
}

function finishSetPoseAtScreen(sx, sy) {
  const [px, py] = screenToPixel(setPoseStart.sx, setPoseStart.sy);
  if (!isPixelInsideMap(px, py)) {
    setPoseStart = null;
    setPoseCurrent = null;
    renderMap();
    return;
  }
  const [wx, wy] = screenToWorld(setPoseStart.sx, setPoseStart.sy);
  let yaw = 0;
  if (setPoseCurrent) {
    const dx = sx - setPoseStart.sx;
    const dy = sy - setPoseStart.sy;
    if (Math.sqrt(dx * dx + dy * dy) > 10) {
      yaw = Math.atan2(-dy, dx); // screen Y inverted
    }
  }
  sendMessage({ type: 'set_initial_pose', x: wx, y: wy, yaw: yaw });
  setPoseStart = null;
  setPoseCurrent = null;
}

function openWaypointOverlayAtScreen(sx, sy) {
  if (setPoseMode) return false; // Don't add waypoints in set-pose mode
  if (state.liveMapActive) return false; // Live SLAM preview is not a persisted map yet
  if (!state.mapMeta) return false;
  const [px, py] = screenToPixel(sx, sy);
  if (!isPixelInsideMap(px, py)) return false;
  const [wx, wy] = screenToWorld(sx, sy);
  state.pendingClick = { px, py, wx, wy };
  document.getElementById('overlay-coords').textContent =
    `世界坐标: (${wx.toFixed(2)}, ${wy.toFixed(2)})`;
  const input = document.getElementById('waypoint-name-input');
  input.value = '';
  document.getElementById('map-overlay').classList.remove('hidden');
  setTimeout(() => input.focus(), 100);
  renderMap();
  return true;
}

function onTouchStart(e) {
  e.preventDefault();
  if (e.touches.length === 1) {
    const touch = e.touches[0];
    touchMode = 'single';
    touchStart = { x: touch.clientX, y: touch.clientY, time: Date.now() };
    if (setPoseMode && state.mapMeta) {
      const pos = touchToCanvasPos(touch);
      setPoseStart = { sx: pos.x, sy: pos.y };
      setPoseCurrent = { sx: pos.x, sy: pos.y };
      return;
    }
    isDragging = true;
    dragStart = { x: touch.clientX, y: touch.clientY };
    dragOffsetStart = { ...viewOffset };
  } else if (e.touches.length === 2) {
    touchMode = 'pinch';
    touchStart = null;
    lastTap = null;
    isDragging = false;
    lastPinchDist = pinchDist(e.touches);
    lastPinchCenter = pinchCenter(e.touches);
  }
}

function onTouchMove(e) {
  e.preventDefault();
  if (setPoseMode && setPoseStart && e.touches.length === 1) {
    const pos = touchToCanvasPos(e.touches[0]);
    setPoseCurrent = { sx: pos.x, sy: pos.y };
    renderMap();
    return;
  }
  if (e.touches.length === 1 && isDragging) {
    viewOffset.x = dragOffsetStart.x + (e.touches[0].clientX - dragStart.x);
    viewOffset.y = dragOffsetStart.y + (e.touches[0].clientY - dragStart.y);
    renderMap();
  } else if (e.touches.length === 2) {
    const dist = pinchDist(e.touches);
    const center = pinchCenter(e.touches);
    const rect = mapCanvas.getBoundingClientRect();
    const cx = center.x - rect.left;
    const cy = center.y - rect.top;
    const newScale = Math.max(0.1, Math.min(20, viewScale * (dist / lastPinchDist)));
    viewOffset.x = cx - (cx - viewOffset.x) * (newScale / viewScale);
    viewOffset.y = cy - (cy - viewOffset.y) * (newScale / viewScale);
    viewOffset.x += center.x - lastPinchCenter.x;
    viewOffset.y += center.y - lastPinchCenter.y;
    viewScale = newScale;
    lastPinchDist = dist;
    lastPinchCenter = center;
    renderMap();
  }
}

function onTouchEnd(e) {
  e.preventDefault();
  if (setPoseMode && setPoseStart && e.changedTouches.length > 0) {
    const pos = touchToCanvasPos(e.changedTouches[0]);
    finishSetPoseAtScreen(pos.x, pos.y);
    touchMode = 'none';
    touchStart = null;
    return;
  }

  if (e.touches.length === 0) {
    isDragging = false;
    mapCanvas.style.cursor = setPoseMode ? 'cell' : 'crosshair';

    if (touchMode === 'single' && touchStart && e.changedTouches.length > 0) {
      const touch = e.changedTouches[0];
      const now = Date.now();
      const moved = distance(
        touch.clientX,
        touch.clientY,
        touchStart.x,
        touchStart.y
      );
      if ((now - touchStart.time) <= TAP_MAX_MS && moved <= TAP_MOVE_PX) {
        const pos = touchToCanvasPos(touch);
        if (
          lastTap
          && (now - lastTap.time) <= DOUBLE_TAP_MS
          && distance(pos.x, pos.y, lastTap.x, lastTap.y) <= DOUBLE_TAP_PX
        ) {
          lastTap = null;
          openWaypointOverlayAtScreen(pos.x, pos.y);
        } else {
          lastTap = { x: pos.x, y: pos.y, time: now };
        }
      }
    }
    touchMode = 'none';
    touchStart = null;
  }
}

function onTouchCancel(e) {
  e.preventDefault();
  isDragging = false;
  touchMode = 'none';
  touchStart = null;
  setPoseStart = null;
  setPoseCurrent = null;
  mapCanvas.style.cursor = setPoseMode ? 'cell' : 'crosshair';
  renderMap();
}

function touchToCanvasPos(touch) {
  const rect = mapCanvas.getBoundingClientRect();
  return { x: touch.clientX - rect.left, y: touch.clientY - rect.top };
}

function distance(x1, y1, x2, y2) {
  const dx = x1 - x2;
  const dy = y1 - y2;
  return Math.sqrt(dx * dx + dy * dy);
}

function pinchDist(t) {
  const dx = t[0].clientX - t[1].clientX;
  const dy = t[0].clientY - t[1].clientY;
  return Math.sqrt(dx * dx + dy * dy);
}

function pinchCenter(t) {
  return { x: (t[0].clientX + t[1].clientX) / 2, y: (t[0].clientY + t[1].clientY) / 2 };
}
