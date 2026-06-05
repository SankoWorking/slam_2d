// Manual D-pad control. Claims control, sends set_velocity while buttons are
// held, sends zero on release. Auto-release on disconnect handled server-side.

const ctrl = {
  linearX: 0,
  linearY: 0,
  angularZ: 0,
  owned: false,
  linSpeed: 0.2,
  angSpeed: 0.8,
  pressed: new Set(),
  sendTimer: null,
  pendingKeys: new Set(),
  watchdogTimer: null,
  activePointers: new Map(),
};

const SEND_INTERVAL = 200;
const WATCHDOG_MS = 5000;

window.addEventListener('load', () => {
  // Claim button
  document.getElementById('btn-claim').addEventListener('click', () => {
    if (ctrl.owned) {
      sendMessage({ type: 'release_control' });
    } else {
      sendMessage({ type: 'claim_control' });
    }
  });

  // Speed sliders
  const linSlider = document.getElementById('lin-speed');
  const angSlider = document.getElementById('ang-speed');
  linSlider.addEventListener('input', () => {
    ctrl.linSpeed = parseFloat(linSlider.value);
    document.getElementById('lin-speed-val').textContent = ctrl.linSpeed.toFixed(2);
  });
  angSlider.addEventListener('input', () => {
    ctrl.angSpeed = parseFloat(angSlider.value);
    document.getElementById('ang-speed-val').textContent = ctrl.angSpeed.toFixed(2);
  });

  // D-pad buttons
  document.querySelectorAll('.dpad-btn').forEach(btn => {
    const key = btn.getAttribute('data-key');
    if (window.PointerEvent) {
      btn.addEventListener('pointerdown', (e) => startButtonPress(e, btn, key));
      btn.addEventListener('pointerup', (e) => endButtonPress(e, btn, key));
      btn.addEventListener('pointercancel', (e) => endButtonPress(e, btn, key));
      btn.addEventListener('lostpointercapture', (e) => endButtonPress(e, btn, key));
    } else {
      btn.addEventListener('mousedown', (e) => startButtonPress(e, btn, key));
      btn.addEventListener('mouseup', (e) => endButtonPress(e, btn, key));
      btn.addEventListener('mouseleave', (e) => endButtonPress(e, btn, key));
      btn.addEventListener('touchstart', (e) => startButtonPress(e, btn, key), { passive: false });
      btn.addEventListener('touchend', (e) => endButtonPress(e, btn, key), { passive: false });
      btn.addEventListener('touchcancel', (e) => endButtonPress(e, btn, key), { passive: false });
    }
  });

  // Keyboard fallback (only when control is owned and input not focused)
  document.addEventListener('keydown', (e) => {
    if (document.activeElement && document.activeElement.tagName === 'INPUT') return;
    if (!ctrl.owned) return;
    if (e.repeat) return;
    const map = { 'ArrowUp': 'up', 'ArrowDown': 'down', 'ArrowLeft': 'left', 'ArrowRight': 'right', ' ': 'stop' };
    const k = map[e.key];
    if (!k) return;
    e.preventDefault();
    if (k === 'stop') {
      activateKey(k);
      return;
    }
    ctrl.pressed.add(k);
    applyPressed();
    startSendLoop();
  });
  document.addEventListener('keyup', (e) => {
    const map = { 'ArrowUp': 'up', 'ArrowDown': 'down', 'ArrowLeft': 'left', 'ArrowRight': 'right' };
    const k = map[e.key];
    if (!k) return;
    ctrl.pressed.delete(k);
    applyPressed();
    if (ctrl.pressed.size === 0) stopSendLoop();
  });
});

function startButtonPress(e, btn, key) {
  e.preventDefault();
  if (e.pointerId !== undefined) {
    ctrl.activePointers.set(e.pointerId, key);
    try {
      btn.setPointerCapture(e.pointerId);
    } catch (_) {}
  }
  if (!ctrl.owned) {
    ctrl.pendingKeys.add(key);
    sendMessage({ type: 'claim_control' });
    flashClaimHint();
    return;
  }
  activateKey(key);
}

function endButtonPress(e, btn, key) {
  e.preventDefault();
  if (e.pointerId !== undefined) {
    const activeKey = ctrl.activePointers.get(e.pointerId);
    if (!activeKey && e.type === 'lostpointercapture') return;
    if (activeKey && activeKey !== key) key = activeKey;
    ctrl.activePointers.delete(e.pointerId);
    try {
      if (btn.hasPointerCapture(e.pointerId)) btn.releasePointerCapture(e.pointerId);
    } catch (_) {}
  }
  if (!ctrl.owned) {
    if (!hasActivePointerForKey(key)) ctrl.pendingKeys.delete(key);
    return;
  }
  releaseKey(key);
}

function hasActivePointerForKey(key) {
  for (const activeKey of ctrl.activePointers.values()) {
    if (activeKey === key) return true;
  }
  return false;
}

function activateKey(key) {
  if (key === 'stop') {
    ctrl.pressed.clear();
    ctrl.linearX = 0; ctrl.linearY = 0; ctrl.angularZ = 0;
    // Flash the stop button
    const stopBtn = document.querySelector('.dpad-stop');
    if (stopBtn) {
      stopBtn.classList.add('pressed');
      setTimeout(() => stopBtn.classList.remove('pressed'), 120);
    }
    stopSendLoop();
    return;
  } else {
    ctrl.pressed.add(key);
    applyPressed();
  }
  // Visual feedback for direction buttons
  const btn = document.querySelector(`.dpad-btn[data-key="${key}"]`);
  if (btn) btn.classList.add('pressed');
  startSendLoop();
}

function releaseKey(key) {
  if (key !== 'stop') {
    ctrl.pressed.delete(key);
    applyPressed();
  }
  const btn = document.querySelector(`.dpad-btn[data-key="${key}"]`);
  if (btn) btn.classList.remove('pressed');
  if (ctrl.pressed.size === 0) stopSendLoop();
}

function replayPendingKeys() {
  if (ctrl.pendingKeys.size === 0) return;
  for (const key of ctrl.pendingKeys) {
    activateKey(key);
  }
  ctrl.pendingKeys.clear();
}

function applyPressed() {
  let lx = 0, ly = 0, az = 0;
  if (ctrl.pressed.has('up'))    lx += ctrl.linSpeed;
  if (ctrl.pressed.has('down'))  lx -= ctrl.linSpeed;
  if (ctrl.pressed.has('left'))  az += ctrl.angSpeed;
  if (ctrl.pressed.has('right')) az -= ctrl.angSpeed;
  ctrl.linearX = lx;
  ctrl.linearY = ly;
  ctrl.angularZ = az;
}

function startSendLoop() {
  if (ctrl.sendTimer) return;
  sendOnce();
  ctrl.sendTimer = setInterval(sendOnce, SEND_INTERVAL);
  // Safety watchdog: stop loop if no interaction for WATCHDOG_MS
  clearTimeout(ctrl.watchdogTimer);
  ctrl.watchdogTimer = setTimeout(() => {
    console.warn('[control] watchdog fired, stopping send loop');
    ctrl.pressed.clear();
    document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('pressed'));
    stopSendLoop();
  }, WATCHDOG_MS);
}

function stopSendLoop(sendZero = true) {
  if (ctrl.sendTimer) {
    clearInterval(ctrl.sendTimer);
    ctrl.sendTimer = null;
  }
  clearTimeout(ctrl.watchdogTimer);
  ctrl.linearX = 0; ctrl.linearY = 0; ctrl.angularZ = 0;
  if (sendZero) sendOnce();
}

function sendOnce() {
  sendMessage({
    type: 'set_velocity',
    linear_x: ctrl.linearX,
    linear_y: ctrl.linearY,
    angular_z: ctrl.angularZ,
  });
}

function flashClaimHint() {
  const btn = document.getElementById('btn-claim');
  btn.animate(
    [{ background: 'var(--warn)' }, { background: '' }],
    { duration: 400 }
  );
}

// Server messages — register with app.js handler registry
registerHandler('claim_control_result', (msg) => {
  if (msg.ok) {
    ctrl.owned = true;
    updateClaimUI();
    replayPendingKeys();
  } else {
    ctrl.owned = false;
    ctrl.pendingKeys.clear();
    updateClaimUI();
    showAlert('无法获取控制权', '另一客户端正在使用,请等待对方释放。');
  }
});

registerHandler('control_status', (msg) => {
  const myId = state.clientId;
  const owner = msg.owner;
  const wasOwned = ctrl.owned;
  if (owner && owner !== myId) {
    ctrl.owned = false;
  } else if (owner === myId) {
    ctrl.owned = true;
  } else if (!owner) {
    ctrl.owned = false;
  }
  if (wasOwned && !ctrl.owned) {
    ctrl.pressed.clear();
    ctrl.pendingKeys.clear();
    ctrl.activePointers.clear();
    document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('pressed'));
    stopSendLoop(false);
  }
  updateClaimUI();
  const ownerEl = document.getElementById('ctrl-owner');
  if (!owner) {
    ownerEl.textContent = '空闲';
    ownerEl.className = 'muted small';
  } else if (owner === myId) {
    ownerEl.textContent = '✓ 你在控制';
    ownerEl.className = 'small badge badge-good';
  } else {
    ownerEl.textContent = `占用: ${owner}`;
    ownerEl.className = 'small badge badge-bad';
  }
  updateControlDiagnostics(msg);
});

function updateControlDiagnostics(msg) {
  const velocityEl = document.getElementById('ctrl-diag-velocity');
  const cmdvelEl = document.getElementById('ctrl-diag-cmdvel');
  const chassisEl = document.getElementById('ctrl-diag-chassis');
  const portEl = document.getElementById('ctrl-diag-port');
  const writeEl = document.getElementById('ctrl-diag-write');
  const cmdAgeEl = document.getElementById('ctrl-diag-cmd-age');
  const errorRow = document.getElementById('ctrl-diag-error-row');
  const errorEl = document.getElementById('ctrl-diag-error');
  if (!velocityEl) return;

  const lx = Number(msg.linear_x || 0);
  const az = Number(msg.angular_z || 0);
  velocityEl.textContent = `vx ${lx.toFixed(2)} / wz ${az.toFixed(2)}`;

  const subscriberCount = Number(msg.cmd_vel_subscribers || 0);
  cmdvelEl.textContent = `${subscriberCount}`;
  cmdvelEl.className = subscriberCount > 0 ? 'badge badge-good' : 'badge badge-bad';

  const chassis = msg.chassis || {};
  if (!chassis.available) {
    chassisEl.textContent = '无状态';
    chassisEl.className = 'badge badge-warn';
    portEl.textContent = '--';
    writeEl.textContent = '--';
    cmdAgeEl.textContent = '--';
    hideControlError(errorRow, errorEl);
    return;
  }

  portEl.textContent = chassis.port || '--';

  if (chassis.stale) {
    chassisEl.textContent = '超时';
    chassisEl.className = 'badge badge-bad';
  } else if (chassis.connected && chassis.port_open) {
    chassisEl.textContent = '已连接';
    chassisEl.className = 'badge badge-good';
  } else if (chassis.port_open) {
    chassisEl.textContent = '未就绪';
    chassisEl.className = 'badge badge-warn';
  } else {
    chassisEl.textContent = '未连接';
    chassisEl.className = 'badge badge-bad';
  }

  if (!chassis.has_last_cmd) {
    writeEl.textContent = '--';
    cmdAgeEl.textContent = '--';
  } else {
    writeEl.textContent = chassis.last_write_ok ? 'OK' : '失败';
    cmdAgeEl.textContent = formatAge(chassis.last_cmd_age);
  }

  const err = chassis.last_error || chassis.parse_error || '';
  if (err) {
    errorEl.textContent = err;
    errorRow.classList.remove('hidden');
  } else {
    hideControlError(errorRow, errorEl);
  }
}

function hideControlError(row, el) {
  if (el) el.textContent = '';
  if (row) row.classList.add('hidden');
}

function formatAge(value) {
  const age = Number(value);
  if (!Number.isFinite(age) || age < 0) return '--';
  if (age < 10) return `${age.toFixed(1)}s`;
  return `${age.toFixed(0)}s`;
}

function updateClaimUI() {
  const btn = document.getElementById('btn-claim');
  if (ctrl.owned) {
    btn.textContent = '释放控制权';
    btn.classList.add('btn-danger');
    btn.classList.remove('btn-secondary');
  } else {
    btn.textContent = '申请控制权';
    btn.classList.remove('btn-danger');
    btn.classList.add('btn-secondary');
  }
  document.querySelectorAll('.dpad-btn').forEach(b => {
    b.classList.toggle('dpad-unowned', !ctrl.owned);
    b.setAttribute('aria-disabled', ctrl.owned ? 'false' : 'true');
  });
}

// Release on tab close
window.addEventListener('beforeunload', () => {
  if (ctrl.owned) sendMessage({ type: 'release_control' });
});
