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
    const start = (e) => {
      e.preventDefault();
      if (!ctrl.owned) {
        ctrl.pendingKeys.add(key);
        sendMessage({ type: 'claim_control' });
        flashClaimHint();
        return;
      }
      activateKey(key);
    };
    const end = (e) => {
      e.preventDefault();
      if (key !== 'stop') {
        ctrl.pressed.delete(key);
        applyPressed();
      }
      btn.classList.remove('pressed');
      if (ctrl.pressed.size === 0) stopSendLoop();
    };
    btn.addEventListener('mousedown', start);
    btn.addEventListener('mouseup', end);
    btn.addEventListener('mouseleave', end);
    btn.addEventListener('touchstart', start, { passive: false });
    btn.addEventListener('touchend', end, { passive: false });
    btn.addEventListener('touchcancel', end, { passive: false });
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
      ctrl.pressed.clear();
      document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('pressed'));
    } else {
      ctrl.pressed.add(k);
    }
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
  } else {
    ctrl.pressed.add(key);
    applyPressed();
  }
  // Visual feedback for direction buttons
  const btn = document.querySelector(`.dpad-btn[data-key="${key}"]`);
  if (btn) btn.classList.add('pressed');
  startSendLoop();
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

function stopSendLoop() {
  if (ctrl.sendTimer) {
    clearInterval(ctrl.sendTimer);
    ctrl.sendTimer = null;
  }
  clearTimeout(ctrl.watchdogTimer);
  ctrl.linearX = 0; ctrl.linearY = 0; ctrl.angularZ = 0;
  sendOnce();
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
  if (owner && owner !== myId) {
    ctrl.owned = false;
  } else if (owner === myId) {
    ctrl.owned = true;
  } else if (!owner) {
    ctrl.owned = false;
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
});

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
  document.querySelectorAll('.dpad-btn').forEach(b => b.disabled = !ctrl.owned);
}

// Release on tab close
window.addEventListener('beforeunload', () => {
  if (ctrl.owned) sendMessage({ type: 'release_control' });
});
