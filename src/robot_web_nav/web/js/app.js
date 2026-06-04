// Global state + WebSocket plumbing + UI message routing.

const state = {
  ws: null,
  connected: false,
  clientId: null,
  activePage: 'nav',
  currentMap: 'map',
  mapMeta: null,
  liveMapActive: false,
  liveMapStats: null,
  waypoints: [],
  robotPose: null,
  pendingClick: null,
  navStatus: null,
  errorLog: [],
};

// ===== Handler registry =====
const _handlers = {};

function registerHandler(type, fn) {
  _handlers[type] = fn;
}

function handleMessage(msg) {
  const fn = _handlers[msg.type];
  if (fn) {
    fn(msg);
  } else if (msg.type && msg.type !== 'ping' && msg.type !== 'pong') {
    console.warn('[ws] unhandled message type:', msg.type);
  }
}

// ===== Feature pages =====

function setFeaturePage(page) {
  state.activePage = page;
  document.querySelectorAll('.feature-tab').forEach(btn => {
    const active = btn.getAttribute('data-page') === page;
    btn.classList.toggle('active', active);
    btn.setAttribute('aria-selected', active ? 'true' : 'false');
  });
  document.querySelectorAll('.feature-panel').forEach(panel => {
    panel.classList.toggle('active', panel.getAttribute('data-feature-page') === page);
  });
  if (typeof renderMap === 'function') renderMap();
}

function initFeaturePages() {
  document.querySelectorAll('.feature-tab').forEach(btn => {
    btn.addEventListener('click', () => {
      setFeaturePage(btn.getAttribute('data-page'));
    });
  });
  setFeaturePage(state.activePage);
}

// ===== WebSocket =====
let _reconnectDelay = 1000;
const RECONNECT_MIN = 1000;
const RECONNECT_MAX = 30000;
let _pingInterval = null;
let _pongTimeout = null;

function connectWebSocket() {
  const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const url = `${proto}//${location.host}/ws`;
  state.ws = new WebSocket(url);

  state.ws.onopen = () => {
    state.connected = true;
    _reconnectDelay = RECONNECT_MIN;
    updateConnectionStatus(true);
    hideReconnectBanner();
    sendMessage({ type: 'list_maps' });
    startHeartbeat();
  };

  state.ws.onclose = () => {
    state.connected = false;
    updateConnectionStatus(false);
    stopHeartbeat();
    showReconnectBanner();
    setTimeout(connectWebSocket, _reconnectDelay);
    _reconnectDelay = Math.min(_reconnectDelay * 2, RECONNECT_MAX);
  };

  state.ws.onerror = () => {};

  state.ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      handleMessage(msg);
    } catch (e) {
      console.warn('[ws] non-JSON message:', event.data);
    }
  };
}

function startHeartbeat() {
  stopHeartbeat();
  _pingInterval = setInterval(() => {
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
      sendMessage({ type: 'ping' });
      _pongTimeout = setTimeout(() => {
        console.warn('[ws] heartbeat timeout, closing connection');
        if (state.ws) state.ws.close();
      }, 5000);
    }
  }, 15000);
}

function stopHeartbeat() {
  clearInterval(_pingInterval);
  clearTimeout(_pongTimeout);
  _pingInterval = null;
  _pongTimeout = null;
}

function sendMessage(msg) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(msg));
  }
}

// ===== Built-in handlers =====

registerHandler('ping', () => { sendMessage({ type: 'pong' }); });
registerHandler('pong', () => { clearTimeout(_pongTimeout); });

registerHandler('hello', (msg) => {
  state.clientId = msg.client_id;
  const idEl = document.getElementById('client-id-text');
  if (idEl) idEl.textContent = `#${state.clientId}`;
});

registerHandler('map_list', (msg) => {
  onMapList(msg);
});

registerHandler('waypoints', (msg) => {
  state.waypoints = msg.waypoints;
  renderWaypointList();
  if (typeof renderMap === 'function') renderMap();
});

registerHandler('waypoint_added', (msg) => {
  const exists = state.waypoints.find(w => w.name === msg.waypoint.name);
  if (!exists) state.waypoints.push(msg.waypoint);
  else Object.assign(exists, msg.waypoint);
  renderWaypointList();
  if (typeof renderMap === 'function') renderMap();
});

registerHandler('waypoint_deleted', (msg) => {
  state.waypoints = state.waypoints.filter(w => w.name !== msg.name);
  renderWaypointList();
  if (typeof renderMap === 'function') renderMap();
});

registerHandler('robot_pose', (msg) => {
  state.robotPose = msg;
  updatePoseReadout(msg);
});

registerHandler('robot_pose_full', (msg) => {
  state.robotPose = msg;
  updatePoseReadout(msg);
});

registerHandler('nav_status', (msg) => {
  state.navStatus = msg.status === 'navigating' ? msg : null;
  updateNavStatus(state.navStatus ? msg : null);
});

registerHandler('nav_result', (msg) => {
  state.navStatus = null;
  state.plannedPath = null;
  updateNavStatus(null);
  if (typeof renderMap === 'function') renderMap();
});

registerHandler('nav_feedback', (msg) => {
  updateNavFeedback(msg);
});

registerHandler('error', (msg) => {
  console.warn('[server error]', msg.message);
  toastError(msg.message);
  state.errorLog.push({ time: new Date(), message: msg.message, level: 'error' });
  updateErrorBadge();
  renderLogPanel();
});

registerHandler('log_entries', (msg) => {
  if (msg.entries) {
    for (const entry of msg.entries) {
      state.errorLog.push({ time: new Date(entry.timestamp * 1000), message: entry.message, level: entry.level });
    }
    renderLogPanel();
  }
});

registerHandler('map_renamed', () => {});
registerHandler('map_duplicated', () => {});

// ===== Connection UI =====

function updateConnectionStatus(connected) {
  const dot = document.getElementById('connection-dot');
  const text = document.getElementById('connection-text');
  dot.className = connected ? 'dot connected' : 'dot disconnected';
  text.textContent = connected ? '已连接' : '未连接';
}

function showReconnectBanner() {
  let banner = document.getElementById('reconnect-banner');
  if (!banner) {
    banner = document.createElement('div');
    banner.id = 'reconnect-banner';
    banner.className = 'reconnect-banner';
    banner.textContent = '连接断开,正在重连...';
    const container = document.getElementById('map-container');
    if (container) container.appendChild(banner);
  }
  banner.classList.remove('hidden');
}

function hideReconnectBanner() {
  const banner = document.getElementById('reconnect-banner');
  if (banner) banner.classList.add('hidden');
}

// ===== Map loading state =====

function showMapLoading() {
  const el = document.getElementById('map-loading');
  if (el) el.classList.remove('hidden');
}

function hideMapLoading() {
  const el = document.getElementById('map-loading');
  if (el) el.classList.add('hidden');
}

// ===== Pose readout =====

function updatePoseReadout(pose) {
  const x = document.getElementById('pose-x');
  const y = document.getElementById('pose-y');
  const yaw = document.getElementById('pose-yaw');
  if (!x) return;
  x.textContent = pose.x.toFixed(2);
  y.textContent = pose.y.toFixed(2);
  yaw.textContent = (pose.yaw * 180 / Math.PI).toFixed(0);
}

// ===== Map selector =====

function onMapList(msg) {
  if (msg.current) state.currentMap = msg.current;
  const sel = document.getElementById('map-selector');
  sel.innerHTML = '';
  msg.maps.forEach(m => {
    const opt = document.createElement('option');
    opt.value = m.id;
    opt.textContent = m.name;
    if (m.id === msg.current) opt.selected = true;
    sel.appendChild(opt);
  });
  sel.onchange = () => {
    state.currentMap = sel.value;
    sendMessage({ type: 'load_map', map_name: sel.value });
  };
  const cur = msg.maps.find(m => m.id === msg.current);
  const dn = document.getElementById('map-display-name');
  if (dn) dn.textContent = cur ? `显示名: ${cur.name}` : '';
}

// ===== Waypoints =====

function renderWaypointList() {
  const list = document.getElementById('waypoint-list');
  const counter = document.getElementById('wp-count');
  if (counter) counter.textContent = state.waypoints.length;
  if (state.waypoints.length === 0) {
    list.innerHTML = '<div class="wp-empty">双击地图添加路径点</div>';
    return;
  }
  list.innerHTML = '';
  state.waypoints.forEach(wp => {
    const div = document.createElement('div');
    div.className = 'wp-item';
    div.innerHTML = `
      <div class="wp-info">
        <span class="wp-name"></span>
        <span class="wp-coords">(${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})</span>
      </div>
      <div class="wp-actions">
        <button class="btn btn-primary btn-sm" data-action="nav">导航</button>
        <button class="btn btn-danger btn-sm" data-action="del">删除</button>
      </div>
    `;
    div.querySelector('.wp-name').textContent = wp.name;
    div.querySelector('[data-action="nav"]').onclick = (e) => {
      e.stopPropagation();
      sendMessage({ type: 'navigate_to', name: wp.name });
    };
    div.querySelector('[data-action="del"]').onclick = (e) => {
      e.stopPropagation();
      showConfirm('删除路径点', `确认删除 "${wp.name}"?`, () => {
        sendMessage({ type: 'delete_waypoint', name: wp.name });
      });
    };
    list.appendChild(div);
  });
}

// ===== Navigation status =====

function updateNavStatus(msg) {
  const el = document.getElementById('nav-status');
  const text = document.getElementById('nav-status-text');
  if (!msg) {
    el.classList.add('hidden');
    return;
  }
  el.classList.remove('hidden');
  const goal = msg.goal || {};
  text.textContent = `正在导航到 ${goal.name || '目标点'}...`;
}

function updateNavFeedback(msg) {
  const text = document.getElementById('nav-status-text');
  const goal = state.navStatus?.goal?.name || '目标点';
  const dist = msg.distance_remaining;
  if (typeof dist === 'number') {
    text.textContent = `导航到 ${goal},剩余 ${dist.toFixed(2)} m`;
  }
}

document.getElementById('btn-cancel-nav').onclick = () => {
  sendMessage({ type: 'cancel_nav' });
};

// ===== Waypoint add overlay =====

const overlay = document.getElementById('map-overlay');
const nameInput = document.getElementById('waypoint-name-input');

document.getElementById('btn-confirm-waypoint').onclick = () => {
  const name = nameInput.value.trim();
  if (!name) return;
  sendMessage({
    type: 'add_waypoint',
    name: name,
    pixel_x: state.pendingClick.px,
    pixel_y: state.pendingClick.py,
  });
  overlay.classList.add('hidden');
  state.pendingClick = null;
};

document.getElementById('btn-cancel-waypoint').onclick = () => {
  overlay.classList.add('hidden');
  state.pendingClick = null;
};

nameInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') document.getElementById('btn-confirm-waypoint').click();
  if (e.key === 'Escape') document.getElementById('btn-cancel-waypoint').click();
});

// ===== Map rename =====

const renameOverlay = document.getElementById('rename-overlay');
const renameInput = document.getElementById('rename-input');

document.getElementById('btn-map-rename').onclick = () => {
  if (!state.currentMap) return;
  document.getElementById('rename-current').textContent =
    `当前文件名: ${state.currentMap}`;
  renameInput.value = state.currentMap;
  renameOverlay.classList.remove('hidden');
  setTimeout(() => renameInput.focus(), 100);
};

document.getElementById('btn-rename-confirm').onclick = () => {
  const newName = renameInput.value.trim();
  if (!newName || newName === state.currentMap) {
    renameOverlay.classList.add('hidden');
    return;
  }
  sendMessage({ type: 'rename_map', old: state.currentMap, new: newName });
  renameOverlay.classList.add('hidden');
};

document.getElementById('btn-rename-cancel').onclick = () => {
  renameOverlay.classList.add('hidden');
};

renameInput.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') document.getElementById('btn-rename-confirm').click();
  if (e.key === 'Escape') document.getElementById('btn-rename-cancel').click();
});

// ===== Custom confirm / alert dialogs =====

function showConfirm(title, message, onYes) {
  const dlg = document.getElementById('confirm-dialog');
  document.getElementById('confirm-title').textContent = title;
  document.getElementById('confirm-message').textContent = message;
  dlg.classList.remove('hidden');

  const yesBtn = document.getElementById('confirm-yes');
  const noBtn = document.getElementById('confirm-no');
  const cleanup = () => {
    dlg.classList.add('hidden');
    yesBtn.replaceWith(yesBtn.cloneNode(true));
    noBtn.replaceWith(noBtn.cloneNode(true));
  };
  document.getElementById('confirm-yes').addEventListener('click', () => {
    cleanup();
    onYes();
  });
  document.getElementById('confirm-no').addEventListener('click', cleanup);
}

function showAlert(title, message) {
  const dlg = document.getElementById('alert-dialog');
  document.getElementById('alert-title').textContent = title;
  document.getElementById('alert-message').textContent = message;
  dlg.classList.remove('hidden');

  const okBtn = document.getElementById('alert-ok');
  const cleanup = () => {
    dlg.classList.add('hidden');
    okBtn.replaceWith(okBtn.cloneNode(true));
  };
  document.getElementById('alert-ok').addEventListener('click', cleanup);
}

// ===== Toast error =====

function toastError(message) {
  let toast = document.getElementById('error-toast');
  if (!toast) {
    toast = document.createElement('div');
    toast.id = 'error-toast';
    toast.className = 'error-toast';
    document.body.appendChild(toast);
  }
  toast.textContent = message;
  toast.classList.add('visible');
  clearTimeout(toast._t);
  toast._t = setTimeout(() => { toast.classList.remove('visible'); }, 3500);
}

// ===== Error badge =====

function updateErrorBadge() {
  const badge = document.getElementById('error-badge');
  if (!badge) return;
  const count = state.errorLog.length;
  if (count > 0) {
    badge.textContent = count;
    badge.classList.remove('hidden');
  } else {
    badge.classList.add('hidden');
  }
}

// ===== Log panel rendering =====

function renderLogPanel() {
  const list = document.getElementById('log-list');
  if (!list) return;
  const recent = state.errorLog.slice(-50);
  list.innerHTML = '';
  recent.forEach(entry => {
    const div = document.createElement('div');
    const level = entry.level || 'warn';
    const time = entry.time.toLocaleTimeString();
    div.className = `log-entry log-${level}`;
    div.textContent = `[${time}] ${entry.message}`;
    list.appendChild(div);
  });
  list.scrollTop = list.scrollHeight;
}

// ===== Init =====

window.addEventListener('load', () => {
  initFeaturePages();
  showMapLoading();
  connectWebSocket();

  // Logs panel toggle
  document.getElementById('logs-header').addEventListener('click', () => {
    const panel = document.getElementById('logs-panel');
    const toggle = document.getElementById('logs-toggle');
    panel.classList.toggle('hidden');
    toggle.textContent = panel.classList.contains('hidden') ? '▶' : '▼';
  });

  // Error badge click opens logs
  document.getElementById('error-badge').addEventListener('click', () => {
    setFeaturePage('nav');
    const panel = document.getElementById('logs-panel');
    const toggle = document.getElementById('logs-toggle');
    panel.classList.remove('hidden');
    toggle.textContent = '▼';
  });
});
