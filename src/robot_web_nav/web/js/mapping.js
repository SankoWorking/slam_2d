// Mapping workflow: start/stop SLAM and save the current /map.

const mappingUi = {
  running: false,
  backend: null,
  startedAt: null,
  tickTimer: null,
};

window.addEventListener('load', () => {
  const startBtn = document.getElementById('btn-start-mapping');
  const stopBtn = document.getElementById('btn-stop-mapping');
  const saveBtn = document.getElementById('btn-save-mapping');
  const nameInput = document.getElementById('mapping-name');

  startBtn.addEventListener('click', () => {
    const backend = document.getElementById('mapping-backend').value;
    sendMessage({ type: 'start_mapping', backend });
    setMappingText('正在启动建图...');
  });

  stopBtn.addEventListener('click', () => {
    showConfirm('停止建图', '确认停止当前建图任务?', () => {
      sendMessage({ type: 'stop_mapping' });
      setMappingText('正在停止建图...');
    });
  });

  saveBtn.addEventListener('click', saveMapping);

  nameInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') saveMapping();
  });
});

registerHandler('mapping_status', (msg) => {
  mappingUi.running = !!msg.running;
  mappingUi.backend = msg.backend || null;
  mappingUi.startedAt = msg.started_at || null;
  updateMappingUi(msg);
  updateLiveMapStats();
});

registerHandler('map_saved', (msg) => {
  const id = msg.map?.id || 'new map';
  setMappingText(`已保存: ${id}`);
  const nameInput = document.getElementById('mapping-name');
  const displayInput = document.getElementById('mapping-display-input');
  if (nameInput) nameInput.value = '';
  if (displayInput) displayInput.value = '';
  updateLiveMapStats();
});

function saveMapping() {
  const nameInput = document.getElementById('mapping-name');
  const displayInput = document.getElementById('mapping-display-input');
  const overwriteInput = document.getElementById('mapping-overwrite');
  const name = nameInput.value.trim();
  if (!name) {
    toastError('请输入地图文件名');
    nameInput.focus();
    return;
  }
  if (!/^[A-Za-z0-9_.-]+$/.test(name)) {
    toastError('文件名只能包含字母、数字、点、短横线和下划线');
    nameInput.focus();
    return;
  }
  sendMessage({
    type: 'save_mapping',
    name,
    display_name: displayInput.value.trim(),
    overwrite: overwriteInput.checked,
  });
  setMappingText('正在保存地图...');
}

function updateMappingUi(msg) {
  const badge = document.getElementById('mapping-state');
  const backend = document.getElementById('mapping-backend');
  const startBtn = document.getElementById('btn-start-mapping');
  const stopBtn = document.getElementById('btn-stop-mapping');
  const saveBtn = document.getElementById('btn-save-mapping');

  badge.textContent = mappingUi.running ? '建图中' : '未运行';
  badge.className = mappingUi.running ? 'badge badge-good' : 'badge badge-bad';
  backend.disabled = mappingUi.running;
  startBtn.disabled = mappingUi.running;
  stopBtn.disabled = !mappingUi.running;
  saveBtn.classList.toggle('btn-primary', mappingUi.running);
  saveBtn.classList.toggle('btn-secondary', !mappingUi.running);

  clearInterval(mappingUi.tickTimer);
  if (mappingUi.running) {
    updateMappingElapsed();
    mappingUi.tickTimer = setInterval(updateMappingElapsed, 1000);
  } else {
    const error = msg.last_error || '';
    setMappingText(error ? `已停止: ${error}` : '等待建图任务');
  }
}

function updateMappingElapsed() {
  const started = mappingUi.startedAt;
  if (!started) {
    setMappingText(`${mappingBackendName(mappingUi.backend)} 运行中`);
    return;
  }
  const elapsed = Math.max(0, Math.floor(Date.now() / 1000 - started));
  const mm = String(Math.floor(elapsed / 60)).padStart(2, '0');
  const ss = String(elapsed % 60).padStart(2, '0');
  setMappingText(`${mappingBackendName(mappingUi.backend)} 运行中 ${mm}:${ss}`);
}

function setMappingText(text) {
  const el = document.getElementById('mapping-status-text');
  if (el) el.textContent = text;
}

function mappingBackendName(backend) {
  if (backend === 'cartographer') return 'Cartographer';
  return 'SLAM Toolbox';
}

function updateLiveMapStats() {
  const stateEl = document.getElementById('mapping-live-state');
  const knownEl = document.getElementById('mapping-known-percent');
  const occupiedEl = document.getElementById('mapping-occupied-cells');
  if (!stateEl || !knownEl || !occupiedEl) return;

  if (!mappingUi.running) {
    stateEl.textContent = '未运行';
    knownEl.textContent = '--';
    occupiedEl.textContent = '--';
    return;
  }

  if (!state.liveMapActive || !state.liveMapStats) {
    stateEl.textContent = '等待 /map';
    knownEl.textContent = '--';
    occupiedEl.textContent = '--';
    return;
  }

  stateEl.textContent = '接收中';
  knownEl.textContent = `${Number(state.liveMapStats.knownPercent || 0).toFixed(1)}%`;
  occupiedEl.textContent = String(state.liveMapStats.occupiedCells || 0);
}
