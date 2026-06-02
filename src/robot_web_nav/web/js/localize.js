// Auto-localization UI: triggers AMCL global localization and shows covariance.

let _locTimeout = null;
const LOC_TIMEOUT_MS = 30000;

window.addEventListener('load', () => {
  document.getElementById('btn-auto-localize').addEventListener('click', () => {
    showConfirm('自动定位', '将触发 AMCL 全局定位。请准备遥控机器人走 1~2 米并旋转 360°,直到位置σ < 0.05。继续?', () => {
      sendMessage({ type: 'start_localization' });
      setStatus('正在撒粒子,请遥控机器人...', 'badge-warn');
      // Timeout if no convergence data arrives
      clearTimeout(_locTimeout);
      _locTimeout = setTimeout(() => {
        setStatus('定位超时,请重试', 'badge-bad');
        document.getElementById('loc-hint').textContent = '30 秒内未收敛,请确认 AMCL 节点正常运行。';
      }, LOC_TIMEOUT_MS);
    });
  });
});

registerHandler('localization_started', (msg) => {
  if (!msg.ok) {
    clearTimeout(_locTimeout);
    setStatus('不可用: ' + (msg.message || 'service not ready'), 'badge-bad');
  }
});

registerHandler('localization_status', (msg) => {
  const xy = msg.cov_xy;
  const yaw = msg.cov_yaw;
  const xyEl = document.getElementById('loc-cov-xy');
  const yawEl = document.getElementById('loc-cov-yaw');
  xyEl.textContent = (xy == null) ? '—' : xy.toFixed(3);
  yawEl.textContent = (yaw == null) ? '—' : yaw.toFixed(3);

  if (msg.converged) {
    clearTimeout(_locTimeout);
    setStatus('已收敛 ✓', 'badge-good');
    document.getElementById('loc-hint').textContent = '定位完成。触发后遥控机器人走 1~2 米并旋转,直到位置σ < 0.05';
  } else if (xy != null && xy < 0.2) {
    setStatus('接近收敛', 'badge-warn');
  } else if (xy != null) {
    setStatus('定位中...', 'badge-warn');
  } else {
    setStatus('等待数据', 'badge-warn');
  }
});

function setStatus(text, cls) {
  const el = document.getElementById('loc-quality');
  el.textContent = text;
  el.className = 'badge ' + (cls || 'badge-warn');
}
