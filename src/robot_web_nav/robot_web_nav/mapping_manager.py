import os
import re
import signal
import subprocess
import threading
import time


class MappingManager:
    """Starts/stops SLAM launch files and saves the current /map."""

    _BACKENDS = {
        'slam_toolbox': 'slam.launch.py',
        'cartographer': 'cartographer.launch.py',
    }

    def __init__(self, node, maps_dir: str, log):
        self._node = node
        self._maps_dir = maps_dir
        self._log = log
        self._lock = threading.Lock()
        self._proc = None
        self._backend = None
        self._started_at = None
        self._last_error = ''

    @staticmethod
    def sanitize_map_name(name: str) -> str:
        name = (name or '').strip()
        if not name or '/' in name or '\\' in name or name.startswith('.'):
            raise ValueError(f'invalid map name: {name!r}')
        if not re.fullmatch(r'[A-Za-z0-9_.-]+', name):
            raise ValueError('map file name may only contain letters, digits, dot, dash and underscore')
        return name

    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def start(self, backend: str = 'slam_toolbox') -> dict:
        backend = backend or 'slam_toolbox'
        if backend not in self._BACKENDS:
            raise ValueError(f'unsupported mapping backend: {backend}')

        with self._lock:
            self._cleanup_finished_locked()
            if self._proc is not None and self._proc.poll() is None:
                return self._get_status_locked()

            launch_file = self._BACKENDS[backend]
            log_level = os.environ.get('WEB_NAV_CHILD_LOG_LEVEL', 'warn')
            cmd = [
                'ros2',
                'launch',
                'robot_bringup',
                launch_file,
                f'log_level:={log_level}',
            ]
            try:
                kwargs = {
                    'stdout': subprocess.PIPE,
                    'stderr': subprocess.STDOUT,
                    'text': True,
                    'bufsize': 1,
                }
                if os.name == 'nt':
                    kwargs['creationflags'] = subprocess.CREATE_NEW_PROCESS_GROUP
                else:
                    kwargs['start_new_session'] = True
                self._proc = subprocess.Popen(cmd, **kwargs)
                self._backend = backend
                self._started_at = time.time()
                self._last_error = ''
                threading.Thread(
                    target=self._drain_output,
                    args=(self._proc,),
                    daemon=True,
                ).start()
            except Exception as e:
                self._proc = None
                self._backend = None
                self._started_at = None
                self._last_error = str(e)
                raise

        self._log.info('mapping', f'Started {backend}')
        return self.get_status()

    def stop(self) -> dict:
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                self._cleanup_finished_locked()
                return self._get_status_locked()

        self._log.info('mapping', 'Stopping mapping')
        self._stop_process(proc)
        with self._lock:
            if self._proc is proc:
                self._proc = None
                self._backend = None
                self._started_at = None
        return self.get_status()

    def save_map(self, name: str, overwrite: bool = False) -> dict:
        map_name = self.sanitize_map_name(name)
        stem = os.path.join(self._maps_dir, map_name)
        yaml_path = stem + '.yaml'
        pgm_path = stem + '.pgm'
        if not overwrite and (os.path.exists(yaml_path) or os.path.exists(pgm_path)):
            raise FileExistsError(f'map already exists: {map_name}')

        os.makedirs(self._maps_dir, exist_ok=True)
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', stem]
        self._log.info('mapping', f'Saving map: {map_name}')
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=30,
            )
        except subprocess.TimeoutExpired as e:
            raise TimeoutError('map_saver_cli timed out') from e

        output = (result.stdout or '').strip()
        if result.returncode != 0:
            self._last_error = output[-500:] or f'map_saver_cli exited {result.returncode}'
            raise RuntimeError(self._last_error)

        self._log.info('mapping', f'Map saved: {map_name}')
        return {
            'id': map_name,
            'yaml': yaml_path,
            'pgm': pgm_path,
        }

    def get_status(self) -> dict:
        with self._lock:
            self._cleanup_finished_locked()
            return self._get_status_locked()

    def shutdown(self):
        with self._lock:
            proc = self._proc
        if proc is not None and proc.poll() is None:
            self._stop_process(proc)

    def _cleanup_finished_locked(self):
        if self._proc is not None and self._proc.poll() is not None:
            code = self._proc.returncode
            if code not in (0, None):
                self._last_error = f'mapping process exited with code {code}'
            self._proc = None
            self._backend = None
            self._started_at = None

    def _get_status_locked(self) -> dict:
        running = self._proc is not None and self._proc.poll() is None
        return {
            'running': running,
            'backend': self._backend if running else None,
            'started_at': self._started_at if running else None,
            'last_error': self._last_error,
        }

    def _drain_output(self, proc):
        stream = proc.stdout
        if stream is None:
            return
        try:
            for line in stream:
                line = line.strip()
                if line:
                    self._log.info('mapping', line)
        except Exception as e:
            self._node.get_logger().warn(f'mapping output reader error: {e}')

    def _stop_process(self, proc):
        try:
            if os.name == 'nt':
                proc.terminate()
            else:
                os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8)
            return
        except Exception:
            pass

        try:
            if os.name == 'nt':
                proc.kill()
            else:
                os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3)
        except Exception as e:
            self._last_error = str(e)
