import json
import os
import threading
from typing import Optional


class WaypointManager:
    def __init__(self, filepath: str):
        self._filepath = filepath
        self._lock = threading.Lock()
        self._waypoints: list[dict] = []
        self._load()

    def _load(self):
        if os.path.exists(self._filepath):
            with open(self._filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)
                self._waypoints = data.get('waypoints', [])
        else:
            self._waypoints = []

    def _save(self):
        os.makedirs(os.path.dirname(self._filepath), exist_ok=True)
        with open(self._filepath, 'w', encoding='utf-8') as f:
            json.dump({'waypoints': self._waypoints}, f, ensure_ascii=False, indent=2)

    def list_waypoints(self) -> list[dict]:
        with self._lock:
            return list(self._waypoints)

    def add_waypoint(self, name: str, x: float, y: float, yaw: float = 0.0) -> dict:
        with self._lock:
            for wp in self._waypoints:
                if wp['name'] == name:
                    wp['x'] = x
                    wp['y'] = y
                    wp['yaw'] = yaw
                    self._save()
                    return wp
            wp = {'name': name, 'x': x, 'y': y, 'yaw': yaw}
            self._waypoints.append(wp)
            self._save()
            return wp

    def delete_waypoint(self, name: str) -> bool:
        with self._lock:
            before = len(self._waypoints)
            self._waypoints = [wp for wp in self._waypoints if wp['name'] != name]
            if len(self._waypoints) < before:
                self._save()
                return True
            return False

    def get_waypoint(self, name: str) -> Optional[dict]:
        with self._lock:
            for wp in self._waypoints:
                if wp['name'] == name:
                    return dict(wp)
        return None
