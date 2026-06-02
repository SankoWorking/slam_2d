import base64
import io
import json
import os
import shutil
import threading
from dataclasses import dataclass

import yaml
from PIL import Image


@dataclass
class MapInfo:
    image_path: str
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


class MapService:
    """Loads, lists, renames and duplicates ROS map (.yaml + .pgm) pairs.

    Display names live in maps_meta.json alongside the maps and are
    purely cosmetic; the file basename remains the canonical id used
    across the rest of the system.
    """

    _META_FILE = 'maps_meta.json'

    def __init__(self, maps_dir: str):
        self._maps_dir = maps_dir
        self._loaded_maps: dict[str, MapInfo] = {}
        self._meta_lock = threading.Lock()
        self._meta = self._load_meta()

    # ----- meta (display names) -----
    def _meta_path(self) -> str:
        return os.path.join(self._maps_dir, self._META_FILE)

    def _load_meta(self) -> dict:
        path = self._meta_path()
        if os.path.exists(path):
            try:
                with open(path, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception:
                return {}
        return {}

    def _save_meta(self):
        os.makedirs(self._maps_dir, exist_ok=True)
        with open(self._meta_path(), 'w', encoding='utf-8') as f:
            json.dump(self._meta, f, ensure_ascii=False, indent=2)

    def get_display_name(self, map_name: str) -> str:
        with self._meta_lock:
            entry = self._meta.get(map_name, {})
            return entry.get('display_name') or map_name

    def set_display_name(self, map_name: str, display_name: str):
        with self._meta_lock:
            if display_name and display_name != map_name:
                self._meta.setdefault(map_name, {})['display_name'] = display_name
            else:
                if map_name in self._meta and 'display_name' in self._meta[map_name]:
                    del self._meta[map_name]['display_name']
                    if not self._meta[map_name]:
                        del self._meta[map_name]
            self._save_meta()

    # ----- listing -----
    def list_maps(self) -> list[str]:
        names = set()
        for f in os.listdir(self._maps_dir):
            if f.endswith('.yaml') and not f.endswith('.bak'):
                names.add(f[:-5])
        return sorted(names)

    def list_maps_with_display(self) -> list[dict]:
        return [
            {'id': name, 'name': self.get_display_name(name)}
            for name in self.list_maps()
        ]

    # ----- load -----
    def load_map(self, map_name: str) -> MapInfo:
        if map_name in self._loaded_maps:
            return self._loaded_maps[map_name]

        yaml_path = os.path.join(self._maps_dir, f'{map_name}.yaml')
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        image_rel = meta['image'].strip()
        image_path = os.path.join(self._maps_dir, image_rel)
        img = Image.open(image_path)
        width, height = img.size

        origin = meta['origin']
        info = MapInfo(
            image_path=image_path,
            resolution=float(meta['resolution']),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            width=width,
            height=height,
        )
        self._loaded_maps[map_name] = info
        return info

    def get_map_png_base64(self, map_name: str) -> tuple[str, MapInfo]:
        info = self.load_map(map_name)
        img = Image.open(info.image_path)
        buf = io.BytesIO()
        img.save(buf, format='PNG')
        b64 = base64.b64encode(buf.getvalue()).decode('ascii')
        return b64, info

    # ----- coordinate conversion -----
    def pixel_to_world(self, map_name: str, px: int, py: int) -> tuple[float, float]:
        info = self.load_map(map_name)
        wx = info.origin_x + px * info.resolution
        wy = info.origin_y + (info.height - py) * info.resolution
        return wx, wy

    def world_to_pixel(self, map_name: str, wx: float, wy: float) -> tuple[int, int]:
        info = self.load_map(map_name)
        px = int(round((wx - info.origin_x) / info.resolution))
        py = int(round(info.height - (wy - info.origin_y) / info.resolution))
        return px, py

    # ----- rename / duplicate -----
    @staticmethod
    def _sanitize(name: str) -> str:
        """Filesystem-safe map id. Reject anything path-like."""
        name = name.strip()
        if not name or '/' in name or '\\' in name or name.startswith('.'):
            raise ValueError(f'invalid map name: {name!r}')
        return name

    def _read_yaml_image_field(self, map_name: str) -> str:
        yaml_path = os.path.join(self._maps_dir, f'{map_name}.yaml')
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)['image'].strip()

    def _write_yaml(self, map_name: str, data: dict):
        yaml_path = os.path.join(self._maps_dir, f'{map_name}.yaml')
        with open(yaml_path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)

    def rename_map(self, old: str, new: str) -> str:
        """Rename both .yaml and the .pgm it references. Returns new id."""
        old = self._sanitize(old)
        new = self._sanitize(new)
        if old == new:
            return new
        if not os.path.exists(os.path.join(self._maps_dir, f'{old}.yaml')):
            raise FileNotFoundError(f'map not found: {old}')
        if os.path.exists(os.path.join(self._maps_dir, f'{new}.yaml')):
            raise FileExistsError(f'map already exists: {new}')

        old_image_rel = self._read_yaml_image_field(old)
        old_image_abs = os.path.join(self._maps_dir, old_image_rel)
        # Decide new image filename — replace leading "<old>." with "<new>."
        if old_image_rel.startswith(f'{old}.'):
            new_image_rel = f'{new}.{old_image_rel[len(old)+1:]}'
        else:
            new_image_rel = f'{new}.pgm'
        new_image_abs = os.path.join(self._maps_dir, new_image_rel)

        # Move image
        shutil.move(old_image_abs, new_image_abs)
        # Move yaml, then patch its image field
        old_yaml = os.path.join(self._maps_dir, f'{old}.yaml')
        new_yaml = os.path.join(self._maps_dir, f'{new}.yaml')
        with open(old_yaml, 'r') as f:
            data = yaml.safe_load(f)
        data['image'] = new_image_rel
        self._write_yaml(new, data)
        os.remove(old_yaml)

        # Drop cache
        self._loaded_maps.pop(old, None)
        self._loaded_maps.pop(new, None)

        # Migrate display name
        with self._meta_lock:
            if old in self._meta:
                self._meta[new] = self._meta.pop(old)
                self._save_meta()
        return new

    def duplicate_map(self, src: str, new_name: str) -> str:
        """Copy an existing map (.yaml + .pgm) to a new id."""
        src = self._sanitize(src)
        new_name = self._sanitize(new_name)
        if not os.path.exists(os.path.join(self._maps_dir, f'{src}.yaml')):
            raise FileNotFoundError(f'map not found: {src}')
        if os.path.exists(os.path.join(self._maps_dir, f'{new_name}.yaml')):
            raise FileExistsError(f'map already exists: {new_name}')

        src_image_rel = self._read_yaml_image_field(src)
        src_image_abs = os.path.join(self._maps_dir, src_image_rel)
        if src_image_rel.startswith(f'{src}.'):
            new_image_rel = f'{new_name}.{src_image_rel[len(src)+1:]}'
        else:
            new_image_rel = f'{new_name}.pgm'
        new_image_abs = os.path.join(self._maps_dir, new_image_rel)

        shutil.copy2(src_image_abs, new_image_abs)
        with open(os.path.join(self._maps_dir, f'{src}.yaml'), 'r') as f:
            data = yaml.safe_load(f)
        data['image'] = new_image_rel
        self._write_yaml(new_name, data)
        return new_name
