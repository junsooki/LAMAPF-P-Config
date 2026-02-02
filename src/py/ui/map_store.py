import json
import os
from typing import Dict, List


def _ensure_json_name(name: str) -> str:
    if not name.endswith(".json"):
        return name + ".json"
    return name


def list_maps(maps_dir: str) -> List[str]:
    if not os.path.isdir(maps_dir):
        return []
    names = []
    for fname in os.listdir(maps_dir):
        if fname.endswith(".json"):
            names.append(fname)
    return sorted(names)


def load_map(maps_dir: str, name: str) -> Dict:
    path = os.path.join(maps_dir, name)
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return data


def save_map(maps_dir: str, name: str, data: Dict) -> str:
    os.makedirs(maps_dir, exist_ok=True)
    filename = _ensure_json_name(name)
    path = os.path.join(maps_dir, filename)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    return filename
