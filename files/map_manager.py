import json
import os

CONFIG_FILE = "map_configs.json"

if not os.path.exists(CONFIG_FILE):
    raise FileNotFoundError(f"{CONFIG_FILE} not found")

with open(CONFIG_FILE, "r", encoding="utf-8") as f:
    MAP_CONFIGS = json.load(f)["maps"]  # Load the list of maps


def get_map_names():
    """Return list of map names."""
    return [m["name"] for m in MAP_CONFIGS]


def get_map_config_by_name(map_name):
    """Return map config dictionary for selected map."""
    for m in MAP_CONFIGS:
        if m["name"] == map_name:
            return m
    return None


def latlon_to_pixels(lat, lon, map_cfg):
    """ 
    Convert GPS coordinates to image pixel coordinates. 
    (0,0) is top-left. 
    """
    lat_top = map_cfg["top_left"]["lat"]
    lon_left = map_cfg["top_left"]["lon"]
    lat_bottom = map_cfg["bottom_right"]["lat"]
    lon_right = map_cfg["bottom_right"]["lon"]

    width = map_cfg["image_width"]
    height = map_cfg["image_height"]

    x_frac = (lon - lon_left) / (lon_right - lon_left)
    y_frac = (lat_top - lat) / (lat_top - lat_bottom)

    x = int(x_frac * width)
    y = int(y_frac * height)

    return x, y
