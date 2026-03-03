import json

def load_camera_configs(node):
    
    node.declare_parameter("cameras_json", "")
    raw = node.get_parameter("cameras_json").value

    if not raw:
        node.get_logger().warning("No cameras_json parameter provided.")
        return {}

    try:
        data = json.loads(raw)

        # If already a dict with expected keys, keep it
        if isinstance(data, dict):
            configs = data
        # CameraManager passes a list of camera dicts → map by name
        elif isinstance(data, list):
            configs = {c["name"]: c for c in data if isinstance(c, dict) and "name" in c}
        else:
            raise ValueError(f"Unsupported cameras_json type: {type(data)}")

        # Build h264_network_cameras block if missing (but do NOT mutate fields)
        if "h264_network_cameras" not in configs:
            h264 = {
                name: cfg
                for name, cfg in configs.items()
                if isinstance(cfg, dict) and cfg.get("type") == "h264_network"
            }
            if h264:
                configs["h264_network_cameras"] = h264

        return configs
    except Exception as e:
        node.get_logger().error(f"Failed to parse cameras_json: {e}")
        return {}


def find_camera_topics(topics):
    
    candidates = {}
    for name, types in topics:
        if not name.endswith("/image_raw"):
            continue
        if "sensor_msgs/msg/Image" not in types:
            continue

        root = name.strip("/").split("/")[0]

        prev = candidates.get(root)
        if prev is None or ("/rgb/" in name and "/rgb/" not in prev):
            candidates[root] = name

    return candidates


def make_camera_status_json(active_cameras, encoder_info, registered_cameras=None):
    return json.dumps({
        "available_cameras": active_cameras,
        "registered_cameras": registered_cameras or active_cameras,
        "encoder": encoder_info,
    })
