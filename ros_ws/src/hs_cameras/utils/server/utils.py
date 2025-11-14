import json
import re
import json

def load_camera_configs(node):
    """Load camera configuration JSON from ROS parameter 'cameras_json'."""
    node.declare_parameter("cameras_json", "")
    raw = node.get_parameter("cameras_json").value

    if not raw:
        node.get_logger().warning("No cameras_json parameter provided.")
        return {}

    try:
        data = json.loads(raw)
        return {c["name"]: c for c in data}
    except Exception as e:
        node.get_logger().error(f"Failed to parse cameras_json: {e}")
        return {}


def find_camera_topics(topics):
    """Return a mapping of camera_name -> best /image_raw topic."""
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


def make_camera_status_json(active_cameras, encoder_info):
    return json.dumps({
        "available_cameras": active_cameras,
        "encoder": encoder_info,
    })
