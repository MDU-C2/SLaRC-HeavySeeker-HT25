# utils/naming_utils.py
import re
import copy
import logging

from .camera_config import (
    DEFAULT_OAK_PARAMS,
    DEFAULT_USB_PARAMS,
)

logger = logging.getLogger("NamingUtils")


def assign_names(
    cameras,
    config,
    prefix: str,
    id_attr: str = "id",
    port_attr: str = "port_path",
    max_slots: int = 10,
):

    configured = config or {}
    name_slots = [f"{prefix}{i}" for i in range(max_slots)]
    used_names = set()
    assigned = []


    # Split known (from config) vs unknown
    known, unknown = [], []
    for cam in cameras:
        cam_id = str(getattr(cam, id_attr))

        if cam_id in configured:
            known.append(cam)
        else:
            unknown.append(cam)


    # Assign known cameras (use config)
    for cam in known:
        cam_id = getattr(cam, id_attr)
        cfg = configured.get(cam_id, {})

        # Use configured name, else a fallback
        name = cfg.get("name", f"{prefix}_unknown_{cam_id}")

        cam.assigned_name = name
        cam.params = cfg.get("params", {})

        used_names.add(name)
        assigned.append(cam)

    # Sort unknown cameras by port


    def port_sort_key(cam):

        port_str = str(getattr(cam, port_attr, ""))
        numbers = re.findall(r"\d+", port_str)
        if numbers:
            return tuple(map(int, numbers))
        else:
            return (9999,)

    unknown.sort(key=port_sort_key)

    # Remaining available name slots
    available = [n for n in name_slots if n not in used_names]

    # Assign names to unknown cameras
    for cam in unknown:
        name = available.pop(0) if available else f"{prefix}_extra_{getattr(cam, id_attr)}"
        cam.assigned_name = name

        # Deep copy defaults (OAK vs USB)
        cam.params = copy.deepcopy(
            DEFAULT_OAK_PARAMS if prefix == "oak" else DEFAULT_USB_PARAMS
        )

        assigned.append(cam)

    return assigned
