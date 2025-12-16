# utils/usb_utils.py
import os
import glob
import re
import subprocess
import logging

logger = logging.getLogger("USBUtils")


def list_video_devices():

    devices = sorted(glob.glob("/dev/video*"))
    filtered = []

    for dev in devices:
        if any(skip in dev for skip in ["loop", "v4l-subdev"]):
            continue
        if os.path.exists(dev):
            filtered.append(dev)

    return filtered


def extract_camera_name(device):
    info = subprocess.run(
        ["v4l2-ctl", "--info", "-d", device],
        capture_output=True, text=True, timeout=2
    ).stdout

    match = re.search(r"Card type\s*:\s*(.+)", info)
    return match.group(1).strip() if match else "Unknown"


def extract_camera_serial(device):
    info = subprocess.run(
        ["v4l2-ctl", "--info", "-d", device],
        capture_output=True, text=True, timeout=2
    ).stdout

    serial = re.search(r"Serial(?: number| ID)?\s*:\s*(.+)", info, re.I)
    serial = serial.group(1).strip() if serial else "unknown"

    if serial.lower() in {"unknown", "n/a", "none", "0x0001", "0000", "00000000"}:
        serial = "unknown"

    return serial


def get_usb_port_path(device):
    try:
        path = subprocess.check_output(
            ["udevadm", "info", "-q", "path", "-n", device],
            text=True
        ).strip()
        match = re.search(r"usb\d+\/(\d+-\d+(\.\d+)*)", path)
        return match.group(1) if match else "unknown"
    except Exception:
        return "unknown"


def get_max_resolution(device):

    try:
        out = subprocess.run(
            ["v4l2-ctl", "--list-formats-ext", "-d", device],
            capture_output=True, text=True, timeout=2
        ).stdout
    except Exception:
        return (0, 0, False)

    matches = re.findall(r"Size: Discrete (\d+)x(\d+)", out)
    if not matches:
        return (0, 0, False)

    # highest pixel count
    w, h = max((int(w), int(h)) for w, h in matches)
    mjpeg = "mjpg" in out.lower()

    return (w, h, mjpeg)


def detect_usb_cameras():
    # Detect and deduplicate USB cameras
    devices = list_video_devices()
    raw = []

    # First pass: collect raw info
    for dev in devices:
        try:
            formats = subprocess.run(
                ["v4l2-ctl", "--list-formats-ext", "-d", dev],
                capture_output=True, text=True, timeout=3
            ).stdout

            if not re.search(r"\[0\]:.*\n\s+Size:", formats):
                continue

            raw.append({
                "device": dev,
                "name": extract_camera_name(dev),
                "serial": extract_camera_serial(dev),
                "port": get_usb_port_path(dev),
            })

        except subprocess.TimeoutExpired:
            logger.warning("Timeout checking %s", dev)

    # Group by port path for deduplication
    grouped = {}
    for cam in raw:
        grouped.setdefault(cam["port"], []).append(cam)

    # Pick best in each group
    unique = []
    for port, cams in grouped.items():
        if len(cams) == 1:
            chosen = cams[0]
        else:
            best = 0
            chosen = cams[0]

            for c in cams:
                w, h, mjpeg = get_max_resolution(c["device"])
                score = w*h * (1.5 if mjpeg else 1)

                if score > best:
                    best = score
                    chosen = c

        unique.append(chosen)

    logger.info("USB detection: %d unique cameras", len(unique))
    return unique
