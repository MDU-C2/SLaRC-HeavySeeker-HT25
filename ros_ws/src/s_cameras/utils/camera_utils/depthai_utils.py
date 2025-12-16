# utils/depthai_utils.py
import logging

logger = logging.getLogger("DepthAIUtils")

try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False
    logger.warning("DepthAI not installed — skipping detection.")


def detect_depthai_devices():
    """Return list of dicts describing DepthAI devices."""
    if not DEPTHAI_AVAILABLE:
        return []

    devices = dai.Device.getAllAvailableDevices()
    results = []

    for dev in devices:
        mxid = getattr(dev, "mxid", None) or getattr(dev, "deviceId", None)
        name = getattr(dev, "name", None) or "DepthAI Device"
        state = getattr(dev, "state", "UNKNOWN")

        results.append({
            "mxid": mxid,
            "name": name,
            "state": str(state),
            "port": name if "usb" in name.lower() else "unknown",
        })

    logger.info("DepthAI detection: %d devices", len(results))
    return results
