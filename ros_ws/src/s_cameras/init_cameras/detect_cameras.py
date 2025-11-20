#!/usr/bin/env python3
import os
import glob
import re
import subprocess
import logging

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="[%(name)s] %(levelname)s: %(message)s")
logger = logging.getLogger("DetectCameras")

# ---------------------------------------------------------------------------
# Optional import of DepthAI
# ---------------------------------------------------------------------------
try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False
    logger.warning("DepthAI not installed — OAK/DepthAI detection disabled.")


# ---------------------------------------------------------------------------
# Base camera classes
# ---------------------------------------------------------------------------
class BaseCamera:
    """Base class for any detected camera."""

    def __init__(self, name, camera_id, port_path, device=None):
        self.name = name
        self.id = camera_id
        self.port_path = port_path
        self.device = device

    def __repr__(self):
        return f"{self.__class__.__name__}(name={self.name}, id={self.id}, port_path={self.port_path})"


class USBCamera(BaseCamera):
    """Represents a USB camera detected via /dev/video*"""

    def __init__(self, name, camera_id, port_path, device):
        super().__init__(name, camera_id, port_path, device)

    @staticmethod
    def detect_all():
        """Detects all USB cameras available on the system (deduplicated per USB port)."""
        logger.debug("Scanning for USB cameras...")
        devices = sorted(glob.glob("/dev/video*"))
        raw_cameras = []

        for dev in devices:
            if "loop" in dev or "v4l-subdev" in dev or not os.path.exists(dev):
                continue

            try:
                # Check if device supports capture
                result = subprocess.run(
                    ["v4l2-ctl", "--list-formats-ext", "-d", dev],
                    capture_output=True, text=True, timeout=3
                )
                if result.returncode != 0 or not re.search(r"\[0\]:.*\n\s+Size:", result.stdout):
                    continue

                # Extract metadata
                info = subprocess.run(
                    ["v4l2-ctl", "--info", "-d", dev],
                    capture_output=True, text=True, timeout=2
                ).stdout

                name = re.search(r"Card type\s*:\s*(.+)", info)
                name = name.group(1).strip() if name else "Unknown"

                serial = re.search(r"Serial(?: number| ID)?\s*:\s*(.+)", info, re.I)
                serial = serial.group(1).strip() if serial else "unknown"
                if serial.lower() in {"unknown", "n/a", "none", "0x0001", "0000", "00000000"}:
                    serial = "unknown"

                # Determine port path
                try:
                    path = subprocess.check_output(
                        ["udevadm", "info", "-q", "path", "-n", dev], text=True
                    ).strip()
                    match = re.search(r"usb\d+\/(\d+-\d+(\.\d+)*)", path)
                    port_path = match.group(1) if match else "unknown"
                except Exception:
                    port_path = "unknown"

                raw_cameras.append({
                    "name": name,
                    "serial": serial,
                    "port_path": port_path,
                    "device": dev,
                })

            except subprocess.TimeoutExpired:
                logger.warning("Timeout while querying %s", dev)
            except Exception as e:
                logger.error("Error checking %s: %s", dev, e)

        # ---------------------------------------------------------------------
        # Deduplicate cameras per USB port (pick best /dev/video* per device)
        # ---------------------------------------------------------------------
        grouped = {}
        for cam in raw_cameras:
            port = cam["port_path"]
            grouped.setdefault(port, []).append(cam)

        unique_cameras = []
        for port, cams in grouped.items():
            if len(cams) == 1:
                chosen = cams[0]
            else:
                # Pick device with best (highest) resolution, preferring MJPG
                best_pixels = 0
                chosen = cams[0]
                for c in cams:
                    try:
                        res_out = subprocess.run(
                            ["v4l2-ctl", "--list-formats-ext", "-d", c["device"]],
                            capture_output=True, text=True, timeout=2
                        ).stdout

                        # Find highest resolution in output
                        matches = re.findall(r"Size: Discrete (\d+)x(\d+)", res_out)
                        if matches:
                            w, h = map(int, matches[0])
                            pixels = w * h
                            if "MJPG" in res_out or "MJPEG" in res_out:
                                pixels *= 1.5  # prefer MJPG
                            if pixels > best_pixels:
                                best_pixels = pixels
                                chosen = c
                    except Exception:
                        continue

            unique_cameras.append(
                USBCamera(
                    name=chosen["name"],
                    camera_id=chosen["serial"],
                    port_path=chosen["port_path"],
                    device=chosen["device"],
                )
            )

        logger.info("Detected %d USB cameras (deduplicated).", len(unique_cameras))
        return unique_cameras


class DepthAICamera(BaseCamera):
    """Represents a DepthAI/OAK camera device."""

    def __init__(self, name, mxid, port_path, state):
        super().__init__(name, mxid, port_path)
        self.state = state

    @staticmethod
    def detect_all():
        """Detects all connected DepthAI devices."""
        if not DEPTHAI_AVAILABLE:
            logger.debug("DepthAI not installed — skipping.")
            return []

        try:
            devices = dai.Device.getAllAvailableDevices()
            cameras = []

            for dev in devices:
                mxid = getattr(dev, "mxid", None) or getattr(dev, "deviceId", None)
                name = getattr(dev, "name", None) or "DepthAI Device"
                state = str(getattr(dev, "state", "UNKNOWN"))

                # If not booted, name may be port path
                port_path = name if "usb" in name else "unknown"

                cameras.append(DepthAICamera(name, mxid, port_path, state))

            logger.info("Detected %d DepthAI devices.", len(cameras))
            return cameras

        except Exception as e:
            logger.error("Error detecting DepthAI devices: %s", e)
            return []


# ---------------------------------------------------------------------------
# Main detector class
# ---------------------------------------------------------------------------
class DetectCameras:
    """Camera manager that discovers all supported camera types."""

    DETECTORS = [USBCamera, DepthAICamera]

    def __init__(self):
        self.cameras = []

    def detect(self):
        """Run all registered detectors and aggregate results."""
        logger.info("Starting full camera detection...")
        self.cameras = []

        for cls in self.DETECTORS:
            try:
                found = cls.detect_all()
                self.cameras.extend(found)
            except Exception as e:
                logger.error("Error running detector %s: %s", cls.__name__, e)

        logger.info("Detection complete — total %d cameras.", len(self.cameras))
        return self.cameras

    def list_by_type(self, camera_type):
        return [c for c in self.cameras if isinstance(c, camera_type)]

    def summary(self):
        """Returns a short summary of all cameras."""
        for cam in self.cameras:
            yield f"{cam.__class__.__name__}: {cam.name} (id={cam.id}, port={cam.port_path})"


# ---------------------------------------------------------------------------
# Stand-alone test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    detector = DetectCameras()
    all_cams = detector.detect()

    print("\n=== Detected Cameras ===")
    if not all_cams:
        print("No cameras detected.")
    else:
        for cam in all_cams:
            print(f"- {cam}")
