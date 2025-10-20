#!/usr/bin/env python3
import os
import glob
import re
import subprocess
import logging


# ---------------------------------------------------------------------------
# Configure logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="[%(name)s] %(levelname)s: %(message)s"
)
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


class DetectCameras:
    """Detects USB and DepthAI (OAK) cameras connected to the system."""

    def __init__(self):
        self.usb_cameras = []
        self.oak_cameras = []

    # ---------- USB CAMERA DETECTION ----------
    def _detect_usb_cameras(self):
        logger.debug("Starting USB camera detection...")
        
        # Get all /dev/video devices,
        devices = sorted(glob.glob("/dev/video*"))
        camera_list = []

        for dev in devices:
            # Filter out virtual/loopback devices
            if "loop" in dev or "v4l-subdev" in dev:
                logger.debug("Skipping %s (virtual/sub-device).", dev)
                continue
            if not os.path.exists(dev):
                logger.debug("Skipping %s (path does not exist).", dev)
                continue

            try:
                # Verify that the device supports video capture
                result = subprocess.run(
                    ["v4l2-ctl", "--list-formats-ext", "-d", dev],
                    capture_output=True, text=True, timeout=3
                )
                if result.returncode != 0 or not re.search(r"\[0\]:.*\n\s+Size:", result.stdout):
                    logger.debug("%s does not support video capture.", dev)
                    continue

                # Gather camera info
                info_result = subprocess.run(
                    ["v4l2-ctl", "--info", "-d", dev],
                    capture_output=True, text=True, timeout=2
                )
                info_text = info_result.stdout

                # Get that name of the device (Normaly the field "Card type" includes the name of the device)
                name_match = re.search(r"Card type\s*:\s*(.+)", info_text)
                name = name_match.group(1).strip() if name_match else "Unknown"

                # Get the serial number for the device if available.
                serial_match = re.search(r"Serial(?: number| ID)?\s*:\s*(.+)", info_text, re.IGNORECASE)
                serial = serial_match.group(1).strip() if serial_match else None

                # Some devices set a dummy serial just to not leave the field empty, therefor we need to ignore it.                
                if not serial or serial.lower() in {"unknown", "n/a", "none"} or serial in {"0x0001", "0000", "00000000"}:
                    serial = "unknown"


                # Try to get the port path to the device. This will be used to sort the devices later if the devices is not in the config file.
                try:
                    path = subprocess.check_output(["udevadm", "info", "-q", "path", "-n", dev], text=True).strip()
                    path_match = re.search(r"usb\d+\/(\d+-\d+(\.\d+)*)", path)
                    port_path = path_match.group(1) if path_match else "unknown"
                except Exception as e:
                    # If you get here, you are probably screwed <3
                    logger.debug("Could not determine port path for %s: %s", dev, e)
                    port_path = "unknown"
                # Gather all information about the camera
                camera_info = {
                    "device": dev,
                    "id": serial,
                    "name": name,
                    "port_path": port_path,
                }
                logger.debug("Detected USB camera: %s", camera_info)
                # Add to the list
                camera_list.append(camera_info)
            
            # Handle errors
            except subprocess.TimeoutExpired:
                logger.warning("Timeout while querying %s", dev)
            except Exception as e:
                logger.error("Error checking %s: %s", dev, e)

        # Deduplicate cameras — prefer MJPG-capable or higher-res devices
        unique = {}

        def score_device(dev_path):
            """Return a score for device capability based on v4l2-ctl output."""
            try:
                out = subprocess.check_output(["v4l2-ctl", "--list-formats-ext", "-d", dev_path], text=True, timeout=2)
                score = 0
                if "MJPG" in out:
                    score += 2
                if re.search(r"Size:\s+Discrete\s+(\d+)x(\d+)", out):
                    max_res = max([int(w)*int(h) for w, h in re.findall(r"Size:\s+Discrete\s+(\d+)x(\d+)", out)])
                    if max_res > 640*480:
                        score += 1
                return score
            except Exception:
                return 0

        for cam in camera_list:
            key = (cam["port_path"], cam["id"])
            if key not in unique:
                unique[key] = cam
            else:
                current = unique[key]
                # Compare based on device capability
                if score_device(cam["device"]) > score_device(current["device"]):
                    unique[key] = cam

        camera_list = list(unique.values())


        logger.info("Detected %d USB cameras.", len(camera_list))
        return camera_list


    # ---------- DEPTHAI DEVICE DETECTION ----------
    def _detect_oak_cameras(self):
        
        # Only search for devices if DepthAi is installed
        if not DEPTHAI_AVAILABLE:
            logger.warning("DepthAI not installed — skipping DepthAI detection.")
            return []

        try:
            logger.debug("Querying DepthAI devices...")
            # Get information about all connected DepthAI devices
            devices = dai.Device.getAllAvailableDevices()
            depthai_list = []

            # Extract device ID
            for dev in devices:
                mxid = (
                    getattr(dev, "mxid", None)
                    or getattr(dev, "deviceId", None)
                    or getattr(dev, "getDeviceId", lambda: None)()
                )

                # When the device is unbooted, the name field corresponds to the port path and not the actual name of the device. 
                # Therefor we sett the name to unkown if the field is empty, or DepthAI device if non empty. 
                name = getattr(dev, "name", None)
                if not name:
                    name = "unknown"
                    # Try method fallback or set default
                    try:
                        name = dev.getName()
                    except Exception:
                        name = "unknown"
                else:
                    # The device name is the port path when not booted.
                    porth_path = name 
                    name = "DepthAI Device"
                # Get the state of the device
                state = str(getattr(dev, "state", "UNKNOWN"))
                
                # Gather all information about the device
                device_info = {
                    "mxid": mxid,
                    "name": name,
                    "state": state,
                    "port_path": porth_path,
                }
                logger.debug("Detected DepthAI device: %s", device_info)
                # Add to the list
                depthai_list.append(device_info)

            logger.info("Detected %d DepthAI devices.", len(depthai_list))
            return depthai_list
        # Handle errors. Maybe you disconnect the device durring scaning you little rascal
        except Exception as e:
            logger.error("Error detecting DepthAI devices: %s", e)
            return []


    # ---------- MAIN PUBLIC FUNCTION ----------
    def detect(self):
        logger.info("Starting full camera detection...")
        self.usb_cameras = self._detect_usb_cameras()
        self.oak_cameras = self._detect_oak_cameras()
        logger.info(
            "Detection complete — %d USB, %d DepthAI devices.",
            len(self.usb_cameras), len(self.oak_cameras)
        )
        return self.usb_cameras, self.oak_cameras


# ---------------------------------------------------------------------------
# Stand-alone for testing/debug
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    detector = DetectCameras()
    usb, oak = detector.detect()

    print("\n=== USB Cameras ===")
    if usb:
        for cam in usb:
            print(f"- Name: {cam['name']} | Port path: {cam['port_path']} | ID: {cam['id']} | Device: {cam['device']}")
    else:
        print("No USB cameras detected.")

    print("\n=== DepthAI Devices ===")
    if oak:
        for dev in oak:
            print(f"- Name: {dev['name']} | Port path: {dev['port_path']} | DeviceId: {dev['mxid']} | State: {dev['state']}")
    else:
        print("No DepthAI devices detected.")
