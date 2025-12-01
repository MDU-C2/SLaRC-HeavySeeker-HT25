
import logging
from utils.camera_utils.usb_utils import detect_usb_cameras
from utils.camera_utils.depthai_utils import detect_depthai_devices



logger = logging.getLogger("DetectCameras")


class BaseCamera:
    def __init__(self, name, camera_id, port_path, device=None):
        self.name = name
        self.id = camera_id
        self.port_path = port_path
        self.device = device


class USBCamera(BaseCamera):
    pass


class DepthAICamera(BaseCamera):
    def __init__(self, name, camera_id, port_path, state):
        super().__init__(name, camera_id, port_path)
        self.state = state


class DetectCameras:
    def detect(self):
        logger.info("Detecting all cameras...")

        usb_entries = detect_usb_cameras()
        oak_entries = detect_depthai_devices()

        cameras = []

        # Wrap USB cameras
        for c in usb_entries:
            cameras.append(
                USBCamera(
                    name=c["name"],
                    camera_id=c["serial"],
                    port_path=c["port"],
                    device=c["device"]
                )
            )

        # Wrap DepthAI cameras
        for c in oak_entries:
            cameras.append(
                DepthAICamera(
                    name=c["name"],
                    camera_id=c["mxid"],
                    port_path=c["port"],
                    state=c["state"]
                )
            )

        logger.info("Camera detection complete: %d cameras", len(cameras))
        return cameras
