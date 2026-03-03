from .base import EncoderBackend

# Cameras outputing h264 we dont want to encode, only pass it to the next step.
class PassthroughEncoder(EncoderBackend):

    name = "passthrough"
    codec = "h264"
    hw_accel = False

    def start(self):
        pass

    def stop(self):
        pass

    def encode(self, data: bytes) -> bytes:
        return data
