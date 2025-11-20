
import os
import signal
import subprocess
from typing import List


def log(msg: str):
    print(f"[Viewer] {msg}", flush=True)

class DecoderProcess:
    def __init__(self):
        self.proc: subprocess.Popen | None = None
        self.active_topics: List[str] = []

    def stop(self):
        """Stop the decoder subprocess if running."""
        if not self.proc or self.proc.poll() is not None:
            return
        log("Stopping decoder window")
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
        except Exception:
            try:
                self.proc.terminate()
            except Exception:
                pass
        self.proc = None
        self.active_topics = []

    def launch(self, topics: List[str]):
        """Start or restart the decoder for the given topic list."""
        if not topics:
            self.stop()
            return

        topics = sorted(set(topics))

        # Always restart if topics changed or process not running
        process_alive = self.proc and self.proc.poll() is None
        if process_alive and topics == self.active_topics:
            return  # already running same set

        # Stop any old process first
        if process_alive:
            log("Updating decoder view")
            self.stop()
        else:
            log("Launching decoder window")

        # Launch new decoder process
        cmd = ["ros2", "run", "s_cameras", "decoder"] + topics
        log(f"Decoder command: {' '.join(cmd)}")

        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            self.active_topics = topics
        except Exception as e:
            log(f"Failed to start decoder: {e}")
            self.proc = None
            self.active_topics = []