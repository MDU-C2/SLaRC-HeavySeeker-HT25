#!/usr/bin/env python3
import sys
import subprocess
import threading
import time
import numpy as np
import cv2
import rclpy
from pathlib import Path
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class FPVDecoder(Node):

    def __init__(self, topic_names):
        super().__init__("fpv_viewer_client")

        self.topic_names = topic_names
        self.ffmpegs = {}
        self.frames = {}
        self.running = True
        self.resolutions = {}
        self.input_buffer = {t: b"" for t in topic_names}
        self.stats = {
            t: {"bytes": 0, "mbit_s": 0.0, "fps": 0.0, "last_ts": None, "avg_window": []}
            for t in topic_names
        }


        # Subscribe to each encoded stream
        for topic in topic_names:

            self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self.callback(msg, t), 10
            )
            self.get_logger().info(f"Listening on {topic}")

        # Launch display and bitrate monitor threads
        threading.Thread(target=self._display_loop, daemon=True).start()
        threading.Thread(target=self._stats_thread, daemon=True).start()

    # ------------------------------------------------------------------
    def _start_ffmpeg(self, topic, codec):
        """Spawn ffmpeg decoder process for one topic."""
        proc = subprocess.Popen([
            "ffmpeg",
            "-hide_banner", "-loglevel", "error",
            "-fflags", "nobuffer+discardcorrupt",
            "-flags", "low_delay",
            "-probesize", "32",
            "-analyzeduration", "0",
            "-f", "mpegts", "-i", "pipe:0",
            "-map", "0:v:0",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo", "-"
        ], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)

        self.ffmpegs[topic] = proc
        threading.Thread(target=self._frame_reader, args=(topic,), daemon=True).start()

    # ------------------------------------------------------------------
    def _frame_reader(self, topic):
        """Continuously read decoded frames for one topic."""
        w, h = self.resolutions.get(topic, (640, 480))
        frame_size = w * h * 3
        proc = self.ffmpegs[topic]
        buf = b""

        while self.running and proc.poll() is None:
            chunk = proc.stdout.read(frame_size)
            if not chunk:
                continue
            buf += chunk

            while len(buf) >= frame_size:
                raw = buf[:frame_size]
                buf = buf[frame_size:]
                frame = np.frombuffer(raw, np.uint8).reshape((h, w, 3))
                self.frames[topic] = frame

        proc.stdout.close()

    # ------------------------------------------------------------------
    def callback(self, msg: CompressedImage, topic: str):
        """Feed encoded bytes into per-topic ffmpeg and compute camera FPS."""
        fmt = msg.format.lower()

        # Detect codec
        if "h265" in fmt or "hevc" in fmt:
            codec = "h265"
        elif "h264" in fmt or "avc" in fmt:
            codec = "h264"
        else:
            self.get_logger().warning(f"{topic}: unknown codec in format='{msg.format}', assuming h264")
            codec = "h264"

        # Extract width/height metadata
        parts = dict(p.split("=", 1) for p in fmt.split(";") if "=" in p)
        if "width" not in parts or "height" not in parts:
            # Do NOT start ffmpeg until real metadata arrives
            return

        w = int(parts["width"])
        h = int(parts["height"])

        # --- FIX: ensure only ONE ffmpeg per topic ---
        proc = self.ffmpegs.get(topic)
        needs_restart = proc is None or proc.poll() is not None

        if needs_restart:
            self.resolutions[topic] = (w, h)
            self._start_ffmpeg(topic, codec)
            self.get_logger().info(
                f"\n====== New or restarted stream ======\n"
                f"   • Topic: {topic}\n"
                f"   • Codec: {codec.upper()}\n"
                f"   • Resolution: {w}x{h}\n"
                f"=====================================\n"
            )

        # --- Use real camera timestamps for FPS tracking ---
        try:
            stamp = msg.header.stamp
            t_sec = stamp.sec + stamp.nanosec * 1e-9
        except Exception:
            t_sec = time.time()

        s = self.stats[topic]
        last = s["last_ts"]
        s["last_ts"] = t_sec

        if last is not None:
            dt = t_sec - last
            if 0.001 < dt < 1.0:
                fps = 1.0 / dt
                if fps < 240:  # ignore spikes
                    s["avg_window"].append(fps)
                    if len(s["avg_window"]) > 20:
                        s["avg_window"].pop(0)
                    s["fps"] = sum(s["avg_window"]) / len(s["avg_window"])

        # --- Feed encoded bytes to ffmpeg ---
        try:
            proc = self.ffmpegs[topic]
            if proc and proc.stdin:
                data = bytes(msg.data)
                self.input_buffer[topic] += data

                if len(self.input_buffer[topic]) > 65536:
                    proc.stdin.write(self.input_buffer[topic])
                    self.input_buffer[topic] = b""

                s["bytes"] += len(data)
        except BrokenPipeError:
            self.get_logger().error(f"{topic}: ffmpeg pipe closed")
        except Exception as e:
            self.get_logger().error(f"{topic}: write error: {e}")


    # ------------------------------------------------------------------
    def _stats_thread(self):

        last_bytes = {t: 0 for t in self.topic_names}
        last_time = time.time()
        alpha = 0.2  # smoothing factor

        while rclpy.ok() and self.running:
            time.sleep(1.0)
            now = time.time()
            elapsed = now - last_time
            last_time = now
            if elapsed <= 0:
                continue

            for t in self.topic_names:
                s = self.stats[t]
                cur = s["bytes"]
                mbps = (cur - last_bytes[t]) * 8 / (1_000_000 * elapsed)
                last_bytes[t] = cur
                s["avg_mbps"] = s.get("avg_mbps", mbps)
                s["avg_mbps"] = (1 - alpha) * s["avg_mbps"] + alpha * mbps
                s["mbit_s"] = s["avg_mbps"]

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    def _display_loop(self):
        
        cv2.namedWindow("FPV MultiView", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("FPV MultiView", 1280, 720)

        while self.running:
            if not self.frames:
                time.sleep(0.05)
                continue

            cams = list(self.frames.keys())
            frames = [self.frames[c] for c in cams if c in self.frames]
            if not frames:
                continue

            # Normalize frame height
            h = min(f.shape[0] for f in frames)
            frames = [cv2.resize(f, (int(f.shape[1] * h / f.shape[0]), h)) for f in frames]

            num = len(frames)

            if num == 1:
                combined = frames[0]

            elif num == 2:
                combined = np.hstack(frames)

            elif num == 3:
                # top row: first 2
                top = np.hstack(frames[:2])
                # bottom row: single, padded to same width
                bottom = frames[2]
                pad_w = top.shape[1] - bottom.shape[1]
                if pad_w > 0:
                    pad = np.zeros((bottom.shape[0], pad_w, 3), np.uint8)
                    bottom = np.hstack((bottom, pad))
                combined = np.vstack((top, bottom))

            else:  # 4 or more → 2-column grid
                combined = self._grid(frames, cols=2)

            # Draw stats overlays
            y = 30
            for c in cams:
                s = self.stats[c]
                text = f"{c}: {s['mbit_s']:.2f} Mbps  {s['fps']:.1f} FPS"
                cv2.putText(combined, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 255, 0), 2, cv2.LINE_AA)
                y += 30

            cv2.imshow("FPV MultiView", combined)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.running = False
                break

    cv2.destroyAllWindows()


    # ------------------------------------------------------------------
    def _grid(self, frames, cols=2):
        """Arrange frames in a simple grid."""
        rows = []
        for i in range(0, len(frames), cols):
            row = np.hstack(frames[i:i + cols])
            rows.append(row)
        max_w = max(r.shape[1] for r in rows)
        for i, r in enumerate(rows):
            if r.shape[1] < max_w:
                pad = np.zeros((r.shape[0], max_w - r.shape[1], 3), np.uint8)
                rows[i] = np.hstack((r, pad))
        return np.vstack(rows)

    # ------------------------------------------------------------------
    def destroy_node(self):
        """Stop all decoder processes cleanly."""
        self.running = False
        for p in self.ffmpegs.values():
            try:
                if p.stdin:
                    p.stdin.close()
                p.terminate()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    topics = sys.argv[1:] or ["/camera0/encoded"]
    node = FPVDecoder(topics)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Multi decoder stopped")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
