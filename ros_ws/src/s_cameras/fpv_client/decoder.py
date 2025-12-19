#!/usr/bin/env python3
import sys
import subprocess
import threading
import time
import re
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class FPVDecoder(Node):
    def __init__(self, topic_names):
        super().__init__("fpv_viewer_client")

        self.topic_names = topic_names
        self.running = True

        self.ffmpegs = {}
        self.resolutions = {}
        self.frames = {}
        self.frames_lock = threading.Lock()

        self.first_packet_ts = {t: None for t in topic_names}
        self.res_lock = threading.Lock()

        self.stats = {
            t: {
                "bytes": 0,
                "mbit_s": 0.0,
                "avg_mbps": 0.0,
                "fps": 0.0,
                "last_ts": None,
                "avg_window": [],
            }
            for t in topic_names
        }
        self.stats_lock = threading.Lock()

        for topic in topic_names:
            self.resolutions[topic] = None
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, t=topic: self.callback(msg, t),
                10,
            )
            self.get_logger().info(f"Listening on {topic}")

        threading.Thread(target=self._display_loop, daemon=True).start()
        threading.Thread(target=self._stats_thread, daemon=True).start()

    # ------------------------------------------------------------------
    def _parse_resolution_from_format(self, fmt: str):
        if not fmt:
            return None
        m_w = re.search(r"width\s*=\s*(\d+)", fmt)
        m_h = re.search(r"height\s*=\s*(\d+)", fmt)
        if m_w and m_h:
            w, h = int(m_w.group(1)), int(m_h.group(1))
            if 64 <= w <= 8192 and 64 <= h <= 8192:
                return (w, h)
        return None

    # ------------------------------------------------------------------
    def _start_ffmpeg(self, topic):
        self.get_logger().info(f"{topic}: starting ffmpeg")

        proc = subprocess.Popen(
            [
                "ffmpeg",
                "-hide_banner",
                "-loglevel", "info",

                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-max_delay", "0",

                "-f", "mpegts",
                "-i", "pipe:0",

                "-pix_fmt", "bgr24",
                "-f", "rawvideo",
                "pipe:1",
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

        self.ffmpegs[topic] = proc

        threading.Thread(target=self._stderr_reader, args=(topic, proc), daemon=True).start()
        threading.Thread(target=self._frame_reader, args=(topic, proc), daemon=True).start()

    # ------------------------------------------------------------------
    def callback(self, msg: CompressedImage, topic: str):
        # 🔑 FIRST: parse resolution from msg.format
        if self.resolutions[topic] is None:
            res = self._parse_resolution_from_format(msg.format)
            if res:
                with self.res_lock:
                    self.resolutions[topic] = res
                self.get_logger().info(f"{topic}: resolution from format {res[0]}x{res[1]}")

        proc = self.ffmpegs.get(topic)
        if proc is None or proc.poll() is not None:
            self._start_ffmpeg(topic)
            proc = self.ffmpegs[topic]

        if self.first_packet_ts[topic] is None:
            self.first_packet_ts[topic] = time.time()

        try:
            proc.stdin.write(msg.data)
            with self.stats_lock:
                self.stats[topic]["bytes"] += len(msg.data)
        except Exception:
            return

        try:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception:
            t = time.time()

        with self.stats_lock:
            s = self.stats[topic]
            last = s["last_ts"]
            s["last_ts"] = t

            if last is not None:
                dt = t - last
                if 0.001 < dt < 1.0:
                    fps = 1.0 / dt
                    if fps < 240:
                        s["avg_window"].append(fps)
                        if len(s["avg_window"]) > 20:
                            s["avg_window"].pop(0)
                        s["fps"] = sum(s["avg_window"]) / len(s["avg_window"])

    # ------------------------------------------------------------------
    def _stderr_reader(self, topic, proc):
        rx = re.compile(r"(?<!\d)(\d{2,5})x(\d{2,5})(?!\d)")
        while self.running and proc.poll() is None:
            line = proc.stderr.readline()
            if not line:
                break
            m = rx.search(line.decode(errors="ignore"))
            if m and self.resolutions[topic] is None:
                self.resolutions[topic] = (int(m.group(1)), int(m.group(2)))
                self.get_logger().info(
                    f"{topic}: resolution from ffmpeg {self.resolutions[topic][0]}x{self.resolutions[topic][1]}"
                )

    # ------------------------------------------------------------------
    def _frame_reader(self, topic, proc):
        pending = bytearray()
        current_res = None
        frame_size = None

        while self.running and proc.poll() is None:
            with self.res_lock:
                res = self.resolutions[topic]
            if res is None:
                time.sleep(0.01)
                continue

            if res != current_res:
                current_res = res
                frame_size = res[0] * res[1] * 3
                pending.clear()

            chunk = proc.stdout.read(65536)
            if not chunk:
                time.sleep(0.002)
                continue

            pending.extend(chunk)
            if len(pending) > frame_size * 2:
                pending = pending[-frame_size:]

            newest = None
            while len(pending) >= frame_size:
                raw = pending[:frame_size]
                del pending[:frame_size]
                newest = np.frombuffer(raw, np.uint8).reshape((res[1], res[0], 3))

            if newest is not None:
                with self.frames_lock:
                    self.frames[topic] = newest

    # ------------------------------------------------------------------
    def _stats_thread(self):
        last_bytes = {t: 0 for t in self.topic_names}
        last_time = time.time()
        alpha = 0.2

        while rclpy.ok() and self.running:
            time.sleep(1.0)
            now = time.time()
            elapsed = now - last_time
            last_time = now
            if elapsed <= 0:
                continue

            with self.stats_lock:
                for t in self.topic_names:
                    s = self.stats[t]
                    cur = s["bytes"]
                    mbps = (cur - last_bytes[t]) * 8 / (1_000_000 * elapsed)
                    last_bytes[t] = cur
                    s["avg_mbps"] = (1 - alpha) * s["avg_mbps"] + alpha * mbps
                    s["mbit_s"] = s["avg_mbps"]

    # ------------------------------------------------------------------
    def _display_loop(self):
        cv2.namedWindow("FPV MultiView", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("FPV MultiView", 1280, 720)

        while self.running:
            with self.frames_lock:
                items = list(self.frames.items())
            if not items:
                time.sleep(0.02)
                continue

            frames, names = zip(*[(k, v) for k, v in items if v is not None])
            target_h = min(f.shape[0] for f in names)

            imgs = [
                cv2.resize(v, (int(v.shape[1] * target_h / v.shape[0]), target_h))
                for v in names
            ]
            combined = imgs[0] if len(imgs) == 1 else np.hstack(imgs)

            y = 30
            with self.stats_lock:
                for t in frames:
                    s = self.stats[t]
                    cv2.putText(
                        combined,
                        f"{t}: {s['mbit_s']:.2f} Mbps {s['fps']:.1f} FPS",
                        (10, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                    y += 30

            cv2.imshow("FPV MultiView", combined)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.running = False
                break

        cv2.destroyAllWindows()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.running = False
        for p in self.ffmpegs.values():
            try:
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
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
