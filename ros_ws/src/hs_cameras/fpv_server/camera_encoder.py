#!/usr/bin/env python3

import subprocess
import threading

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage


class CameraEncoder:

    def __init__(self, node, camera_name, encoder_info, fps=30, output_topic=None, input_topic=None):
        self.node = node
        self.camera_name = camera_name
        self.encoder_info = encoder_info
        self.fps = fps

        codec = self.encoder_info.get("codec", "h264").lower()
        self.output_topic = output_topic or f"/{camera_name}/encoded/{codec}"

        # ⬇ NEW: support explicit input topic (e.g. /oak1/oak1/rgb/image_raw)
        self.input_topic = input_topic or f"/{camera_name}/rgb/image_raw"

        self.bridge = CvBridge()
        self.running = False
        self.process = None
        self.sub = None

        # Publisher for encoded stream
        self.pub = self.node.create_publisher(CompressedImage, self.output_topic, 10)

        self._first_frame_size = None
        self._last_input_stamp = None

        self.node.get_logger().info(
            f"[{self.camera_name}] Using encoder: {self.encoder_info['description']} "
            f"({self.encoder_info['name']}) [{self.encoder_info['codec']}]"
        )
        self.node.get_logger().info(
            f"[{self.camera_name}] Output topic: {self.output_topic}"
        )

    # ------------------------------------------------------------------
    def start(self):
        topic = self.input_topic
        self.sub = self.node.create_subscription(Image, topic, self._callback, 10)
        self.node.get_logger().info(
            f"[{self.camera_name}] Waiting for first frame on {topic} to start encoder..."
        )

    # ------------------------------------------------------------------
    def _callback(self, msg: Image):
        if not self.running:
            self._first_frame_size = (msg.width, msg.height)
            self._launch_ffmpeg()

            if not self.process or self.process.poll() is not None:
                self.node.get_logger().error(
                    f"[{self.camera_name}] FFmpeg failed to start, not enabling encoder."
                )
                return

            self.running = True
            self.node.get_logger().info(
                f"[{self.camera_name}] Encoder started ({msg.width}x{msg.height}, "
                f"{self.encoder_info['codec']}) → {self.output_topic}"
            )

        try:
            if not self.process or self.process.poll() is not None:
                self.node.get_logger().error(
                    f"[{self.camera_name}] FFmpeg process is not running anymore."
                )
                self.stop()
                return

            # Save original capture timestamp
            self._last_input_stamp = msg.header.stamp

            # Convert to BGR24 and push raw bytes to ffmpeg
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process.stdin.write(frame.tobytes())

        except BrokenPipeError:
            self.node.get_logger().error(f"[{self.camera_name}] Encoder pipe closed.")
            self.stop()
        except Exception as e:
            self.node.get_logger().error(f"[{self.camera_name}] Encoding error: {e}")

    # ------------------------------------------------------------------
    def _launch_ffmpeg(self):
        width, height = self._first_frame_size
        enc = self.encoder_info
        name = enc["name"]

        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(self.fps),
            "-i", "pipe:0",
            "-an",
        ]

        # --- Special handling for VAAPI encoders ---
        if "vaapi" in name:
            cmd += [
                "-vaapi_device", "/dev/dri/renderD128",
                "-vf", "format=nv12,hwupload",
            ]

        # Encoder selection
        cmd += ["-c:v", name]

        # Encoder options
        for k, v in enc.get("options", {}).items():
            cmd += [f"-{k}", str(v)]

        # Extra args (bitrate, tune, etc.)
        cmd += enc.get("extra_args", [])

        # Optional bitstream filter (e.g. h264_mp4toannexb)
        if enc.get("bitstream_filter"):
            cmd += ["-bsf:v", enc["bitstream_filter"]]

        # Mux flags + format
        cmd += enc.get("mux_flags", [])
        cmd += ["-f", enc.get("mux", "mpegts"), "pipe:1"]

        self.node.get_logger().info(
            f"[{self.camera_name}] Launching FFmpeg:\n  {' '.join(cmd)}"
        )

        self.process = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

        threading.Thread(target=self._reader_loop, daemon=True).start()
        threading.Thread(target=self._stderr_reader_loop, daemon=True).start()

    # ------------------------------------------------------------------
    def _reader_loop(self):
        width, height = self._first_frame_size
        try:
            stdout = self.process.stdout
            read_fn = getattr(stdout, "read1", stdout.read)
            chunk_size = 188 * 100  # MPEG-TS packet multiple (~18 KB)

            while self.process and self.process.poll() is None:
                chunk = read_fn(chunk_size)
                if not chunk:
                    continue

                msg = CompressedImage()
                # Use the timestamp of the last camera frame that generated this chunk
                msg.header.stamp = self._last_input_stamp or self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.camera_name
                msg.format = (
                    f"video/{self.encoder_info['codec'].lower()};"
                    f"width={width};height={height};fps={self.fps}"
                )
                msg.data = chunk
                self.pub.publish(msg)

        except Exception as e:
            self.node.get_logger().error(f"[{self.camera_name}] Reader loop error: {e}")

    # ------------------------------------------------------------------
    def _stderr_reader_loop(self):
        try:
            while self.process and self.process.poll() is None:
                line = self.process.stderr.readline()
                if not line:
                    break
                text = line.decode("utf-8", errors="ignore").strip()
                if text:
                    self.node.get_logger().error(f"[{self.camera_name}] ffmpeg: {text}")
        except Exception as e:
            self.node.get_logger().error(
                f"[{self.camera_name}] ffmpeg stderr loop error: {e}"
            )

    # ------------------------------------------------------------------
    def stop(self):
        self.running = False
        if self.sub:
            self.node.destroy_subscription(self.sub)
            self.sub = None
        if self.process:
            try:
                if self.process.stdin:
                    self.process.stdin.close()
                self.process.terminate()
                self.process.wait(timeout=2)
            except Exception:
                pass
        self.node.get_logger().info(f"[{self.camera_name}] Encoder stopped.")

    def is_running(self):
        return self.running and self.process and self.process.poll() is None
