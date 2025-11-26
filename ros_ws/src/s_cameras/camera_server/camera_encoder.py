#!/usr/bin/env python3

import subprocess
import threading
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from foxglove_msgs.msg import CompressedVideo


class CameraEncoder:

    def __init__(self, node, camera_name, encoder_info, fps=30, output_topic=None, input_topic=None):
        self.node = node
        self.camera_name = camera_name
        self.encoder_info = encoder_info
        self.fps = fps

        codec = self.encoder_info.get("codec", "h264").lower()
        self.output_topic = output_topic or f"/{camera_name}/encoded/{codec}"

        # Input topic (e.g. /oak1/oak1/rgb/image_raw)
        self.input_topic = input_topic or f"/{camera_name}/rgb/image_raw"

        self.bridge = CvBridge()
        self.running = False

        # FFmpeg processes
        self.process_ts = None          # MPEG-TS - CompressedImage
        self.process_foxglove = None    # raw H264 Annex-B - CompressedVideo

        self.sub = None

        # Publisher for MPEG-TS
        self.pub_ts = self.node.create_publisher(CompressedImage, self.output_topic, 10)

        # Publisher for Foxglove Annex-B
        self.foxglove_topic = self.output_topic + "/foxglove"
        self.pub_foxglove = self.node.create_publisher(CompressedVideo, self.foxglove_topic, 10)

        self._first_frame_size = None
        self._last_input_stamp = None

        self.node.get_logger().info(
            f"[{self.camera_name}] Using encoder: {self.encoder_info['description']} "
            f"({self.encoder_info['name']}) [{self.encoder_info['codec']}]"
        )
        self.node.get_logger().info(
            f"[{self.camera_name}] Output topic (MPEG-TS): {self.output_topic}"
        )
        self.node.get_logger().info(
            f"[{self.camera_name}] Output topic (Foxglove H264): {self.foxglove_topic}"
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
            self._launch_ffmpeg_pipelines()

            if (
                not self.process_ts or self.process_ts.poll() is not None or
                not self.process_foxglove or self.process_foxglove.poll() is not None
            ):
                self.node.get_logger().error(
                    f"[{self.camera_name}] FFmpeg failed to start, not enabling encoder."
                )
                return

            self.running = True
            self.node.get_logger().info(
                f"[{self.camera_name}] Encoder started ({msg.width}x{msg.height}, "
                f"{self.encoder_info['codec']}) → {self.output_topic} & {self.foxglove_topic}"
            )

        try:
            # Make sure both processes are alive
            if (
                not self.process_ts or self.process_ts.poll() is not None or
                not self.process_foxglove or self.process_foxglove.poll() is not None
            ):
                self.node.get_logger().error(
                    f"[{self.camera_name}] One of the FFmpeg processes is not running anymore."
                )
                self.stop()
                return

            # Save original capture timestamp
            self._last_input_stamp = msg.header.stamp

            # Convert to BGR24 and push raw bytes to both ffmpeg instances (this might be a cpu user)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            raw_bytes = frame.tobytes()

            self.process_ts.stdin.write(raw_bytes)
            self.process_foxglove.stdin.write(raw_bytes)

        except BrokenPipeError:
            self.node.get_logger().error(f"[{self.camera_name}] Encoder pipe closed.")
            self.stop()
        except Exception as e:
            self.node.get_logger().error(f"[{self.camera_name}] Encoding error: {e}")

    # ------------------------------------------------------------------
    def _build_base_cmd(self, width, height):
        enc = self.encoder_info
        name = enc["name"]

        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",      #This might be a CPU user
            "-s", f"{width}x{height}",
            "-r", str(self.fps),
            "-i", "pipe:0",
            "-an",
        ]

        # Special handling for VAAPI encoders (The one used on NUC)
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

        return cmd

    # ------------------------------------------------------------------
    def _launch_ffmpeg_pipelines(self):
        width, height = self._first_frame_size
        enc = self.encoder_info

        # ---------- Pipeline A: MPEG-TS - sensor_msgs/CompressedImage ----------
        cmd_ts = self._build_base_cmd(width, height)

        # Optional bitstream filter (e.g. h264_mp4toannexb)
        if enc.get("bitstream_filter"):
            cmd_ts += ["-bsf:v", enc["bitstream_filter"]]

        # Mux flags + format (default mpegts)
        cmd_ts += enc.get("mux_flags", [])
        cmd_ts += ["-f", enc.get("mux", "mpegts"), "pipe:1"]

        self.node.get_logger().info(
            f"[{self.camera_name}] Launching FFmpeg (MPEG-TS):\n  {' '.join(cmd_ts)}"
        )

        self.process_ts = subprocess.Popen(
            cmd_ts,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

        threading.Thread(target=self._reader_loop_ts, daemon=True).start()
        threading.Thread(
            target=self._stderr_reader_loop,
            args=(self.process_ts, "TS"),
            daemon=True,
        ).start()

        # ---------- Pipeline B: Foxglove-compatible raw H264 Annex-B - foxglove_msgs/CompressedVideo ----------
        cmd_fg = self._build_base_cmd(width, height)
        cmd_fg += ["-bsf:v", "h264_mp4toannexb"]

        cmd_fg += [
            "-fflags", "nobuffer",
            "-muxdelay", "0",
            "-muxpreload", "0",
        ]

        # Raw H264 bitstream on stdout
        cmd_fg += ["-f", "h264", "pipe:1"]

        self.node.get_logger().info(
            f"[{self.camera_name}] Launching FFmpeg (Foxglove H264):\n  {' '.join(cmd_fg)}"
        )

        self.process_foxglove = subprocess.Popen(
            cmd_fg,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

        threading.Thread(target=self._reader_loop_foxglove, daemon=True).start()
        threading.Thread(
            target=self._stderr_reader_loop,
            args=(self.process_foxglove, "FG"),
            daemon=True,
        ).start()

    # -------------------------MPEG-TS non foxglove-----------------------------------------
    def _reader_loop_ts(self):
        width, height = self._first_frame_size
        try:
            stdout = self.process_ts.stdout
            read_fn = getattr(stdout, "read1", stdout.read)
            chunk_size = 188 * 100  # MPEG-TS packet multiple (~18 KB)

            while self.process_ts and self.process_ts.poll() is None:
                chunk = read_fn(chunk_size)
                if not chunk:
                    continue

                msg = CompressedImage()
                msg.header.stamp = self._last_input_stamp or self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.camera_name
                msg.format = (
                    f"video/{self.encoder_info['codec'].lower()};"
                    f"width={width};height={height};fps={self.fps}"
                )
                msg.data = chunk
                self.pub_ts.publish(msg)

        except Exception as e:
            self.node.get_logger().error(f"[{self.camera_name}] Reader loop TS error: {e}")

    # -------------------------------Annex-B for foxglove-----------------------------------
    def _reader_loop_foxglove(self):

        try:
            stdout = self.process_foxglove.stdout
            read_fn = getattr(stdout, "read1", stdout.read)
            chunk_size = 64 * 1024

            while self.process_foxglove and self.process_foxglove.poll() is None:
                chunk = read_fn(chunk_size)
                if not chunk:
                    continue

                msg = CompressedVideo()

                stamp = self._last_input_stamp or self.node.get_clock().now().to_msg()
                msg.timestamp = stamp

                msg.frame_id = self.camera_name

                msg.format = "h264"
                msg.data = chunk

                self.pub_foxglove.publish(msg)

        except Exception as e:
            self.node.get_logger().error(
                f"[{self.camera_name}] Reader loop Foxglove error: {e}"
            )



    # ------------------------------------------------------------------
    def _stderr_reader_loop(self, process, tag):
        try:
            while process and process.poll() is None:
                line = process.stderr.readline()
                if not line:
                    break
                text = line.decode("utf-8", errors="ignore").strip()
                if text:
                    self.node.get_logger().error(f"[{self.camera_name}] ffmpeg[{tag}]: {text}")
        except Exception as e:
            self.node.get_logger().error(
                f"[{self.camera_name}] ffmpeg stderr loop error [{tag}]: {e}"
            )

    # ------------------------------------------------------------------
    def stop(self):
        self.running = False
        if self.sub:
            self.node.destroy_subscription(self.sub)
            self.sub = None

        for proc, label in ((self.process_ts, "TS"), (self.process_foxglove, "FG")):
            if proc:
                try:
                    if proc.stdin:
                        proc.stdin.close()
                    proc.terminate()
                    proc.wait(timeout=2)
                except Exception:
                    pass
        self.process_ts = None
        self.process_foxglove = None

        self.node.get_logger().info(f"[{self.camera_name}] Encoder stopped.")

    def is_running(self):
        return (
            self.running and
            self.process_ts and self.process_ts.poll() is None and
            self.process_foxglove and self.process_foxglove.poll() is None
        )
