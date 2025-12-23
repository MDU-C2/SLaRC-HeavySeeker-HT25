#!/usr/bin/env python3

import subprocess
import threading
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from foxglove_msgs.msg import CompressedVideo


class CameraEncoder:

    def __init__(self, node, camera_name, encoder_info, fps=30,output_topic=None, input_topic=None, output_mode="mpegts"):

        self.node = node
        self.camera_name = camera_name
        self.encoder_info = encoder_info
        self.fps = fps
        self.target_width  = 640
        self.target_height = 480

        codec = self.encoder_info.get("codec", "h264").lower()
        # Base MPEG-TS topic (always the same regardless of mode)
        self.output_topic = output_topic or f"/{camera_name}/encoded/{codec}"

        # Input topic (e.g. /camera0/image_raw)
        self.input_topic = input_topic or f"/{camera_name}/rgb/image_raw"

        self.bridge = CvBridge()
        self.running = False

        # Current output mode: "mpegts", "foxglove", "headless"
        self.output_mode = output_mode.lower()

        # FFmpeg processes
        self.process_ts = None          # MPEG-TS encoder
        self.process_foxglove = None    # TS -> Annex-B (copy + bsf)

        self.sub = None

        # Publisher for MPEG-TS
        #self.pub_ts = self.node.create_publisher(CompressedImage, self.output_topic, 10)

        # Publisher for Foxglove Annex-B (fixed suffix)
        self.foxglove_topic = self.output_topic + "/foxglove"
        #self.pub_foxglove = self.node.create_publisher(CompressedVideo, self.foxglove_topic, 10)
        self.pub_ts = None
        self.pub_foxglove = None
        
        self._first_frame_size = None
        self._last_input_stamp = None
        
        self.node.get_logger().info(
            f"[{self.camera_name}] Using encoder: {self.encoder_info['description']} "
            f"({self.encoder_info['name']}) [{self.encoder_info['codec']}]"
        )

        self.node.get_logger().info(
            f"[{self.camera_name}] Output topic (MPEG-TS): {self.output_topic}"
        )

        # Only show Foxglove topic when mode == 'foxglove'
        if self.output_mode == "foxglove":
            self.node.get_logger().info(
                f"[{self.camera_name}] Output topic (Foxglove H264): {self.foxglove_topic}"
            )
        else:
            self.node.get_logger().info(
                f"[{self.camera_name}] Foxglove output disabled (mode='{self.output_mode}')"
            )

    # ------------------------------------------------------------------
    def apply_output_mode(self, mode: str):
        mode = mode.lower()
        if mode not in ("mpegts", "foxglove", "headless"):
            self.node.get_logger().warn(
                f"[{self.camera_name}] Unknown output mode '{mode}', keeping '{self.output_mode}'"
            )
            return

        self.output_mode = mode
        self.node.get_logger().info(f"[{self.camera_name}] Switched output mode to: {self.output_mode}")

        # Destroy old pubs
        if self.pub_ts:
            self.node.destroy_publisher(self.pub_ts)
            self.pub_ts = None
        if self.pub_foxglove:
            self.node.destroy_publisher(self.pub_foxglove)
            self.pub_foxglove = None

        # Create only the ones needed
        if mode in ("mpegts", "headless"):
            self.pub_ts = self.node.create_publisher(CompressedImage, self.output_topic, 10)

        if mode == "foxglove":
            self.pub_foxglove = self.node.create_publisher(CompressedVideo, self.foxglove_topic, 10)


    # ------------------------------------------------------------------
    def start(self):
        topic = self.input_topic
        self.sub = self.node.create_subscription(Image, topic, self._callback, 10)
        self.node.get_logger().info(
            f"[{self.camera_name}] Waiting for first frame on {topic} to start encoder..."
        )

    # ------------------------------------------------------------------
    def _callback(self, msg: Image):

        # -------------------------------------------------------------
        # First frame: launch pipelines
        # -------------------------------------------------------------
        if not self.running:
            orig_w, orig_h = msg.width, msg.height

            # Determine actual encode resolution (never upscale)
            if orig_w > self.target_width or orig_h > self.target_height:
                enc_w, enc_h = self.target_width, self.target_height
            else:
                enc_w, enc_h = orig_w, orig_h

            # Save final encoder resolution
            self._first_frame_size = (enc_w, enc_h)

            # Launch ffmpeg with the resized resolution
            self._launch_ffmpeg_pipelines()

            # Validate processes
            if not self.process_ts or self.process_ts.poll() is not None:
                self.node.get_logger().error(
                    f"[{self.camera_name}] TS encoder failed to start."
                )
                self.stop()
                return

            if self.output_mode == "foxglove":
                if not self.process_foxglove or self.process_foxglove.poll() is not None:
                    self.node.get_logger().error(
                        f"[{self.camera_name}] Foxglove converter failed to start."
                    )
                    self.stop()
                    return

            self.running = True
            self.node.get_logger().info(
                f"[{self.camera_name}] Encoder started "
                f"(raw={orig_w}x{orig_h} → enc={enc_w}x{enc_h}, {self.encoder_info['codec']})"
            )

        # -------------------------------------------------------------
        # Validate that processes are running
        # -------------------------------------------------------------
        if not self.process_ts or self.process_ts.poll() is not None:
            self.node.get_logger().error(
                f"[{self.camera_name}] TS encoder is not running anymore."
            )
            self.stop()
            return

        if self.output_mode == "foxglove":
            if not self.process_foxglove or self.process_foxglove.poll() is not None:
                self.node.get_logger().error(
                    f"[{self.camera_name}] Foxglove converter stopped unexpectedly."
                )
                self.stop()
                return

        # -------------------------------------------------------------
        # Convert → resize → encode
        # -------------------------------------------------------------
        try:
            self._last_input_stamp = msg.header.stamp
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            h, w = frame.shape[:2]
            enc_w, enc_h = self._first_frame_size

            # Resize if current frame size does not match what ffmpeg expects
            if (w, h) != (enc_w, enc_h):
                frame = cv2.resize(frame, (enc_w, enc_h), interpolation=cv2.INTER_AREA)
                self.node.get_logger().debug(
                    f"[{self.camera_name}] Resized {w}x{h} → {enc_w}x{enc_h}"
    )


            raw_bytes = frame.tobytes()
            self.process_ts.stdin.write(raw_bytes)

        except BrokenPipeError:
            self.node.get_logger().error(
                f"[{self.camera_name}] TS encoder pipe closed."
            )
            self.stop()

        except Exception as e:
            self.node.get_logger().error(
                f"[{self.camera_name}] Encoding error: {e}"
            )
            self.stop()


    # ------------------------------------------------------------------
    def _build_base_cmd(self, width, height):
        enc = self.encoder_info
        name = enc["name"]

        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",      # This is where raw frames come in
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

    def _build_fg_cmd(self):

        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-f", "mpegts",
            "-i", "pipe:0",
            "-an",
            "-c:v", "copy",
            "-bsf:v", "h264_mp4toannexb",
            "-fflags", "nobuffer",
            "-muxdelay", "0",
            "-muxpreload", "0",
            "-f", "h264",
            "pipe:1",
        ]
        return cmd

    # ------------------------------------------------------------------
    def _launch_ffmpeg_pipelines(self):
        width, height = self._first_frame_size
        enc = self.encoder_info

        # ---------- Always Needed: MPEG-TS encoder ----------
        cmd_ts = self._build_base_cmd(width, height)

        if enc.get("bitstream_filter"):
            cmd_ts += ["-bsf:v", enc["bitstream_filter"]]

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

        # ---------------------------------------------------------------------
        # Only start Foxglove converter if mode == "foxglove"
        # ---------------------------------------------------------------------
        if self.output_mode == "foxglove":
            cmd_fg = self._build_fg_cmd()
            self.node.get_logger().info(
                f"[{self.camera_name}] Launching FFmpeg (Foxglove H264 copy+bsf):\n  {' '.join(cmd_fg)}"
            )

            self.process_foxglove = subprocess.Popen(
                cmd_fg,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )

            # Foxglove readers only in foxglove
            threading.Thread(target=self._reader_loop_foxglove, daemon=True).start()
            threading.Thread(
                target=self._stderr_reader_loop,
                args=(self.process_foxglove, "FG"),
                daemon=True,
            ).start()
        else:
            self.process_foxglove = None

        # TS readers always run
        threading.Thread(target=self._reader_loop_ts, daemon=True).start()
        threading.Thread(
            target=self._stderr_reader_loop,
            args=(self.process_ts, "TS"),
            daemon=True,
        ).start()


    # ------------------------- MPEG-TS reader -------------------------
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

                # Build the MPEG-TS message
                msg = CompressedImage()
                msg.header.stamp = self._last_input_stamp or self.node.get_clock().now().to_msg()
                msg.header.frame_id = self.camera_name
                msg.format = (
                    f"video/{self.encoder_info['codec'].lower()};"
                    f"width={width};height={height};fps={self.fps}"
                )
                msg.data = chunk

                # ---------------------------------------------------------
                # Publish only if valid publisher exists AND mode allows it
                # ---------------------------------------------------------
                if self.output_mode in ("mpegts", "headless") and self.pub_ts:
                    self.pub_ts.publish(msg)

                # ---------------------------------------------------------
                # Feed TS → Foxglove ffmpeg (always required, mode independent)
                # ---------------------------------------------------------
                if self.process_foxglove and self.process_foxglove.poll() is None:
                    try:
                        if self.process_foxglove.stdin:
                            self.process_foxglove.stdin.write(chunk)
                    except BrokenPipeError:
                        self.node.get_logger().error(
                            f"[{self.camera_name}] Foxglove ffmpeg pipe closed while writing TS."
                        )
                        break

        except Exception as e:
            self.node.get_logger().error(f"[{self.camera_name}] Reader loop TS error: {e}")

    # ---------------------- Annex-B for Foxglove ----------------------
    def _reader_loop_foxglove(self):

        try:
            stdout = self.process_foxglove.stdout
            read_fn = getattr(stdout, "read1", stdout.read)
            chunk_size = 64 * 1024

            while self.process_foxglove and self.process_foxglove.poll() is None:
                chunk = read_fn(chunk_size)
                if not chunk:
                    continue

                # Build Foxglove compressed video message
                msg = CompressedVideo()
                msg.timestamp = self._last_input_stamp or self.node.get_clock().now().to_msg()
                msg.frame_id = self.camera_name
                msg.format = "h264"
                msg.data = chunk

                # ---------------------------------------------------------
                # Publish ONLY when in foxglove mode AND publisher exists
                # ---------------------------------------------------------
                if self.output_mode == "foxglove" and self.pub_foxglove:
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
        if not self.running:
            return False

        if not self.process_ts or self.process_ts.poll() is not None:
            return False

        # Foxglove mode requires 2 running processes
        if self.output_mode == "foxglove":
            return (
                self.process_foxglove and
                self.process_foxglove.poll() is None
            )

        # MPEGTS or headless: only TS required
        return True
