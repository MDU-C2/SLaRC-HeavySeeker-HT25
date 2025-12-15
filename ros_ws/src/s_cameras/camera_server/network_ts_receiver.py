#!/usr/bin/env python3

import subprocess
import threading

from sensor_msgs.msg import CompressedImage
from foxglove_msgs.msg import CompressedVideo


class NetworkTSReceiver:

    def __init__(
        self,
        node,
        camera_name,
        listen_url,
        fps=30,
        width=None,
        height=None,
        output_mode="mpegts",
        registry=None,
    ):
        self.node = node
        self.camera_name = camera_name
        self.listen_url = listen_url
        self.fps = fps
        self.width = width
        self.height = height
        self.output_mode = output_mode.lower()
        self.registry = registry

        self.output_topic = f"/{camera_name}/encoded/h264"
        self.foxglove_topic = self.output_topic + "/foxglove"

        self.pub_ts = None
        self.pub_foxglove = None

        self.process_ts = None
        self.process_foxglove = None

        self._stop = False
        self.publishing = False
        self._manual_pause = False

        self.node.get_logger().info(
            f"[{self.camera_name}] NetworkTSReceiver listening on {self.listen_url}"
        )

    # ------------------------------------------------------------
    def apply_output_mode(self, mode: str, force: bool = False):
        self.output_mode = mode.lower()

        if self.pub_ts:
            self.node.destroy_publisher(self.pub_ts)
            self.pub_ts = None

        if self.pub_foxglove:
            self.node.destroy_publisher(self.pub_foxglove)
            self.pub_foxglove = None

        # Do not recreate pubs while paused unless explicitly forced
        if not self.publishing and not force:
            return

        if self.output_mode in ("mpegts", "headless"):
            self.pub_ts = self.node.create_publisher(
                CompressedImage,
                self.output_topic,
                10,
            )

        if self.output_mode == "foxglove":
            self.pub_foxglove = self.node.create_publisher(
                CompressedVideo,
                self.foxglove_topic,
                10,
            )

    # ------------------------------------------------------------
    def pause_publishers(self, lock: bool = True):
        """Stop publishing topics but keep listening for heartbeats/data."""
        self._manual_pause = lock
        self.publishing = False
        if self.pub_ts:
            self.node.destroy_publisher(self.pub_ts)
            self.pub_ts = None
        if self.pub_foxglove:
            self.node.destroy_publisher(self.pub_foxglove)
            self.pub_foxglove = None
        self.node.get_logger().info(
            f"[{self.camera_name}] Publishing paused (still listening for data)"
        )

    def resume_publishers(self):
        """Resume publishing with the current output mode."""
        if self.publishing:
            return
        self._manual_pause = False
        self.publishing = True
        self.apply_output_mode(self.output_mode, force=True)
        self.node.get_logger().info(
            f"[{self.camera_name}] Publishing resumed"
        )

    @property
    def is_publishing(self) -> bool:
        return self.publishing

    @property
    def is_dead(self) -> bool:
        """True if the ffmpeg process has stopped or the receiver was stopped."""
        if self._stop:
            return True
        if self.process_ts is None:
            return True
        try:
            return self.process_ts.poll() is not None
        except Exception:
            return True

    # ------------------------------------------------------------
    def start(self):
        """Start listening immediately; publishers are created when data arrives."""
        self._stop = False
        self._manual_pause = False
        self.publishing = False
        if self.registry:
            self.registry.register_network_camera(self.camera_name, self.output_topic)
        self._launch_ffmpeg()

    # ------------------------------------------------------------
    def _launch_ffmpeg(self):
        cmd_ts = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-i", self.listen_url,
            "-an",
            "-c:v", "copy",
            "-f", "mpegts",
            "pipe:1",
        ]

        self.node.get_logger().info(
            f"[{self.camera_name}] Launching TS receiver:\n  {' '.join(cmd_ts)}"
        )

        self.process_ts = subprocess.Popen(
            cmd_ts,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )

        # Fail fast if ffmpeg could not start
        if self.process_ts.poll() is not None:
            self.node.get_logger().error(
                f"[{self.camera_name}] Failed to start TS receiver"
            )
            return

        # --- Foxglove converter (optional) ---
        if self.output_mode == "foxglove":
            cmd_fg = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel", "error",
                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-f", "mpegts",
                "-i", "pipe:0",
                "-an",
                "-c:v", "copy",
                "-bsf:v", "h264_mp4toannexb",
                "-f", "h264",
                "pipe:1",
            ]

            self.process_foxglove = subprocess.Popen(
                cmd_fg,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )

            threading.Thread(
                target=self._reader_foxglove,
                daemon=True,
            ).start()

        threading.Thread(
            target=self._reader_ts,
            daemon=True,
        ).start()

    # ------------------------------------------------------------
    def _reader_ts(self):
        read = getattr(
            self.process_ts.stdout,
            "read1",
            self.process_ts.stdout.read,
        )
        chunk_size = 188 * 100

        while not self._stop:
            if self.process_ts.poll() is not None:
                break

            chunk = read(chunk_size)
            if not chunk:
                continue

            # First data: bring publishers online unless manually paused
            if not self.publishing and not self._manual_pause:
                self.resume_publishers()

            # Try to infer resolution from the stream if not already known
            if not (self.width and self.height):
                inferred = self._infer_resolution(chunk)
                if inferred:
                    self.width, self.height = inferred
                    self.node.get_logger().info(
                        f"[{self.camera_name}] Inferred resolution: {self.width}x{self.height}"
                    )

            # Touch registry so camera stays alive
            if self.registry:
                self.registry.touch(self.camera_name)

            msg = CompressedImage()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.camera_name
            if self.width and self.height:
                msg.format = f"video/h264;width={self.width};height={self.height};fps={self.fps}"
            else:
                msg.format = f"video/h264;fps={self.fps}"
            msg.data = chunk

            if self.pub_ts:
                self.pub_ts.publish(msg)

            if self.process_foxglove and self.process_foxglove.stdin:
                try:
                    self.process_foxglove.stdin.write(chunk)
                except (BrokenPipeError, ValueError):
                    pass

    # ------------------------------------------------------------
    def _reader_foxglove(self):
        read = getattr(
            self.process_foxglove.stdout,
            "read1",
            self.process_foxglove.stdout.read,
        )

        while not self._stop:
            if self.process_foxglove.poll() is not None:
                break

            chunk = read(65536)
            if not chunk:
                continue

            msg = CompressedVideo()
            msg.timestamp = self.node.get_clock().now().to_msg()
            msg.frame_id = self.camera_name
            msg.format = "h264"
            msg.data = chunk

            if self.pub_foxglove:
                self.pub_foxglove.publish(msg)

    # ------------------------------------------------------------
    def stop(self):
        self._stop = True
        self.pause_publishers(lock=False)

        for proc in (self.process_ts, self.process_foxglove):
            if proc:
                try:
                    proc.terminate()
                except Exception:
                    pass

        self.process_ts = None
        self.process_foxglove = None
        self.publishing = False

        self.node.get_logger().info(
            f"[{self.camera_name}] NetworkTSReceiver stopped"
        )

    # ------------------------------------------------------------
    def _infer_resolution(self, data: bytes):
        """
        Attempt to parse SPS from an H.264 stream to extract width/height.
        Works best when the chunk contains SPS NAL units.
        """
        try:
            for nal in self._iter_nals(data):
                nal_type = nal[0] & 0x1F
                if nal_type == 7:  # SPS
                    return self._parse_sps(nal[1:])
        except Exception:
            return None
        return None

    def _iter_nals(self, data: bytes):
        i = 0
        n = len(data)
        while i + 4 < n:
            if data[i:i+3] == b"\x00\x00\x01":
                start = i + 3
            elif data[i:i+4] == b"\x00\x00\x00\x01":
                start = i + 4
            else:
                i += 1
                continue

            j = start
            while j + 3 < n and data[j:j+3] != b"\x00\x00\x01" and data[j:j+4] != b"\x00\x00\x00\x01":
                j += 1
            yield data[start:j]
            i = j

    # Minimal bitstream reader helpers for SPS parsing
    class _BitReader:
        def __init__(self, data: bytes):
            self.data = data
            self.bit_pos = 0

        def read_bits(self, n: int) -> int:
            val = 0
            for _ in range(n):
                byte_idx = self.bit_pos // 8
                shift = 7 - (self.bit_pos % 8)
                if byte_idx >= len(self.data):
                    return val
                val = (val << 1) | ((self.data[byte_idx] >> shift) & 1)
                self.bit_pos += 1
            return val

        def read_ue(self) -> int:
            zeros = 0
            while self.read_bits(1) == 0:
                zeros += 1
            code_num = (1 << zeros) - 1 + self.read_bits(zeros)
            return code_num

        def read_se(self) -> int:
            code_num = self.read_ue()
            m = (code_num + 1) // 2
            return -m if code_num % 2 == 0 else m

    def _rbsp(self, nal: bytes) -> bytes:
        # Remove emulation prevention bytes (0x000003)
        out = bytearray()
        i = 0
        while i < len(nal):
            if i + 2 < len(nal) and nal[i] == 0 and nal[i+1] == 0 and nal[i+2] == 3:
                out.extend([0, 0])
                i += 3
            else:
                out.append(nal[i])
                i += 1
        return bytes(out)

    def _parse_sps(self, nal_payload: bytes):
        rbsp = self._rbsp(nal_payload)
        br = self._BitReader(rbsp)

        profile_idc = br.read_bits(8)
        br.read_bits(8)  # constraint flags + reserved
        br.read_bits(8)  # level_idc
        br.read_ue()     # seq_parameter_set_id

        # High profiles have extra fields
        if profile_idc in (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135):
            chroma_format_idc = br.read_ue()
            if chroma_format_idc == 3:
                br.read_bits(1)  # separate_colour_plane_flag
            br.read_ue()  # bit_depth_luma_minus8
            br.read_ue()  # bit_depth_chroma_minus8
            br.read_bits(1)  # qpprime_y_zero_transform_bypass_flag
            seq_scaling_matrix_present_flag = br.read_bits(1)
            if seq_scaling_matrix_present_flag:
                # Skip scaling lists (we don't need them for resolution)
                for _ in range(8 if chroma_format_idc == 3 else 12):
                    present = br.read_bits(1)
                    if present:
                        # Skip actual scaling list values
                        last_scale = 8
                        next_scale = 8
                        size = 64 if _ < 6 else 64
                        for _ in range(size):
                            if next_scale != 0:
                                delta_scale = br.read_se()
                                next_scale = (last_scale + delta_scale + 256) % 256
                            last_scale = next_scale if next_scale != 0 else last_scale

        br.read_ue()  # log2_max_frame_num_minus4
        pic_order_cnt_type = br.read_ue()
        if pic_order_cnt_type == 0:
            br.read_ue()  # log2_max_pic_order_cnt_lsb_minus4
        elif pic_order_cnt_type == 1:
            br.read_bits(1)  # delta_pic_order_always_zero_flag
            br.read_se()     # offset_for_non_ref_pic
            br.read_se()     # offset_for_top_to_bottom_field
            num_ref_frames_in_pic_order_cnt_cycle = br.read_ue()
            for _ in range(num_ref_frames_in_pic_order_cnt_cycle):
                br.read_se()

        br.read_ue()  # max_num_ref_frames
        br.read_bits(1)  # gaps_in_frame_num_value_allowed_flag
        pic_width_in_mbs_minus1 = br.read_ue()
        pic_height_in_map_units_minus1 = br.read_ue()
        frame_mbs_only_flag = br.read_bits(1)
        if not frame_mbs_only_flag:
            br.read_bits(1)  # mb_adaptive_frame_field_flag
        br.read_bits(1)  # direct_8x8_inference_flag
        frame_cropping_flag = br.read_bits(1)
        crop_left = crop_right = crop_top = crop_bottom = 0
        if frame_cropping_flag:
            crop_left = br.read_ue()
            crop_right = br.read_ue()
            crop_top = br.read_ue()
            crop_bottom = br.read_ue()

        width = (pic_width_in_mbs_minus1 + 1) * 16
        height = (pic_height_in_map_units_minus1 + 1) * 16 * (2 - frame_mbs_only_flag)

        # Assume default crop unit size for 4:2:0 (2 pixels)
        width -= (crop_left + crop_right) * 2
        height -= (crop_top + crop_bottom) * 2

        if width > 0 and height > 0:
            return width, height
        return None

    # ------------------------------------------------------------

    # ------------------------------------------------------------
    def _build_ffmpeg_cmd(self):
        """
        Build ffmpeg command. If target width/height are provided, transcode
        with scaling to that size (or smaller if input is smaller). Otherwise,
        copy the stream.
        """
        base = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel", "error",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-i", self.listen_url,
            "-an",
        ]

        filters = []
        if self.width and self.height:
            filters.append(f"scale='min({self.width},iw)':'min({self.height},ih)'")

        if filters:
            base += [
                "-vf", ",".join(filters),
                "-c:v", "libx264",
                "-preset", "ultrafast",
                "-tune", "zerolatency",
                "-pix_fmt", "yuv420p",
                "-g", "1",
                "-bf", "0",
                "-b:v", "0",
                "-crf", "23",
                "-f", "mpegts",
                "pipe:1",
            ]
        else:
            base += [
                "-c:v", "copy",
                "-f", "mpegts",
                "pipe:1",
            ]

        return base
