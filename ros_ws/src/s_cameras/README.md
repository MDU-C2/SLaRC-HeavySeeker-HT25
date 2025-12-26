# s_cameras – FPV camera stack

ROS 2 package that detects cameras, launches the right drivers, and serves H.264 FPV streams to a CLI viewer. It supports USB webcams, OAK-D/DepthAI devices, and H.264 network cameras (for example, a Pi streaming over UDP/TCP).

## What it does
- Discovers cameras (`init_cameras`) with `CameraManager`: scans USB (`usb_cam`), DepthAI, and any `h264_network` entries in `config/cameras.yaml`. Names are auto‑assigned (`camera0`, `oak0`, …) and merged with YAML overrides.
- Brings up drivers (`launch/multi_cameras.launch.py`): starts `usb_cam` for USB and DepthAI launch for OAK. Network cameras are left for the server to handle.
- Runs a server (`camera_server`): chooses an encoder (NVENC/QSV/VAAPI/CPU), exposes actions to start/stop cameras, and republishes network H.264 streams through `NetworkTSReceiver`.
- Provides a client (`fpv_client`): CLI to list/start/stop cameras and view streams. Uses `decoder.py` for the windowed viewer and supports Foxglove/headless modes.

## Architecture
- **Server node**: `camera_server` (`server_node` executable).
  - Parameters: `cameras_json` (populated from `CameraManager`), `encoder.*` tuning flags (bitrate, gop, etc.).
  - Services: `get_available_cameras` (`s_msgs/srv/GetCameras`) reports detected cameras and encoders; `set_output_mode` (`s_msgs/srv/SetOutputMode`) switches `mpegts|foxglove|headless`.
  - Actions: `start_camera_encoding` / `stop_camera_encoding` (`s_msgs/action/StartCamera`, `StopCamera`) to start/stop encoders or network receivers.
  - Topics: `/{name}/encoded/{codec}` (MPEG‑TS as `CompressedImage`); optional `.../foxglove` (`CompressedVideo`) in foxglove mode. Network cameras reuse the same layout via `NetworkTSReceiver`.
  - Registry: `CameraRegistry` tracks heartbeats and maps camera name → input topic for local cameras; network cameras are registered so clients can find them even if they stream over UDP/TCP.

## Encoders and output modes
- Local cameras: `CameraEncoder` converts raw ROS images to H.264 (MPEG‑TS).
- Network cameras: `NetworkTSReceiver` pulls the H.264 TS from the URL and republishes. In `foxglove` mode it also converts TS→Annex‑B.
- Output modes (client command `use <mode>`):
  - `mpegts`: default viewer window.
  - `foxglove`: publish Annex‑B for Foxglove.
  - `headless`: no local viewer but starts encoding and publishes topics.

## Encoder parameters (from `camera_server.launch.py`)
All encoder knobs are ROS 2 parameters on `camera_server` and can be overridden on the launch command line, e.g.:
`ros2 launch s_cameras camera_server.launch.py encoder.bitrate:=6M encoder.bitrate_mode:=VBR`

- `encoder.prefer_hevc` (bool): pick H.265 before H.264 when hardware supports it. Default in launch is `False` for compatibility.
- `encoder.quality` (int 1–7): maps to ffmpeg presets per backend; higher = faster/lower quality (opposite of CRF). Also toggles low‑latency tunes on some encoders.
- `encoder.latency` (`ultra_low|low|normal`): enables low‑latency flags (lookahead off, low_power, etc.) when set to ultra_low/low.
- `encoder.bitrate_mode` (`CBR|VBR|CQP|CRF`): rate control mode passed to the encoder backend.
- `encoder.bitrate`, `encoder.maxrate`, `encoder.bufsize` (strings like `3M`): primary bitrate, maxrate cap, and VBV buffer size. Used by `common_extra_args`.
- `encoder.crf` (int): CRF value for CPU encoders (x264/x265) when bitrate_mode is CRF/CQP; ignored by hardware modes.
- `encoder.gop` (int): GOP length / keyframe interval (`-g`).
- `encoder.bframes` (int): number of B‑frames (`-bf`).
- `encoder.mux` (string): container for the encoded stream, default `mpegts`.
- `encoder.mux_flags` (string): extra muxer flags appended to ffmpeg (flush/nobuffer/low delay).

## Configuring cameras
- File: `src/s_cameras/config/cameras.yaml` (installed to the share directory).
- Sections:
  - `usb_cameras`: per‑ID overrides; defaults in code are 1280x720 @ 30 fps with auto exposure/focus/AWB.
  - `oak_cameras`: DepthAI parameters (see `config/README.md` for the full list).
  - `h264_network_cameras`: name → `{url, port?, params}`. `framerate` is metadata, defaults to 30 if missing, though this should be set to match the framerate of the network camera (Raspberry Pi).
- USB/OAK matches by device IDs/MxID; network cameras match by name.

## Run the stack
> **Note**   
Build `s_msgs` (in this workspace) before `s_cameras` since the server and client use its actions and services.
- Build: `colcon build --packages-select s_cameras` and source the workspace.
- Launch drivers + server: `ros2 launch s_cameras cameras.launch.py`
- Start the client: `ros2 run s_cameras fpv_client`
  - Handy commands: `list`, `start <cam...>`, `stop`, `use mpegts|foxglove|headless`, `exit`. Support starting several cameras in one command `(start camera0 camera1 oak0 ...)`. Stop can stop individual cameras such as `stop camera0` or stop all cameras by only sending `stop` 

## Client notes
- `fpv_client.py` drives the CLI, calls server services/actions via `client_service.py`, and manages the decoder with `image_decoder.py`.
- `decoder.py` consumes `CompressedImage` MPEG‑TS topics, decodes with ffmpeg, and shows a resizable OpenCV window; supports multiple cameras side‑by‑side with bandwidth/FPS overlays.
- Typical flow: `list` → `start camera0` → viewer opens; `use foxglove` to switch output; `stop` to stop all.

## Visual guide

### FPV client CLI

> **Note**  
> If you run the `start` command and no window opens within a reasonable amount of time, you likely do not have **ffmpeg** installed.  
> Install it with:
>
> ```bash
> sudo apt install ffmpeg
> ```
>
> Then relaunch the client.

![FPV client CLI](docs/images/fpv_client_interface.png)

**Commands**
- `list` – shows available cameras and which are active
- `start <cam...>` – begin streaming
- `use mpegts|foxglove|headless` – switch output mode
- `stop` – stop active cameras (or just the decoder if none are active)
- `exit` – quit the client

---

### Encoder selection

![Encoder selection](docs/images/encoder_usage.png)

On server launch, encoders are probed in the following order:  
**NVENC → QSV → VAAPI → CPU**

- “Selected NVENC / QSV / VAAPI” → hardware acceleration in use
- “Selected libx264” → CPU fallback (no hardware encoder detected)

> **Note**  
> The CPU fallback works but can significantly increase CPU usage.  
> If possible, install the appropriate GPU drivers to enable hardware encoding.


### Managed cameras

![Managed cameras](docs/images/managed_cameras.png)

- Shows detected USB/OAK cameras and configured network cameras
- Entries marked **“new”** lack a known ID/serial or are not yet in `cameras.yaml`
- When a serial/ID is shown, add it to the YAML to lock names and parameters

## Requirements
- ROS 2 (tested with Jazzy) with `usb_cam` and `depthai_ros_driver` installed.
- `ffmpeg` available on the host system (encoder selection is automatic).
- Supported hardware encoders:
  - NVIDIA (NVENC)
  - Intel (QSV / VAAPI)
  - CPU fallback (`libx264`)
- Network cameras must stream **H.264 over MPEG-TS** at the configured URL.

## Installation (Jazzy)
Install package dependencies and tools:
```bash
sudo apt install ffmpeg
sudo rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
sudo pip install depthai --no-deps --force-reinstall --break-system-packages
```

