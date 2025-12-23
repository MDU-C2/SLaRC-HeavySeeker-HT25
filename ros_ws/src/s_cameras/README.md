# s_cameras – FPV camera stack

ROS 2 package that detects cameras, launches the right drivers, and serves H.264 FPV streams to a CLI viewer. It handles USB webcams, OAK/DepthAI devices, and H.264 network cameras (e.g., Pi streaming H.264 over UDP/TCP).

## How it works
- Detection & naming (`init_cameras`): `CameraManager` scans USB (`usb_cam`), DepthAI devices, and reads any `h264_network` entries from `config/cameras.yaml`. Cameras are auto‑named (`camera0`, `oak0`, …) and merged with YAML overrides.
- Drivers (`launch/multi_cameras.launch.py`): Starts `usb_cam` for USB, DepthAI launch for OAK. Network cameras are not started here—they’re handled by the server.
- Server (`camera_server`): Picks an encoder (NVENC/QSV/VAAPI/CPU) via `EncoderManager`, exposes actions to start/stop cameras, and republishes network H.264 streams through `NetworkTSReceiver`.
- Client (`fpv_client`): CLI tool to list/start/stop cameras and view streams. Uses `decoder.py` to open a window or can be switched to Foxglove/headless modes.

## Server
- Node: `camera_server` (executable `server_node`).
- Parameters: `cameras_json` (injected by launch from `CameraManager`), `encoder.*` tuning flags (bitrate, gop, etc.).
- Services:
  - `get_available_cameras` (`s_msgs/srv/GetCameras`): reports detected cameras and active encoders.
  - `set_output_mode` (`s_msgs/srv/SetOutputMode`): switch between `mpegts|foxglove|headless`.
- Actions:
  - `start_camera_encoding` / `stop_camera_encoding` (`s_msgs/action/StartCamera`, `StopCamera`): start/stop local encoders or network receivers.
- Topics produced:
  - Local cameras: `/{name}/encoded/{codec}` (MPEG‑TS as `CompressedImage`); optional `.../foxglove` (`CompressedVideo`) in foxglove mode.
  - Network cameras: same topic layout via `NetworkTSReceiver`.
- Registry: `CameraRegistry` tracks heartbeats and maps camera name → input topic for local cameras; network cameras are registered so clients can see them even if they’re streaming over UDP/TCP.

## Encoders and output modes
- Local cameras (USB/OAK): `CameraEncoder` encodes raw ROS images to H.264 (MPEG‑TS). Resolution now follows the camera’s native size (no downscale).
- Network cameras: `NetworkTSReceiver` pulls the H.264 TS from the URL, infers resolution from the stream, and republishes. In `foxglove` mode it runs a TS→Annex‑B converter like the local path.
- Output modes (client command `use <mode>`): `mpegts` (default windowed decoder), `foxglove` (publish Annex‑B for Foxglove), `headless` (no local viewer).

## cameras.yaml
- Location: `src/s_cameras/config/cameras.yaml` (installed to the share dir).
- Sections:
  - `usb_cameras`: per‑ID overrides (else defaults in code: 1280x720, 30 fps, auto exposure/focus/AWB).
  - `oak_cameras`: DepthAI params (see `config/README.md` for full list).
  - `h264_network_cameras`: name → `{url, port?, params}`. Width/height are optional; the receiver infers resolution. `framerate` is metadata (defaults to 30 if omitted).
- IDs for USB/OAK are matched by device IDs/MxID; network cams are matched by name.

## Running
- Build: `colcon build --packages-select s_cameras` and source your workspace.
- Launch everything (drivers + server): `ros2 launch s_cameras cameras.launch.py`
- Client: `ros2 run s_cameras fpv_client`
  - Commands: `list`, `start <cam...>`, `stop`, `use mpegts|foxglove|headless`, `exit`.

## Client (fpv_client)
- Composition: `fpv_client.py` orchestrates the CLI, calls server services/actions via `client_service.py`, and manages the decoder via `image_decoder.py`.
- Decoder: `decoder.py` consumes `CompressedImage` MPEG‑TS topics, decodes with ffmpeg, and shows a resizable OpenCV window; supports multiple cameras side‑by‑side with bandwidth/FPS overlays.
- Modes: `use mpegts` (default viewer), `use foxglove` (publishes Annex‑B only), `use headless` (no local window).
- Typical flow: `list` → `start camera0` → viewer opens; `use foxglove` to switch outputs; `stop` to stop all.

## Visual guide
- FPV client CLI (`docs/images/fpv_client_interface.png`):  
  - Commands: `list` (shows available cameras and which are active), `start <cam...>` to begin streaming, `use mpegts|foxglove|headless` to switch output mode, `stop` to stop active cameras (or just the decoder if none are active), `exit` to quit.
- Encoder selection (`docs/images/encoder_usage.png`):  
  - On server launch, encoders are probed (NVENC, QSV, VAAPI, then CPU). “Selected VAAPI” (or NVENC/QSV) means hardware accel is used. If it falls back to `libx264`, no GPU encoder was detected or drivers are missing; it works but will load the CPU heavily—install the correct GPU drivers if possible.
- Managed cameras (`docs/images/managed_cameras.png`):  
  - Shows detected USB/OAK cameras and configured network cameras. Entries marked “new” lack a known ID/serial or are not yet in `cameras.yaml`. When a serial/ID is shown, add it to the YAML to lock names/params.

## Requirements
- ROS 2 (tested with Jazzy) with `usb_cam` and `depthai_ros_driver` installed.
- ffmpeg available on the host (encoders selected automatically).
- For hardware encoders: NVIDIA (NVENC), Intel (QSV/VAAPI) or CPU fallback.
- Network cameras must stream H.264 TS at the configured URL.

## Installation (Jazzy)
Install package dependencies and tools:
```bash
sudo rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
sudo pip install depthai --no-deps --force-reinstall --break-system-packages
sudo apt install ffmpeg
```
- Messages: `s_msgs` (in this workspace) must be built before `s_cameras` since the server/client use its actions and services.
