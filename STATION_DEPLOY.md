# Station Deploy Guide

How to pull this branch onto the lab PC ("station") and run it after the
config / event-writer / GPU refactor.

Assumed split: this PC runs `gui`, `ceiling_camera`, `localization`,
`light_dimmer`, all writer nodes, and the side-by-side worker. The Pi
("foma") runs `lidar`, `motor_control`, `video_camera`, `feeder` from its
own checkout and only needs `git pull` + `catkin_make`.

---

## 1. Pull and rebuild

```bash
cd ~/ros_ws/src/foma            # adjust if your workspace lives elsewhere
git fetch && git pull
cd ~/ros_ws
catkin_make                     # required: TrialEvent.msg is new
source devel/setup.bash
```

If `catkin_make` errors on `foma/TrialEvent`, run `catkin_make clean` once
then `catkin_make` again — stale generated Python lingers otherwise.

## 2. System packages

Most are usually present; install the ones missing. Apt names below are
for Ubuntu 20.04 / 22.04.

```bash
sudo apt update
sudo apt install -y \
    libturbojpeg                 \
    gstreamer1.0-plugins-bad     \
    gstreamer1.0-plugins-ugly    \
    gstreamer1.0-libav           \
    ffmpeg
```

NVIDIA GStreamer plugins (`nvh264dec`, `nvh264enc`, `nvvideoconvert`)
come from the NVIDIA driver / `gstreamer1.0-vaapi` on desktops and from
`nvidia-l4t-gstreamer` on Jetson. On a desktop with a recent NVIDIA
driver they are typically already installed; verify with the probes in
§5 before debugging.

## 3. Python packages

```bash
# ROS noetic uses system Python 3.8; install with --user or in your venv
pip3 install --user PyTurboJPEG PyYAML numpy
```

`PyTurboJPEG` is optional — the helper falls back to `cv2.imdecode` if
missing, just slower. Same for NVIDIA plugins: missing → CPU fallback,
logged as `WARN`.

## 4. Run

```bash
# Terminal 1
roscore

# Terminal 2 (on the station PC)
source ~/ros_ws/devel/setup.bash
roslaunch foma station.launch
```

That launch file loads `config/foma_params.yaml` into `/`-namespace
rosparams, brings up GUI / ceiling camera / localization / dimmer, and
includes `writers.launch` which spins up:

- `csv_writer_foma_location`, `csv_writer_fish_location`, `csv_writer_foma_speed`, `csv_writer_lidar`
- `event_writer`
- `video_writer_room`, `video_writer_foma`, `video_writer_room_map`

The Pi side runs separately, e.g.:

```bash
roslaunch foma foma.launch
```

## 5. Verify GPU paths after launch

Look in `roslaunch` console output:

| Node               | Expected line                                                   |
|--------------------|-----------------------------------------------------------------|
| `ceiling_camera`   | `CeilingCamera: using NVIDIA nvh264dec + nvvideoconvert.`       |
| `video_writer_*`   | `Created NVENC video writer at …`                               |

If you see fallback warnings, probe what's actually installed:

```bash
gst-inspect-1.0 nvh264dec nvh264enc nvvideoconvert 2>&1 | grep -E 'Factory|No such'
ffmpeg -hide_banner -encoders 2>&1 | grep -E 'h264_nvenc'
ffmpeg -hide_banner -decoders 2>&1 | grep -E 'h264_cuvid'
python3 -c 'from turbojpeg import TurboJPEG; print("turbojpeg OK")'
```

While a trial is running, confirm the GPU is actually doing decode/encode:

```bash
nvidia-smi dmon -s u            # watch "enc" / "dec" columns
```

## 6. After a trial: side-by-side video

When you click **Reset** to end a trial, the GUI spawns a detached worker
that produces `<trial_folder>/side_by_side.mp4` from `room_video.mp4` +
`foma_video.mp4`. Progress lives in `<trial_folder>/sidebyside.log`.

It tries `h264_cuvid` decode + `h264_nvenc` encode first; on nonzero exit
it retries with `libx264 veryfast`. Both inputs are normalized to the
higher fps and the smaller height is stretched to the larger.

## 7. Editing parameters

All shared constants and per-node writer config live in
`config/foma_params.yaml`. Edit there only — `src/etc/settings.py` has
been removed. Resolutions use YAML anchors so the ceiling camera and the
room video writer can't drift apart:

```yaml
ROOM_CAMERA_FRAME_SHAPE: &room_camera_shape [640, 640]
...
video_writer_room:
  frame_shape: *room_camera_shape
```

After changing the YAML, just relaunch — no rebuild required.

## 8. Troubleshooting

- **`KeyError: '/FOO'` at startup** → that node started before any launch
  loaded the YAML; only run nodes through `roslaunch`, never via raw
  `rosrun foma <node>.py`.
- **`Failed to open GStreamer pipeline`** in `ceiling_camera` → URL wrong
  or NVIDIA fallback also failed; check `rtsp://...` works in VLC.
- **`NVENC pipeline unavailable, falling back …`** → gst-bad nvcodec
  plugins missing; see §2.
- **Red/blue swap in `room_video.mp4`** → was caused by an old stray
  `RGB2BGR` after a BGR decode; this branch removes it. Re-record a
  trial and verify; if still wrong, capture one frame and open it with
  `feh`/VLC to confirm before blaming the writer.
- **Event log missing** → check `event_writer` started in roslaunch
  output, and that `event_writer/write` was reachable from the GUI on
  trial start.
