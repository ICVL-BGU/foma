# OpenCV with H.264 Support - Build Guide

## Quick Check Script

Save this as `check_opencv_h264.py` and run it:

```python
#!/usr/bin/env python3
"""
OpenCV H.264 Support Checker
Verifies if OpenCV is properly configured for H.264 video encoding
"""

import sys

def check_opencv_h264():
    print("=" * 60)
    print("OpenCV H.264 Support Checker")
    print("=" * 60)
    
    # Check if OpenCV is installed
    try:
        import cv2
        print(f"✓ OpenCV installed: {cv2.__version__}")
    except ImportError:
        print("✗ OpenCV NOT installed")
        print("\nInstall with: sudo apt install python3-opencv")
        return False
    
    print()
    
    # Check available codecs
    codecs = {
        'X264': 'H.264 (software)',
        'H264': 'H.264 (may use hardware)',
        'avc1': 'H.264 (Apple variant)',
        'mp4v': 'MPEG-4 Part 2 (fallback)',
        'MJPG': 'Motion JPEG (fast, large)',
        'XVID': 'Xvid (alternative)',
    }
    
    print("Codec Support:")
    print("-" * 60)
    available_codecs = []
    for codec_name, description in codecs.items():
        try:
            fourcc = cv2.VideoWriter_fourcc(*codec_name)
            if fourcc > 0:
                print(f"  ✓ {codec_name:6s} - {description}")
                available_codecs.append(codec_name)
            else:
                print(f"  ✗ {codec_name:6s} - {description} (not available)")
        except Exception as e:
            print(f"  ✗ {codec_name:6s} - Error: {e}")
    
    print()
    
    # Test actual video writing
    print("Testing Video Writer:")
    print("-" * 60)
    
    import numpy as np
    import tempfile
    import os
    
    test_codecs = ['X264', 'H264', 'mp4v']
    success = False
    
    for codec in test_codecs:
        if codec not in available_codecs:
            continue
            
        temp_file = tempfile.mktemp(suffix='.mp4')
        try:
            fourcc = cv2.VideoWriter_fourcc(*codec)
            writer = cv2.VideoWriter(temp_file, fourcc, 25.0, (640, 480))
            
            if not writer.isOpened():
                print(f"  ✗ {codec}: VideoWriter failed to open")
                continue
            
            # Write test frames
            for i in range(10):
                frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                writer.write(frame)
            
            writer.release()
            
            # Check if file was created and has content
            if os.path.exists(temp_file) and os.path.getsize(temp_file) > 0:
                size_kb = os.path.getsize(temp_file) / 1024
                print(f"  ✓ {codec}: SUCCESS - wrote {size_kb:.1f} KB")
                success = True
                os.remove(temp_file)
            else:
                print(f"  ✗ {codec}: File created but empty")
        except Exception as e:
            print(f"  ✗ {codec}: {e}")
        finally:
            if os.path.exists(temp_file):
                try:
                    os.remove(temp_file)
                except:
                    pass
    
    print()
    
    # Check build info
    print("OpenCV Build Information:")
    print("-" * 60)
    build_info = cv2.getBuildInformation()
    
    # Extract relevant lines
    relevant_keys = ['Video I/O', 'FFMPEG', 'GStreamer', 'codec']
    for line in build_info.split('\\n'):
        line = line.strip()
        if any(key in line for key in relevant_keys):
            print(f"  {line}")
    
    print()
    print("=" * 60)
    
    if 'X264' in available_codecs or 'H264' in available_codecs:
        print("✓ H.264 ENCODING IS AVAILABLE")
        print("\nRecommended codec for your system: X264" if 'X264' in available_codecs else "H264")
        return True
    else:
        print("✗ H.264 ENCODING NOT AVAILABLE")
        print("\nYou can:")
        print("1. Install: sudo apt install libx264-dev")
        print("2. Rebuild OpenCV from source (see guide below)")
        print("3. Use fallback codec 'mp4v' (slower, larger files)")
        return False

if __name__ == "__main__":
    result = check_opencv_h264()
    sys.exit(0 if result else 1)
```

**Run it:**
```bash
# Make executable
chmod +x check_opencv_h264.py

# Run check
./check_opencv_h264.py
```

---

## Check Current Support

```bash
# In your ROS environment
source ~/ros_ws/devel/setup.bash
python3 -c "import cv2; print('OpenCV:', cv2.__version__); print('H264 support:', cv2.VideoWriter_fourcc(*'X264'))"
```

If you see an error or negative value, H.264 is not available.

---

## Option 1: Install via apt (Ubuntu/Debian) - EASIEST

```bash
# Install x264 library
sudo apt update
sudo apt install libx264-dev

# Install OpenCV with H.264 support
sudo apt install python3-opencv

# For ROS: reinstall cv_bridge
sudo apt install ros-noetic-cv-bridge
```

---

## Option 2: Build OpenCV from Source with H.264 - ADVANCED

### Prerequisites

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev \
    python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev
```

### Build OpenCV

```bash
# Create workspace
mkdir -p ~/opencv_build && cd ~/opencv_build

# Clone OpenCV
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# Checkout specific version (match your current OpenCV)
cd opencv
git checkout 4.5.5  # Adjust to your version
cd ../opencv_contrib
git checkout 4.5.5
cd ..

# Create build directory
cd opencv && mkdir build && cd build

# Configure with H.264 support
cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D WITH_FFMPEG=ON \
    -D WITH_GSTREAMER=ON \
    -D WITH_V4L=ON \
    -D BUILD_opencv_python3=ON \
    -D PYTHON3_EXECUTABLE=$(which python3) \
    -D PYTHON3_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -D PYTHON3_PACKAGES_PATH=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
    -D BUILD_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D OPENCV_ENABLE_NONFREE=ON ..

# Build (use all cores)
make -j$(nproc)

# Install
sudo make install
sudo ldconfig
```

### Verify Installation

```bash
python3 -c "import cv2; print('OpenCV:', cv2.__version__); \
    print('X264:', cv2.VideoWriter_fourcc(*'X264')); \
    print('H264:', cv2.VideoWriter_fourcc(*'H264')); \
    print('Build info:'); print(cv2.getBuildInformation())"
```

Look for:
- `FFMPEG: YES`
- `Video I/O: ... ffmpeg`

---

## Option 3: Use GStreamer Backend (Already in your system!)

Your `ceiling_camera_node.py` already uses GStreamer. OpenCV can use GStreamer for encoding:

```python
# Instead of changing fourcc, use GStreamer pipeline for writing
gst_out = (
    f"appsrc ! videoconvert ! x264enc speed-preset=ultrafast tune=zerolatency ! "
    f"h264parse ! mp4mux ! filesink location={filepath}"
)
writer = cv2.VideoWriter(gst_out, cv2.CAP_GSTREAMER, 0, fps, frame_shape, True)
```

This uses hardware-accelerated H.264 encoding via GStreamer.

---

## Codec Comparison

| Codec | Speed | Quality | File Size | Hardware Accel |
|-------|-------|---------|-----------|----------------|
| mp4v  | Medium| Low     | Large     | No             |
| X264  | Fast  | High    | Small     | Software       |
| H264  | Fast  | High    | Small     | Yes (if avail) |
| MJPEG | Fastest| Medium | Huge      | No             |

**Recommendation**: Use `X264` - good balance of speed and quality.

---

## Troubleshooting

### Error: "Could not find encoder for codec id 27"
- X264 library not installed
- Solution: `sudo apt install libx264-dev` and rebuild OpenCV

### Error: "VIDEOIO ERROR: ... codec not found"
- OpenCV not built with FFMPEG support
- Solution: Build from source with `-D WITH_FFMPEG=ON`

### Video file is huge
- Using uncompressed codec or MJPEG
- Solution: Switch to X264/H264

### System running slow during video recording
- CPU encoding is expensive for 2560x2560
- Solution: Use hardware encoder or reduce resolution

---

## Testing New Codec

```bash
# Quick test
python3 << 'EOF'
import cv2
import numpy as np

fourcc = cv2.VideoWriter_fourcc(*'X264')
out = cv2.VideoWriter('/tmp/test.mp4', fourcc, 25.0, (640, 480))

for i in range(100):
    frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    out.write(frame)

out.release()
print("Test successful! File size:", os.path.getsize('/tmp/test.mp4'), "bytes")
EOF

# Play the test video
ffplay /tmp/test.mp4
```

---

## Final Notes

- Your system already has NVMe storage ✓
- GStreamer is already installed ✓
- The `X264` codec should work out of the box on most Linux systems
- If `X264` fails, fallback to `mp4v` or try `MJPG` for fastest (but largest files)
