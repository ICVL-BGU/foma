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
    for line in build_info.split('\n'):
        line = line.strip()
        if any(key in line for key in relevant_keys):
            print(f"  {line}")
    
    print()
    print("=" * 60)
    
    if 'X264' in available_codecs or 'H264' in available_codecs:
        print("✓ H.264 ENCODING IS AVAILABLE")
        print("\nRecommended codec for your system:", 'X264' if 'X264' in available_codecs else 'H264')
        return True
    else:
        print("✗ H.264 ENCODING NOT AVAILABLE")
        print("\nYou can:")
        print("1. Install: sudo apt install libx264-dev")
        print("2. Rebuild OpenCV from source (see OPENCV_H264_BUILD_GUIDE.md)")
        print("3. Use fallback codec 'mp4v' (slower, larger files)")
        return False

if __name__ == "__main__":
    result = check_opencv_h264()
    sys.exit(0 if result else 1)
