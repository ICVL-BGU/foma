#!/usr/bin/env python3
import os
import json
import argparse
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

import cv2
import numpy as np
from tqdm import tqdm
from ultralytics import YOLO

LIDAR_TAG = 1
_MODEL = None
_MAPPER = None


def _map_point(x: float, y: float, mapper_params: dict) -> tuple[float, float]:
    """Map a single image point (x, y) to world coordinates using the mapper JSON."""
    cx = mapper_params["cx"]
    cy = mapper_params["cy"]
    f = mapper_params["f"]

    # Undistort (equisolid fisheye)
    x_d = (x - cx) / f
    y_d = (y - cy) / f
    r_d = np.sqrt(x_d ** 2 + y_d ** 2)

    arg = np.clip(r_d / 2, -1.0, 1.0)
    theta = 2 * np.arcsin(arg)

    scale = np.tan(theta) / r_d if r_d > 1e-9 else 1.0
    x_u = x_d * scale
    y_u = y_d * scale

    undistorted_x = x_u * f + cx
    undistorted_y = y_u * f + cy

    # Homography
    H = np.array(mapper_params["homography"], dtype=float)
    point_h = np.array([undistorted_x, undistorted_y, 1.0], dtype=float)
    world_h = H @ point_h

    x_w = world_h[0] / world_h[2]
    y_w = world_h[1] / world_h[2]
    return float(x_w), float(y_w)


def _create_grid_background(size_px: int = 1000) -> tuple[np.ndarray, float]:
    """Create a square 5000x5000mm grid rendered into size_px pixels."""
    grid_bg = np.ones((size_px, size_px, 3), dtype=np.uint8) * 255
    scale = size_px / 5000.0  # px per mm

    for i in range(0, 5001, 1000):
        px = int(i * scale)
        color = (220, 220, 220)
        thickness = 1
        cv2.line(grid_bg, (px, 0), (px, size_px - 1), color, thickness)
        cv2.line(grid_bg, (0, px), (size_px - 1, px), color, thickness)

    font = cv2.FONT_HERSHEY_SIMPLEX

    for i in range(0, 5001, 1000):
        px = int(i * scale)
        cv2.putText(grid_bg, str(i), (px + 5, 15), font, 0.4, (0, 0, 0), 1)
        cv2.putText(grid_bg, str(i), (5, px + 15), font, 0.4, (0, 0, 0), 1)

    return grid_bg, scale


def _init_models(model_path: str, mapper_path: str):
    global _MODEL, _MAPPER
    if _MODEL is None:
        _MODEL = YOLO(model_path, verbose=False)
    if _MAPPER is None:
        with open(mapper_path, "r") as f:
            _MAPPER = json.load(f)


def _process_video(folder: Path, model_path: str, mapper_path: str, grid_size_px: int) -> tuple[Path, bool, str | None]:
    try:
        _init_models(model_path, mapper_path)
        video_path = folder / "room_video.mp4"
        if not video_path.exists():
            return folder, False, "missing room_video.mp4"

        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            return folder, False, "could not open video"

        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
        
        if frame_count == 0:
            return folder, False, "video has 0 frames (corrupted)"

        grid_bg, scale = _create_grid_background(grid_size_px)
        out_path = folder / "room_map.mp4"
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(out_path), fourcc, fps, (grid_size_px, grid_size_px))

        path_history = []

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            results = _MODEL.track(frame, verbose=False)
            boxes = results[0].boxes
            best_box = None
            best_conf = -1.0
            if boxes is not None:
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())
                    if cls_id == LIDAR_TAG and conf > best_conf:
                        best_conf = conf
                        best_box = box

            grid_frame = grid_bg.copy()
            
            # Add current point to path history
            if best_box is not None:
                cx, cy, _, _ = best_box.xywh[0].cpu().numpy()
                x_w, y_w = _map_point(cx, cy, _MAPPER)
                # Flip y-axis so origin is at bottom-left
                pt = (int(x_w * scale), int(grid_size_px - y_w * scale))
                path_history.append(pt)
            
            # Draw entire path history
            if len(path_history) > 1:
                for i in range(len(path_history) - 1):
                    cv2.line(grid_frame, path_history[i], path_history[i + 1], (0, 255, 0), 2)
            
            # Draw current point
            if len(path_history) > 0:
                cv2.circle(grid_frame, path_history[-1], 6, (0, 0, 255), -1)

            writer.write(grid_frame)

        cap.release()
        writer.release()
        return folder, True, None
    except Exception as exc:
        return folder, False, str(exc)


def main():
    parser = argparse.ArgumentParser(description="Batch room map generator")
    parser.add_argument("root", type=str, help="Path containing folders to process")
    parser.add_argument("--model", type=str, default=r"/home/alex/FOMA/repo/models/foma_detection.pt", help="YOLO model path")
    parser.add_argument("--mapper", type=str, default=r"/home/alex/FOMA/repo/models/image_world_mapper.json", help="Mapper JSON path")
    parser.add_argument("--workers", type=int, default=os.cpu_count() or 4, help="Number of parallel workers")
    parser.add_argument("--grid-size", type=int, default=1000, help="Output grid size in pixels (square)")
    args = parser.parse_args()

    root = Path(args.root)
    if not root.exists() or not root.is_dir():
        raise SystemExit(f"Root path not found or not a directory: {root}")

    # Discover folders containing room_video.mp4
    folders = [p for p in root.iterdir() if p.is_dir()]
    if not folders:
        raise SystemExit("No folders found to process.")

    failures = []
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        futures = {
            executor.submit(_process_video, folder, args.model, args.mapper, args.grid_size): folder
            for folder in folders
        }

        for future in tqdm(
            as_completed(futures),
            total=len(futures),
            desc="Processing folders",
            unit="folder",
        ):
            folder, ok, err = future.result()
            if not ok:
                failures.append((folder, err or "unknown error"))

    if failures:
        print("\n" + "="*60)
        print("SKIPPED OR FAILED VIDEOS:")
        print("="*60)
        for folder, reason in failures:
            print(f"  {folder.name}: {reason}")
        print("="*60)
        print(f"Total failed: {len(failures)}/{len(folders)}")
    else:
        print("\nAll videos processed successfully.")


if __name__ == "__main__":
    main()
