#!/usr/bin/env python3
"""Depth Anything V2 sanity check -- THROWAWAY DIAGNOSTIC, not production
code. Purpose: prove the model produces usable depth on our Gazebo
imagery before any more pipeline code gets written.

For each input frame, saves a side-by-side PNG (original | depth
colormap) and prints raw predicted_depth stats (min/max/mean).

Depth Anything V2 (relative-depth checkpoints, including the Small one
used by default here) predicts RELATIVE INVERSE depth: LARGER values
mean CLOSER, not farther, unlike a normal depth map. This matters for
interpreting "near/far ordering" -- see the printed note.

Usage:
  tools/check_depth.py --frames-dir DIR --out-dir OUT [--limit N]
  tools/check_depth.py --single path/to/one_frame.png --out-dir OUT
"""
import argparse
import glob
import os

import cv2
import numpy as np
from PIL import Image


def load_model(model_name: str):
    from transformers import pipeline
    return pipeline(task="depth-estimation", model=model_name)


def run_one(pipe, frame_path: str, out_dir: str):
    # PIL's .convert("RGB") is the point being confirmed here: our PNGs
    # were written by record_pass.py with an explicit RGB->BGR conversion
    # before cv2.imwrite (cv2.imwrite assumes BGR input) specifically so
    # the FILE ON DISK displays correctly everywhere -- so loading it back
    # with PIL's standard RGB convention reconstructs the same rgb8 array
    # FramePacket.image originally held (see Step H: image encoding is
    # pinned to rgb8, not passthrough). No further conversion is applied
    # before feeding the model -- this pipeline call receives genuine RGB.
    image = Image.open(frame_path).convert("RGB")

    result = pipe(image)
    raw_depth = result["predicted_depth"]  # torch tensor, relative inverse depth
    raw_depth_np = raw_depth.squeeze().detach().cpu().numpy()

    d_min, d_max, d_mean = float(raw_depth_np.min()), float(raw_depth_np.max()), float(raw_depth_np.mean())
    print(f"{os.path.basename(frame_path)}: "
          f"min={d_min:.3f} max={d_max:.3f} mean={d_mean:.3f} "
          f"(larger=closer, relative inverse depth)")

    # Normalize raw depth to 0-255 for the colormap (independent of the
    # pipeline's own 'depth' visualization image, so we're coloring the
    # exact raw tensor we just printed stats for).
    norm = (raw_depth_np - d_min) / max(d_max - d_min, 1e-6)
    norm_u8 = (norm * 255).astype(np.uint8)
    colormap = cv2.applyColorMap(norm_u8, cv2.COLORMAP_INFERNO)

    orig_rgb = np.array(image)
    orig_bgr = cv2.cvtColor(orig_rgb, cv2.COLOR_RGB2BGR)
    if colormap.shape[:2] != orig_bgr.shape[:2]:
        colormap = cv2.resize(colormap, (orig_bgr.shape[1], orig_bgr.shape[0]))

    side_by_side = np.hstack([orig_bgr, colormap])
    out_path = os.path.join(out_dir, f"depth_{os.path.basename(frame_path)}")
    cv2.imwrite(out_path, side_by_side)
    return d_min, d_max, d_mean


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--frames-dir", type=str, help="directory of PNG frames (glob *.png)")
    parser.add_argument("--single", type=str, help="run on a single frame only, for quick iteration")
    parser.add_argument("--out-dir", type=str, required=True)
    parser.add_argument("--limit", type=int, default=50, help="max frames to process from --frames-dir")
    parser.add_argument("--model", type=str, default="depth-anything/Depth-Anything-V2-Small-hf")
    args = parser.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    print(f"loading model {args.model} ...")
    pipe = load_model(args.model)
    print("model loaded. Confirming RGB input: PIL Image.convert('RGB') is "
          "used with no further channel conversion before the model call "
          "-- see run_one()'s docstring comment for why this matches "
          "FramePacket's pinned rgb8 encoding.")

    if args.single:
        run_one(pipe, args.single, args.out_dir)
        return

    if not args.frames_dir:
        parser.error("either --frames-dir or --single is required")

    paths = sorted(glob.glob(os.path.join(args.frames_dir, "*.png")))[:args.limit]
    print(f"processing {len(paths)} frames from {args.frames_dir}")

    stats = []
    for p in paths:
        stats.append(run_one(pipe, p, args.out_dir))

    if stats:
        mins, maxs, means = zip(*stats)
        print(f"\nacross {len(stats)} frames: "
              f"min(min)={min(mins):.3f} max(max)={max(maxs):.3f} "
              f"mean(mean)={sum(means)/len(means):.3f}")
    print(f"side-by-side images saved to {args.out_dir}")


if __name__ == "__main__":
    main()
