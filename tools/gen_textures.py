#!/usr/bin/env python3
"""Generate non-periodic, high-frequency noise textures for the new
world (Step N). Multi-octave value noise (poor-man's Perlin) so nothing
tiles/repeats in an obviously periodic way -- avoids the checkerboard
aperture-ambiguity problem the task explicitly warns about."""
import numpy as np
from PIL import Image


def value_noise(size, scale, seed):
    """Smooth random noise at a given feature scale via bilinear
    upsampling of coarse random grid -- cheap fractal-noise building
    block."""
    rng = np.random.default_rng(seed)
    small = max(2, size // scale)
    coarse = rng.random((small, small))
    img = Image.fromarray((coarse * 255).astype(np.uint8), mode="L")
    img = img.resize((size, size), Image.BICUBIC)
    return np.asarray(img, dtype=np.float64) / 255.0


def fractal_noise(size, base_scale, octaves, seed):
    total = np.zeros((size, size))
    amplitude = 1.0
    amp_sum = 0.0
    for o in range(octaves):
        scale = max(2, base_scale // (2 ** o))
        total += amplitude * value_noise(size, scale, seed + o * 101)
        amp_sum += amplitude
        amplitude *= 0.55
    return total / amp_sum


def make_texture(path, size, base_color, variation, base_scale, octaves, seed,
                  streak_axis=None, streak_strength=0.0, grain_strength=0.35):
    noise = fractal_noise(size, base_scale, octaves, seed)
    noise = (noise - noise.min()) / (noise.max() - noise.min())

    # Sharp per-pixel grain on top of the smooth fractal blobs -- the
    # blobs alone are too soft (low local contrast) for corner
    # detection; goodFeaturesToTrack needs genuine sharp gradients, not
    # just slow blob-to-blob variation.
    rng = np.random.default_rng(seed + 555)
    grain = rng.random((size, size))
    noise = (1 - grain_strength) * noise + grain_strength * grain

    if streak_axis is not None and streak_strength > 0:
        # Elongate noise along one axis (e.g. vertical bark streaks) by
        # blending in a stretched low-frequency component.
        stretched = fractal_noise(size, base_scale * 4, 2, seed + 999)
        if streak_axis == "y":
            stretched = np.tile(stretched.mean(axis=0, keepdims=True), (size, 1))
        else:
            stretched = np.tile(stretched.mean(axis=1, keepdims=True), (1, size))
        noise = (1 - streak_strength) * noise + streak_strength * stretched

    rgb = np.zeros((size, size, 3), dtype=np.float64)
    for c in range(3):
        rgb[:, :, c] = base_color[c] + variation[c] * (noise - 0.5) * 2
    rgb = np.clip(rgb, 0.0, 1.0)
    Image.fromarray((rgb * 255).astype(np.uint8), mode="RGB").save(path)
    print(f"wrote {path} ({size}x{size})")


OUT = "/home/saurabh/ardu_ws/src/obst_avoidance/models/textures"

# Ground: fine gravel/dirt speckle, high resolution so a single UV tile
# across a large plane still has local detail at flight altitude.
make_texture(f"{OUT}/ground_gravel.png", 2048,
             base_color=(0.42, 0.38, 0.32), variation=(0.16, 0.14, 0.12),
             base_scale=48, octaves=5, seed=1)

# Tree bark: brown, vertically-streaked, fine noise. Bumped resolution
# and grain vs the first pass -- Step O found close-range views (a
# single obstacle filling most of the frame, common in the tight-gap
# zones) dropped below the 100-feature target with the original,
# softer textures. Finer base_scale + stronger grain fixes that.
make_texture(f"{OUT}/bark.png", 1024,
             base_color=(0.35, 0.22, 0.12), variation=(0.16, 0.11, 0.08),
             base_scale=14, octaves=4, seed=2,
             streak_axis="y", streak_strength=0.30, grain_strength=0.55)

# Concrete/building: gray mottled noise for box obstacles and walls.
make_texture(f"{OUT}/concrete.png", 1536,
             base_color=(0.55, 0.54, 0.52), variation=(0.16, 0.16, 0.16),
             base_scale=22, octaves=5, seed=3, grain_strength=0.5)

# Stone/rubble: a second, visually distinct textured surface for Zone C
# variety (warm gray-brown, coarser noise).
make_texture(f"{OUT}/stone.png", 1536,
             base_color=(0.48, 0.42, 0.36), variation=(0.20, 0.18, 0.16),
             base_scale=16, octaves=5, seed=4, grain_strength=0.5)

# Leaves: mottled green canopy texture (canopies were flat, zero-texture
# spheres before -- "textured is the default" applies to them too now).
make_texture(f"{OUT}/leaves.png", 1024,
             base_color=(0.16, 0.38, 0.12), variation=(0.14, 0.16, 0.12),
             base_scale=12, octaves=4, seed=5, grain_strength=0.5)
