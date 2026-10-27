import numpy as np
from noise import pnoise2
from PIL import Image

# ============================================================
# Terrain parameters
# ============================================================
N = 2000                  # resolution: 500x500
terrain_size = 50.0     # half-size in meters -> 20m x 20m terrain
max_height = 1.0      # maximum terrain height
seed = 42

# Reproducibility
np.random.seed(seed)

# ============================================================
# Generate coordinate grid
# ============================================================
heightmap = np.zeros((N, N), dtype=np.float32)

for i in range(N):
    for j in range(N):

        # Normalized coordinates [0, 1]
        x = i / (N - 1)
        y = j / (N - 1)

        # ----------------------------------------------------
        # 1. Large-scale hills
        # ----------------------------------------------------
        large = pnoise2(
            x * 1.5,
            y * 1.5,
            octaves=3,
            persistence=0.6,
            lacunarity=2.0,
            base=seed,
        )

        # ----------------------------------------------------
        # 2. Medium terrain structure
        # ----------------------------------------------------
        medium = pnoise2(
            x * 5.0,
            y * 5.0,
            octaves=4,
            persistence=0.55,
            lacunarity=2.0,
            base=seed + 10,
        )

        # ----------------------------------------------------
        # 3. Fine roughness
        # ----------------------------------------------------
        fine = pnoise2(
            x * 15.0,
            y * 15.0,
            octaves=4,
            persistence=0.5,
            lacunarity=2.0,
            base=seed + 20,
        )

        # ----------------------------------------------------
        # 4. High-frequency small bumps
        # ----------------------------------------------------
        detail = pnoise2(
            x * 35.0,
            y * 35.0,
            octaves=2,
            persistence=0.5,
            lacunarity=2.0,
            base=seed + 30,
        )

        # ----------------------------------------------------
        # Combine multiple scales
        # ----------------------------------------------------
        height = (
            0.45 * large +
            0.30 * medium +
            0.18 * fine +
            0.07 * detail
        )

        heightmap[i, j] = height


# ============================================================
# Add localized hills / bumps
# ============================================================
num_bumps = 25

for _ in range(num_bumps):

    cx = np.random.uniform(0.05, 0.95)
    cy = np.random.uniform(0.05, 0.95)

    radius = np.random.uniform(0.01, 0.06)
    amplitude = np.random.uniform(0.05, 0.20)

    yy, xx = np.meshgrid(
        np.linspace(0, 1, N),
        np.linspace(0, 1, N),
        indexing="ij"
    )

    dist2 = (xx - cx) ** 2 + (yy - cy) ** 2

    bump = amplitude * np.exp(
        -dist2 / (2.0 * radius**2)
    )

    heightmap += bump


# ============================================================
# Add some depressions / holes
# ============================================================
num_pits = 15

for _ in range(num_pits):

    cx = np.random.uniform(0.05, 0.95)
    cy = np.random.uniform(0.05, 0.95)

    radius = np.random.uniform(0.01, 0.05)
    amplitude = np.random.uniform(0.03, 0.12)

    yy, xx = np.meshgrid(
        np.linspace(0, 1, N),
        np.linspace(0, 1, N),
        indexing="ij"
    )

    dist2 = (xx - cx) ** 2 + (yy - cy) ** 2

    pit = amplitude * np.exp(
        -dist2 / (2.0 * radius**2)
    )

    heightmap -= pit


# ============================================================
# Create a few larger terrain features
# ============================================================
num_features = 8

for _ in range(num_features):

    cx = np.random.uniform(0.1, 0.9)
    cy = np.random.uniform(0.1, 0.9)

    radius = np.random.uniform(0.03, 0.10)
    amplitude = np.random.uniform(-0.15, 0.20)

    yy, xx = np.meshgrid(
        np.linspace(0, 1, N),
        np.linspace(0, 1, N),
        indexing="ij"
    )

    dist = np.sqrt((xx - cx) ** 2 + (yy - cy) ** 2)

    feature = amplitude * np.exp(
        -(dist ** 2) / (2.0 * radius**2)
    )

    heightmap += feature


# ============================================================
# Normalize
# ============================================================
heightmap -= heightmap.min()

max_val = heightmap.max()

if max_val > 0:
    heightmap /= max_val

heightmap *= max_height


# ============================================================
# Flatten spawn area around center
# ============================================================
yy, xx = np.meshgrid(
    np.linspace(-1, 1, N),
    np.linspace(-1, 1, N),
    indexing="ij"
)

spawn_radius = 0.12

distance = np.sqrt(xx**2 + yy**2)

spawn_mask = distance < spawn_radius

# Set central region to a nearly flat surface
center_height = np.mean(heightmap[spawn_mask])

heightmap[spawn_mask] = center_height


# ============================================================
# Smooth transition around spawn region
# ============================================================
transition_radius = 0.22

mask = distance < transition_radius

alpha = np.clip(
    (distance - spawn_radius) /
    (transition_radius - spawn_radius),
    0.0,
    1.0
)

heightmap[mask] = (
    (1 - alpha[mask]) * center_height
    + alpha[mask] * heightmap[mask]
)


# ============================================================
# Clip extreme values
# ============================================================
heightmap = np.clip(
    heightmap,
    0.0,
    max_height
)


# ============================================================
# Convert to 8-bit grayscale PNG
# ============================================================
img = (
    (heightmap / max_height) * 255
).astype(np.uint8)

Image.fromarray(img, mode="L").save(
    "../models/terrain.png"
)

print("Terrain generated.")
print("Resolution:", N, "x", N)
print("Terrain size:", 2 * terrain_size, "x", 2 * terrain_size, "m")
print("Height range:", heightmap.min(), "to", heightmap.max())

print("min:", heightmap.min())
print("max:", heightmap.max())
print("mean:", heightmap.mean())
print("std:", heightmap.std())