"""
Note: The test cases have been generated through ChatGPT.

This script creates a suite of image/CSV pairs under ./testcases covering
happy-path, steep slope, too-few-points (via horizon cropping), vertical line
(SS_tot == 0), noisy/low-R², dilation sensitivity, no-overlap, and curved lanes.

Each test case directory contains:
  - mask.png   : binary mask image (uint8) with foreground pixels = 255
  - prior.csv  : sparse polyline points (u, v) as integers

Run:
    python3 gen_test.py
"""

from __future__ import annotations

import csv
import os
from typing import List, Sequence, Tuple, Optional

import cv2
import numpy as np
from numpy.typing import NDArray

# --------------------------- Constants & RNG ---------------------------------

H: int = 480
W: int = 640
Y0: int = int(0.5 * H)  # draw only on the bottom half
RNG: np.random.Generator = np.random.default_rng(42)  # reproducible


# A typed alias for OpenCV grayscale images (uint8)
NDArrayU8 = NDArray[np.uint8]


# ------------------------------ Utilities ------------------------------------


def ensure_dir(path: str) -> None:
    """Ensure a directory exists.

    Args:
        path: Directory path to create if missing.
    """
    os.makedirs(path, exist_ok=True)


def clamp_x(x: float) -> int:
    """Clamp an x-coordinate to the valid [0, W-1] range.

    Args:
        x: Floating-point x value.

    Returns:
        The clamped integer x within image bounds.
    """
    xi = int(round(x))
    if xi < 0:
        return 0
    if xi >= W:
        return W - 1
    return xi


def write_prior_csv(path: str, pts_uv: Sequence[Tuple[int, int]]) -> None:
    """Write (u,v) prior polyline points to CSV.

    Args:
        path: Output CSV path.
        pts_uv: Sequence of points (u, v) in pixel units.
    """
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        for u, v in pts_uv:
            writer.writerow([int(u), int(v)])


# ----------------------- Prior (polyline) generators -------------------------


def line_points_for_prior(
    x_center: float,
    slope: float,
    y_start: int = Y0,
    y_end: Optional[int] = None,
    n: int = 25,
) -> List[Tuple[int, int]]:
    """Generate a sparse straight-line prior polyline.

    The line equation (in image coordinates) is:
        x(y) = x_center + slope * (y - y_start)

    Args:
        x_center: x at y == y_start.
        slope: Pixels of x per pixel of y.
        y_start: Inclusive starting y for sampling.
        y_end: Inclusive ending y for sampling (defaults to H-1).
        n: Number of evenly spaced points from y_start..y_end.

    Returns:
        List of (u, v) integer pixel coordinates.
    """
    if y_end is None:
        y_end = H - 1
    ys = np.linspace(y_start, y_end, n).astype(int)
    pts: List[Tuple[int, int]] = []
    for y in ys:
        x = x_center + slope * (y - y_start)
        pts.append((clamp_x(x), int(y)))
    return pts


def vertical_points_for_prior(
    x_const: int,
    y_start: int = Y0,
    y_end: Optional[int] = None,
    n: int = 25,
) -> List[Tuple[int, int]]:
    """Generate a sparse vertical prior polyline at fixed x.

    Args:
        x_const: Constant x position of the vertical line.
        y_start: Inclusive starting y for sampling.
        y_end: Inclusive ending y for sampling (defaults to H-1).
        n: Number of evenly spaced points from y_start..y_end.

    Returns:
        List of (u, v) integer pixel coordinates.
    """
    if y_end is None:
        y_end = H - 1
    ys = np.linspace(y_start, y_end, n).astype(int)
    return [(int(x_const), int(y)) for y in ys]


# ----------------------------- Mask drawing ----------------------------------


def draw_mask_line(
    img: NDArrayU8,
    x_center: float,
    slope: float,
    *,
    noise_std: float = 0.0,
    thickness: int = 1,
    y_start: int = Y0,
    y_end: Optional[int] = None,
) -> None:
    """Draw a noisy straight line into a binary mask image.

    The line is drawn as a set of filled circles along the model x(y).

    Args:
        img: Target image (H×W, uint8). Will be modified in-place; foreground=255.
        x_center: x at y == y_start.
        slope: Pixels of x per pixel of y.
        noise_std: Gaussian noise std-dev (pixels) added to x.
        thickness: Circle radius (in pixels) for each plotted point.
        y_start: Inclusive starting y to draw.
        y_end: Inclusive ending y to draw (defaults to H-1).
    """
    if y_end is None:
        y_end = H - 1
    for y in range(y_start, y_end + 1):
        x = x_center + slope * (y - y_start)
        if noise_std > 0.0:
            x += float(RNG.normal(0.0, noise_std))
        cv2.circle(img, (clamp_x(x), int(y)), thickness, 255, -1)


def draw_mask_vertical(
    img: NDArrayU8,
    x_const: int,
    *,
    thickness: int = 1,
    y_start: int = Y0,
    y_end: Optional[int] = None,
) -> None:
    """Draw a vertical line into a binary mask image.

    Args:
        img: Target image (H×W, uint8). Will be modified in-place; foreground=255.
        x_const: Constant x position of the vertical line.
        thickness: Circle radius (in pixels) for each plotted point.
        y_start: Inclusive starting y to draw.
        y_end: Inclusive ending y to draw (defaults to H-1).
    """
    if y_end is None:
        y_end = H - 1
    for y in range(y_start, y_end + 1):
        cv2.circle(img, (int(x_const), int(y)), thickness, 255, -1)


def draw_mask_curved(
    img: NDArrayU8,
    x_center: float,
    slope: float,
    k_quad: float,
    *,
    thickness: int = 1,
    y_start: int = Y0,
    y_end: Optional[int] = None,
) -> None:
    """Draw a quadratic (curved) lane into a binary mask image.

    Uses the model:
        x(y) = x_center + slope * (y - y_start) + k_quad * (y - y_start)^2

    Args:
        img: Target image (H×W, uint8). Will be modified in-place; foreground=255.
        x_center: x at y == y_start (baseline).
        slope: Linear term (pixels of x per pixel of y).
        k_quad: Quadratic curvature coefficient.
        thickness: Circle radius (in pixels) for each plotted point.
        y_start: Inclusive starting y to draw.
        y_end: Inclusive ending y to draw (defaults to H-1).
    """
    if y_end is None:
        y_end = H - 1
    for y in range(y_start, y_end + 1):
        dy = float(y - y_start)
        x = x_center + slope * dy + k_quad * (dy**2)
        cv2.circle(img, (clamp_x(x), int(y)), thickness, 255, -1)


# --------------------------- Case I/O helpers --------------------------------


def save_case(
    folder: str, img: NDArrayU8, prior_pts: Sequence[Tuple[int, int]]
) -> None:
    """Save an image and matching prior CSV to a directory.

    Args:
        folder: Output directory for the case.
        img: Binary mask image (H×W, uint8).
        prior_pts: Sparse prior polyline points (u, v).
    """
    ensure_dir(folder)
    mask_path = os.path.join(folder, "mask.png")
    prior_path = os.path.join(folder, "prior.csv")
    ok = cv2.imwrite(mask_path, img)
    if not ok:
        raise RuntimeError(f"Failed to write {mask_path}")
    write_prior_csv(prior_path, prior_pts)


# ------------------------------ Generation -----------------------------------


def gen_all(base_dir: str = "testcases") -> None:
    """Generate all predefined test cases.

    Creates subfolders under `base_dir`:
        01_good_line, 02_steeper_line, 03_few_points_horizon, 04_vertical_line,
        05_noisy_low_r2, 06_needs_dilation, 07_wrong_prior, 08_curved_lane.

    Args:
        base_dir: Root directory under which test case folders are created.
    """
    ensure_dir(base_dir)

    # 01_good_line
    img01: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    slope: float = 0.22
    x_center: int = W // 2 + 10
    draw_mask_line(img01, x_center, slope, noise_std=0.0, thickness=1)
    prior01 = line_points_for_prior(x_center, slope)
    save_case(os.path.join(base_dir, "01_good_line"), img01, prior01)

    # 02_steeper_line
    img02: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    slope2: float = 0.40
    x_center2: int = W // 2 - 30
    draw_mask_line(img02, x_center2, slope2, noise_std=0.0, thickness=1)
    prior02 = line_points_for_prior(x_center2, slope2)
    save_case(os.path.join(base_dir, "02_steeper_line"), img02, prior02)

    # 03_few_points_horizon (same geometry as 01; horizon varied at runtime)
    img03: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    draw_mask_line(img03, x_center, slope, noise_std=0.0, thickness=1)
    prior03 = line_points_for_prior(x_center, slope)
    save_case(os.path.join(base_dir, "03_few_points_horizon"), img03, prior03)

    # 04_vertical_line
    img04: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    x_vert: int = W // 2 + 40
    draw_mask_vertical(img04, x_vert, thickness=1)
    prior04 = vertical_points_for_prior(x_vert)
    save_case(os.path.join(base_dir, "04_vertical_line"), img04, prior04)

    # 05_noisy_low_r2
    img05: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    draw_mask_line(img05, x_center, slope, noise_std=12.0, thickness=1)
    prior05 = line_points_for_prior(x_center, slope)
    save_case(os.path.join(base_dir, "05_noisy_low_r2"), img05, prior05)

    # 06_needs_dilation (mask shifted +12 px; prior unshifted)
    img06: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    delta: int = 12
    draw_mask_line(img06, x_center + delta, slope, noise_std=0.0, thickness=1)
    prior06 = line_points_for_prior(x_center, slope)  # unshifted
    save_case(os.path.join(base_dir, "06_needs_dilation"), img06, prior06)

    # 07_wrong_prior (far apart)
    img07: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    mask_center: int = W // 2 + 100
    prior_center: int = W // 2 - 100
    draw_mask_line(img07, mask_center, slope, noise_std=0.0, thickness=1)
    prior07 = line_points_for_prior(prior_center, slope)
    save_case(os.path.join(base_dir, "07_wrong_prior"), img07, prior07)

    # 08_curved_lane (quadratic)
    img08: NDArrayU8 = np.zeros((H, W), dtype=np.uint8)
    k_quad: float = 0.002  # curvature coefficient
    draw_mask_curved(img08, x_center, slope, k_quad, thickness=1)
    prior08 = line_points_for_prior(x_center, slope)  # straight prior
    save_case(os.path.join(base_dir, "08_curved_lane"), img08, prior08)

    print(f"Generated testcases in ./{base_dir}\n")
    print("Summary:")
    rows: List[Tuple[str, str]] = [
        ("01_good_line", "happy path"),
        ("02_steeper_line", "different slope"),
        ("03_few_points_horizon", "trip <20 pts via high horizon_frac"),
        ("04_vertical_line", "SS_tot=0 -> R² guard"),
        ("05_noisy_low_r2", "low R² path"),
        ("06_needs_dilation", "small vs large --roi"),
        ("07_wrong_prior", "no overlap"),
        ("08_curved_lane", "nonlinear lane"),
    ]
    for rid, desc in rows:
        print(f"- {rid}: {desc}")


# --------------------------------- Main --------------------------------------

if __name__ == "__main__":
    gen_all()
