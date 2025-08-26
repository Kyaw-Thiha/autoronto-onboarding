# Simple COCO analyzer: counts images/annotations, per-class freq, and size stats.
# No external deps; works with standard COCO-style JSON.

from __future__ import annotations
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import mean, median

from load_images import load_coco

TRAIN_IM_DIR = Path("data/train")
VALID_IM_DIR = Path("data/valid")
TRAIN_ANN = Path("data/annotations_train.coco.json")
VALID_ANN = Path("data/annotations_valid.coco.json")


def analyze_split(name: str, img_dir: Path, ann_path: Path):
    """
    Analyze a dataset split (train/valid) given its COCO annotations and image directory.

    Parameters
    ----------
    name : str
        Name of the split (e.g., "train" or "valid") used for printing and report filenames.
    img_dir : Path
        Directory containing the image files for this split.
    ann_path : Path
        Path to the annotation JSON file for this split.

    Returns
    -------
    None
        Prints analysis results to the console and writes reports to `reports/`.

    What it does
    ------------
    - Loads the COCO annotations via `load_coco`.
    - Counts images (in JSON vs on disk), annotations, and categories.
    - Computes per-category annotation frequencies.
    - Computes statistics (min, max, mean, median) for image widths, heights, and aspect ratios.
    - Prints the top 10 most frequent categories.
    - Writes two tab-separated report files in `reports/`:
        * `<split>_category_freq.tsv`  → category frequencies
        * `<split>_image_sizes.tsv`   → per-image width, height, aspect ratio
    """
    print(f"\n=== {name.upper()} ===")
    if not ann_path.exists():
        print(f"!! Missing annotations: {ann_path}")
        return
    images, ann, id2cat, id2img = load_coco(ann_path)

    # Basic counts
    n_images_json = len(images)
    n_images_disk = sum(1 for _ in img_dir.glob("*")) if img_dir.exists() else 0
    n_ann = len(ann)
    n_cats = len(id2cat)

    print(f"Images (in JSON): {n_images_json}")
    print(f"Images (found on disk in '{img_dir}'): {n_images_disk}")
    print(f"Annotations: {n_ann}")
    print(f"Categories: {n_cats}")

    # Per-category frequency
    cat_freq = Counter(id2cat.get(a["category_id"], "UNKNOWN") for a in ann)
    # Image-level size stats
    widths = []
    heights = []
    aspect_ratios = []
    for im in images:
        w = im.get("width")
        h = im.get("height")
        if isinstance(w, int) and isinstance(h, int) and w > 0 and h > 0:
            widths.append(w)
            heights.append(h)
            aspect_ratios.append(round(w / h, 4))

    def safe_stats(xs):
        return {
            "count": len(xs),
            "min": min(xs) if xs else None,
            "max": max(xs) if xs else None,
            "mean": round(mean(xs), 2) if xs else None,
            "median": round(median(xs), 2) if xs else None,
        }

    w_stats = safe_stats(widths)
    h_stats = safe_stats(heights)
    ar_stats = safe_stats(aspect_ratios)

    print("\n-- Image size stats --")
    print(f"Width : {w_stats}")
    print(f"Height: {h_stats}")
    print(f"Aspect ratio (W/H): {ar_stats}")

    # Top 10 categories by annotation count
    print("\n-- Top categories --")
    for cls, cnt in cat_freq.most_common(10):
        print(f"{cls:20s} {cnt}")

    # Optional: write simple TSVs
    out_dir = Path("reports")
    out_dir.mkdir(parents=True, exist_ok=True)

    # Category frequency TSV
    cat_tsv = out_dir / f"{name}_category_freq.tsv"
    with cat_tsv.open("w", encoding="utf-8") as f:
        f.write("category\tcount\n")
        for c, cnt in cat_freq.most_common():
            f.write(f"{c}\t{cnt}\n")

    # Image sizes TSV
    sizes_tsv = out_dir / f"{name}_image_sizes.tsv"
    with sizes_tsv.open("w", encoding="utf-8") as f:
        f.write("image_id\twidth\theight\taspect_ratio\n")
        for im in images:
            w = im.get("width")
            h = im.get("height")
            if isinstance(w, int) and isinstance(h, int) and w > 0 and h > 0:
                ar = w / h
                f.write(f"{im.get('id')}\t{w}\t{h}\t{ar:.6f}\n")

    print(f"\nSaved: {cat_tsv} and {sizes_tsv}")


def main():
    analyze_split("train", TRAIN_IM_DIR, TRAIN_ANN)
    analyze_split("valid", VALID_IM_DIR, VALID_ANN)


if __name__ == "__main__":
    main()
