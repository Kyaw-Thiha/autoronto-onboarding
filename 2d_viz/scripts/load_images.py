from pathlib import Path
import json


def load_coco(path: Path):
    """
    Load a COCO annotation JSON file and return useful structures.

    Parameters
    ----------
    path : Path
        Path to the COCO annotation JSON file.

    Returns
    -------
    images : list[dict]
        List of image entries, each with metadata (id, width, height, file_name, etc.).
    annotations : list[dict]
        List of annotation entries (bounding boxes, segmentation, category_id, etc.).
    id_to_cat : dict[int, str]
        Mapping from category ID to category name.
    id_to_img : dict[int, dict]
        Mapping from image ID to the corresponding image metadata dictionary.
    """
    with path.open("r", encoding="utf-8") as f:
        coco = json.load(f)
    # Basic schema checks
    images = coco.get("images", [])
    annotations = coco.get("annotations", [])
    categories = coco.get("categories", [])
    id_to_cat = {c["id"]: c["name"] for c in categories}
    id_to_img = {im["id"]: im for im in images}
    for k in coco.keys():
        print(" -", k)
    return images, annotations, id_to_cat, id_to_img
