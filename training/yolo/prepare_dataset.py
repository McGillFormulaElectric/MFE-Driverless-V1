"""
Convert FSOCO bounding-box dataset (Supervisely JSON) → YOLO format.

Input layout (Supervisely):
    fsoco_bounding_boxes_train/
        <team>/
            img/  *.png / *.jpg
            ann/  *.json   ← one JSON per image

Output layout (YOLO):
    ~/mfe_datasets/fsoco_yolo/
        images/train/  val/  test/
        labels/train/  val/  test/
        data.yaml

Classes (rectangle bboxes only — seg_* bitmaps skipped):
    0  blue_cone
    1  yellow_cone
    2  orange_cone
    3  large_orange_cone
    4  unknown_cone

Usage:
    python3 prepare_dataset.py [--src ~/Downloads/fsoco_bounding_boxes_train]
                               [--dst ~/mfe_datasets/fsoco_yolo]
                               [--val-teams eufs epflrt]
                               [--seed 42]
"""

import argparse
import json
import os
import random
import shutil
from pathlib import Path

# ---- class mapping -------------------------------------------------------- #
CLASSES = {
    'blue_cone':         0,
    'yellow_cone':       1,
    'orange_cone':       2,
    'large_orange_cone': 3,
    'unknown_cone':      4,
}
CLASS_NAMES = ['blue_cone', 'yellow_cone', 'orange_cone',
               'large_orange_cone', 'unknown_cone']

# Teams reserved for validation/test (real-world diverse cameras)
DEFAULT_VAL_TEAMS  = ['eufs', 'epflrt']
DEFAULT_TEST_TEAMS = ['ka']


def ann_to_yolo(ann_path: Path, img_w: int, img_h: int) -> list[str]:
    """Convert one Supervisely JSON annotation to YOLO label lines."""
    with open(ann_path) as f:
        data = json.load(f)

    lines = []
    for obj in data.get('objects', []):
        cls_title = obj.get('classTitle', '')
        if cls_title not in CLASSES:
            continue
        if obj.get('geometryType') != 'rectangle':
            continue

        pts  = obj['points']['exterior']
        x1, y1 = pts[0]
        x2, y2 = pts[1]

        # Clamp to image bounds
        x1 = max(0, min(x1, img_w))
        x2 = max(0, min(x2, img_w))
        y1 = max(0, min(y1, img_h))
        y2 = max(0, min(y2, img_h))

        if x2 <= x1 or y2 <= y1:
            continue

        cx = (x1 + x2) / 2 / img_w
        cy = (y1 + y2) / 2 / img_h
        w  = (x2 - x1) / img_w
        h  = (y2 - y1) / img_h

        lines.append(f'{CLASSES[cls_title]} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}')

    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--src',  default=os.path.expanduser(
        '~/Downloads/fsoco_bounding_boxes_train'))
    parser.add_argument('--dst',  default=os.path.expanduser(
        '~/mfe_datasets/fsoco_yolo'))
    parser.add_argument('--val-split',  type=float, default=0.2,
                        help='Fraction of data for validation (default 0.2 = 80/20)')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    random.seed(args.seed)

    # ------------------------------------------------------------------ #
    # Discover all (image, annotation) pairs
    # ------------------------------------------------------------------ #
    samples = []   # list of (split, img_path, ann_path, img_w, img_h)

    for team_dir in sorted(src.iterdir()):
        if not team_dir.is_dir():
            continue
        team = team_dir.name
        img_dir = team_dir / 'img'
        ann_dir = team_dir / 'ann'
        if not img_dir.exists() or not ann_dir.exists():
            continue

        split = 'train'   # assigned later after full shuffle

        for ann_path in sorted(ann_dir.glob('*.json')):
            with open(ann_path) as f:
                meta = json.load(f)
            img_w = meta['size']['width']
            img_h = meta['size']['height']

            # Find matching image (strip .json suffix)
            img_stem = ann_path.stem   # e.g. "eufs_00000.png"
            img_path = img_dir / img_stem
            if not img_path.exists():
                # Try without extension in case stem already has it
                candidates = list(img_dir.glob(img_stem.rsplit('.', 1)[0] + '.*'))
                if not candidates:
                    continue
                img_path = candidates[0]

            # Only include images that have at least one usable annotation
            lines = ann_to_yolo(ann_path, img_w, img_h)
            if not lines:
                continue

            samples.append((split, img_path, ann_path, img_w, img_h, lines))

    # 80/20 random split
    random.shuffle(samples)
    n_val = int(len(samples) * args.val_split)
    val   = samples[:n_val]
    train = samples[n_val:]
    test  = []   # no separate test split

    # Re-tag split field
    train = [(('train',) + s[1:]) for s in train]
    val   = [(('val',)   + s[1:]) for s in val]

    print(f'Dataset split (80/20):')
    print(f'  train: {len(train)} images')
    print(f'  val:   {len(val)} images')
    print(f'  total: {len(samples)} images')

    # ------------------------------------------------------------------ #
    # Write files
    # ------------------------------------------------------------------ #
    for split_name, split_samples in [('train', train), ('val', val), ('test', test)]:
        (dst / 'images' / split_name).mkdir(parents=True, exist_ok=True)
        (dst / 'labels' / split_name).mkdir(parents=True, exist_ok=True)

        class_counts = {c: 0 for c in CLASS_NAMES}

        for (split, img_path, ann_path, img_w, img_h, lines) in split_samples:
            # Copy image
            out_img = dst / 'images' / split_name / img_path.name
            shutil.copy2(img_path, out_img)

            # Write YOLO label
            out_lbl = dst / 'labels' / split_name / (img_path.stem.rsplit('.', 1)[0] + '.txt')
            out_lbl.write_text('\n'.join(lines))

            for line in lines:
                cls_idx = int(line.split()[0])
                class_counts[CLASS_NAMES[cls_idx]] += 1

        print(f'\n{split_name} class distribution:')
        for name, count in class_counts.items():
            print(f'  {name}: {count}')

    # ------------------------------------------------------------------ #
    # Write data.yaml
    # ------------------------------------------------------------------ #
    yaml_content = f"""# FSOCO cone detection dataset — converted from Supervisely format
path: {dst}
train: images/train
val:   images/val
test:  images/test

nc: {len(CLASS_NAMES)}
names: {CLASS_NAMES}
"""
    (dst / 'data.yaml').write_text(yaml_content)
    print(f'\nDataset written to: {dst}')
    print(f'data.yaml:          {dst / "data.yaml"}')


if __name__ == '__main__':
    main()
