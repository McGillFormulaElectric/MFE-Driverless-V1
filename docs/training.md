# YOLO Training

Training YOLO cone detectors on the FSOCO dataset, tuning for RTX 5060 Ti, and deploying weights to the ROS 2 vision node.

---

## Overview

The training pipeline (`training/yolo/`) trains multiple YOLO model variants on the FSOCO cone dataset and compares their mAP. The best model weights are automatically copied to a fixed path for the ROS node.

**Models trained**: `yolov8n`, `yolov8s`, `yolo11n`, `yolo11s`  
**Dataset**: [FSOCO](https://github.com/fsoco/fsoco-dataset) (Formula Student cone images, YOLO format)  
**Hardware target**: NVIDIA RTX 5060 Ti (16 GB VRAM)

---

## Dataset preparation

```bash
python3 training/yolo/prepare_dataset.py
```

This downloads and converts the FSOCO dataset to YOLO format. Output goes to `~/mfe_datasets/fsoco_yolo/`.

Expected structure:
```
~/mfe_datasets/fsoco_yolo/
├── data.yaml        ← class names + train/val paths
├── images/
│   ├── train/
│   └── val/
└── labels/
    ├── train/
    └── val/
```

**Classes** (cone colors): `blue`, `yellow`, `orange`

---

## Training

```bash
cd training/yolo
python3 train.py \
  [--data ~/mfe_datasets/fsoco_yolo/data.yaml] \
  [--epochs 100] \
  [--batch 32] \
  [--imgsz 640] \
  [--models yolov8n yolov8s yolo11n yolo11s] \
  [--output ~/mfe_models/yolo]
```

All arguments are optional — defaults shown above.

To train a single model:
```bash
python3 train.py --models yolov8s --epochs 100 --batch 128
```

### GPU tuning for RTX 5060 Ti

| Setting | Value | Reason |
|---------|-------|--------|
| `batch` | `128` | Fills ~13.4 GB VRAM at ~80% GPU utilization |
| `workers` | `2` | Hard-coded in `train.py`; `workers=8` thrashes swap, drops GPU to 3% |
| `device` | `0` | Single GPU |
| `patience` | `20` | Early stop after 20 epochs without improvement |

**Do not change `workers`** — the sim containers running alongside training add memory pressure. `workers=2` was tuned specifically for this coexistence.

### Augmentation settings

```python
hsv_h=0.015, hsv_s=0.7, hsv_v=0.4   # color jitter
degrees=5.0, translate=0.1, scale=0.5 # geometric
fliplr=0.5, mosaic=1.0, mixup=0.1     # mosaic + mixup
```

---

## Outputs

Training writes to `~/mfe_models/yolo/<model_name>/`:
```
~/mfe_models/yolo/
├── yolov8s/
│   └── weights/
│       ├── best.pt     ← best checkpoint (mAP)
│       └── last.pt
├── comparison_results.json
└── cone_detector_best.pt   ← auto-copied best across all models
```

After training completes, `train.py`:
1. Prints a comparison table (mAP50, mAP50-95, precision, recall, training time).
2. Identifies the model with the highest mAP50.
3. Copies its `best.pt` to `~/mfe_models/yolo/cone_detector_best.pt`.
4. Saves full metrics to `~/mfe_models/yolo/comparison_results.json`.

---

## Using weights in ROS 2

### Default path (used by bringup.launch.py)

The default `vision_model_path` argument in `bringup.launch.py` is:
```
~/mfe_models/yolo/yolov8s/weights/best.pt
```

### Custom path

```bash
ros2 launch mfe_bringup bringup.launch.py \
  vision_model_path:=~/mfe_models/yolo/cone_detector_best.pt
```

### Disabling vision (LiDAR only)

```bash
ros2 launch mfe_bringup bringup.launch.py vision_model_path:=''
```

When `vision_model_path` is empty, the vision node is skipped entirely at launch (it would crash if started without weights because `YOLO(model_path)` is called in `__init__`).

---

## Pre-downloaded weights

The following base model weights are included in `training/yolo/` (used as starting points for fine-tuning):

| File | Parameters | Use case |
|------|-----------|---------|
| `yolov8n.pt` | 3.2M | Fastest, lowest accuracy |
| `yolov8s.pt` | 11.2M | Good speed/accuracy balance |
| `yolo11n.pt` | ~3M | Newer architecture, fast |
| `yolo11s.pt` | ~9M | Newer architecture, balanced |
| `yolo26n.pt` | ~3M | YOLOv26 nano |

---

## Inference pipeline (ROS 2)

```
/camera/image_raw (Image)
        │
        ▼
cone_detection_node (YOLO11 inference)
        │
        ▼
image/track (mfe_msgs/Track)
  └── Cone.BLUE    — blue boundary cones
  └── Cone.YELLOW  — yellow boundary cones
  └── Cone.ORANGE_SMALL — orange cones
        │
        ▼
boundary_extractor (fuses with LiDAR positions)
        │
        ▼
/planning/cones (mfe_msgs/Track)
```

---

## Expected metrics (FSOCO, yolov8s, 100 epochs)

Approximate ranges based on FSOCO benchmarks:

| Metric | Typical range |
|--------|--------------|
| mAP50 | 0.85 – 0.95 |
| mAP50-95 | 0.60 – 0.75 |
| Precision | 0.85 – 0.93 |
| Recall | 0.83 – 0.92 |
| Training time | 45 – 90 min (RTX 5060 Ti, batch=128) |
