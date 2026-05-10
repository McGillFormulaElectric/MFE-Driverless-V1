"""
Train YOLOv8n, YOLOv8s, YOLO11n, YOLO11s on FSOCO and compare results.

Usage:
    python3 train.py [--data ~/mfe_datasets/fsoco_yolo/data.yaml]
                     [--epochs 100] [--batch 32] [--imgsz 640]
                     [--models yolov8n yolov8s yolo11n yolo11s]
                     [--output ~/mfe_models/yolo]
"""

import argparse
import json
import os
import time
from pathlib import Path


def train_model(model_name: str, data_yaml: str, epochs: int,
                batch: int, imgsz: int, output_dir: Path):
    from ultralytics import YOLO

    print(f'\n{"="*60}')
    print(f'Training {model_name}')
    print(f'{"="*60}')

    model = YOLO(f'{model_name}.pt')
    t0 = time.time()

    results = model.train(
        data       = data_yaml,
        epochs     = epochs,
        batch      = batch,
        imgsz      = imgsz,
        project    = str(output_dir),
        name       = model_name,
        exist_ok   = True,
        device     = 0,            # GPU 0 (RTX 5060 Ti)
        workers    = 2,            # low to avoid swap thrash when sim containers are running
        patience   = 20,           # early stop if no improvement for 20 epochs
        save       = True,
        plots      = True,
        verbose    = False,
        # Augmentation
        hsv_h      = 0.015,
        hsv_s      = 0.7,
        hsv_v      = 0.4,
        degrees    = 5.0,
        translate  = 0.1,
        scale      = 0.5,
        fliplr     = 0.5,
        mosaic     = 1.0,
        mixup      = 0.1,
    )

    elapsed = time.time() - t0

    # Extract key metrics from results
    metrics = {
        'model':          model_name,
        'epochs_run':     int(results.epoch) + 1 if hasattr(results, 'epoch') else epochs,
        'train_time_min': round(elapsed / 60, 1),
        'mAP50':          round(float(results.results_dict.get('metrics/mAP50(B)', 0)), 4),
        'mAP50-95':       round(float(results.results_dict.get('metrics/mAP50-95(B)', 0)), 4),
        'precision':      round(float(results.results_dict.get('metrics/precision(B)', 0)), 4),
        'recall':         round(float(results.results_dict.get('metrics/recall(B)', 0)), 4),
        'best_weights':   str(output_dir / model_name / 'weights' / 'best.pt'),
    }

    print(f'\n{model_name} results:')
    for k, v in metrics.items():
        print(f'  {k}: {v}')

    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data',   default=os.path.expanduser(
        '~/mfe_datasets/fsoco_yolo/data.yaml'))
    parser.add_argument('--epochs', type=int,   default=100)
    parser.add_argument('--batch',  type=int,   default=32)
    parser.add_argument('--imgsz',  type=int,   default=640)
    parser.add_argument('--models', nargs='+',
        default=['yolov8n', 'yolov8s', 'yolo11n', 'yolo11s'])
    parser.add_argument('--output', default=os.path.expanduser(
        '~/mfe_models/yolo'))
    args = parser.parse_args()

    data_yaml  = os.path.expanduser(args.data)
    output_dir = Path(os.path.expanduser(args.output))
    output_dir.mkdir(parents=True, exist_ok=True)

    if not Path(data_yaml).exists():
        print(f'ERROR: data.yaml not found at {data_yaml}')
        print('Run prepare_dataset.py first.')
        return

    print(f'Training {len(args.models)} models: {args.models}')
    print(f'Epochs: {args.epochs}  Batch: {args.batch}  ImgSz: {args.imgsz}')
    print(f'Data: {data_yaml}')
    print(f'Output: {output_dir}')

    all_metrics = []

    for model_name in args.models:
        try:
            metrics = train_model(
                model_name = model_name,
                data_yaml  = data_yaml,
                epochs     = args.epochs,
                batch      = args.batch,
                imgsz      = args.imgsz,
                output_dir = output_dir,
            )
            all_metrics.append(metrics)
        except Exception as e:
            print(f'ERROR training {model_name}: {e}')
            all_metrics.append({'model': model_name, 'error': str(e)})

    # ------------------------------------------------------------------ #
    # Comparison table
    # ------------------------------------------------------------------ #
    print(f'\n{"="*70}')
    print('RESULTS COMPARISON')
    print(f'{"="*70}')
    print(f'{"Model":<12} {"mAP50":>8} {"mAP50-95":>10} {"Precision":>10} {"Recall":>8} {"Time(min)":>10}')
    print('-' * 70)

    best_map50  = 0
    best_model  = None

    for m in all_metrics:
        if 'error' in m:
            print(f'{m["model"]:<12}  ERROR: {m["error"]}')
            continue
        print(f'{m["model"]:<12} {m["mAP50"]:>8.4f} {m["mAP50-95"]:>10.4f} '
              f'{m["precision"]:>10.4f} {m["recall"]:>8.4f} {m["train_time_min"]:>10.1f}')
        if m['mAP50'] > best_map50:
            best_map50 = m['mAP50']
            best_model = m

    print('-' * 70)
    if best_model:
        print(f'\nBest model: {best_model["model"]}  (mAP50={best_map50:.4f})')
        print(f'Best weights: {best_model["best_weights"]}')

        # Copy best weights to a fixed path for the ROS node
        import shutil
        ros_weights = Path(os.path.expanduser(
            '~/mfe_models/yolo/cone_detector_best.pt'))
        shutil.copy2(best_model['best_weights'], ros_weights)
        print(f'Copied to:    {ros_weights}')
        print(f'\nTo use in ROS:')
        print(f'  ros2 launch mfe_bringup bringup.launch.py '
              f'vision_model_path:={ros_weights}')

    # Save full results JSON
    results_path = output_dir / 'comparison_results.json'
    results_path.write_text(json.dumps(all_metrics, indent=2))
    print(f'\nFull results saved to: {results_path}')


if __name__ == '__main__':
    main()
