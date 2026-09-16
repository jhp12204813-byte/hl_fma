# Obstacle positive dataset

This is the reviewed two-class YOLO dataset used to train
`models/obstacle_detector_candidate.pt`.

```text
0 child_dummy
1 vehicle_obstacle
```

`child_dummy` is one box around the visible child dummy **and its full trolley**
(hat, body, handle, platform, and wheels). Do not include an accompanying adult,
umbrella, or inferred out-of-frame extent. `vehicle_obstacle` covers the visible
vehicle-shaped obstacle body.

## Inventory

| Split | Images | child boxes | vehicle boxes | hard-negative images |
|---|---:|---:|---:|---:|
| train | 153 | 84 | 60 | 9 |
| val | 28 | 26 | 2 | 0 |

Labels are normalized YOLO `class cx cy width height`. The files committed under
`labels/` are the final manually reviewed `labels_clean` copy; older pre-review
labels, caches, and label-history backups are deliberately not committed.

Recording sessions are kept entirely in one split. Explicit child horizontal
mirrors remain in the same train session as their source and use the
`__hflip` suffix. They are augmentation derivatives, not independent sessions.
Do not randomly split individual frames.

The validation set has only two `vehicle_obstacle` boxes, so it is inadequate for
final vehicle performance selection. Add newly recorded hard-negative sessions and
an independent vehicle validation session without moving or repartitioning the
existing files.

Validate before training:

```bash
python autonomous_vision/train_obstacles.py \
  --data autonomous_vision/dataset_obstacles/data.yaml \
  --model autonomous_vision/models/obstacle_detector_candidate.pt \
  --epochs 50 --batch 4 --imgsz 640 --name obstacle_finetune_v2
```

Use a new run name. Do not replace the candidate checkpoint unless the new model
reduces class-agnostic obstacle FN without an unacceptable background-FP increase.
