# child+trolley clean-label 50-epoch training

## Inputs and preservation

- Dataset snapshot: `autonomous_vision/dataset_obstacles_childclean`
- Classes: `0 child_dummy`, `1 vehicle_obstacle`
- Train: 153 images (child 84 boxes, vehicle 60 boxes)
- Val: 28 images (child 26 boxes, vehicle 2 boxes)
- Settings: YOLO11n, 640 px, batch 4, AdamW, lr0 0.001, 50 epochs
- Horizontal flip augmentation disabled; the 42 explicitly generated child flips remain dataset samples.
- Original dataset, `labels/`, `data.yaml`, and previous weights were not replaced.

## Training result

Training completed successfully. Ultralytics selected epoch 12 as `best.pt` by validation
fitness. Its built-in validation metrics were:

| Scope | Precision | Recall | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| all | 0.343 | 0.442 | 0.358 | 0.164 |
| child_dummy | 0.471 | 0.385 | 0.468 | 0.251 |
| vehicle_obstacle | 0.216 | 0.500 | 0.247 | 0.0766 |

## Fixed-threshold comparison on the same clean val labels

Matching uses confidence 0.25 and IoU 0.50. All three weights were evaluated against
the same `dataset_obstacles_childclean` validation set.

| Weight | child TP/FP/FN | child F1 | vehicle TP/FP/FN | vehicle F1 |
|---|---:|---:|---:|---:|
| new `obstacle_childclean_50ep` | 9/9/17 | 0.4091 | 0/1/2 | 0.0000 |
| previous `obstacle_childflip_50ep` | 1/8/25 | 0.0571 | 1/0/1 | 0.6667 |
| previous `obstacle_50ep` | 9/15/17 | 0.3600 | 2/12/0 | 0.2500 |

The new model improves child F1 over the original model at this threshold, but it is
not safe to promote as a common obstacle model yet: vehicle validation has only two
boxes and neither is a TP at confidence 0.25. At confidence 0.05 it finds one vehicle
but produces 17 vehicle false positives. More independent vehicle validation data is
needed before model/threshold selection.

## Artifacts

- `weights/best.pt`: selected checkpoint
- `weights/last.pt`: final epoch checkpoint
- `dataset_snapshot.json`: exact training input hashes
- `custom_eval/review.json`: TP/FP/FN evaluation details
- `custom_eval/*.jpg`: ground-truth/prediction comparison sheets

No default/deployment weight was replaced and no ROS2 control code was changed.

## Avoidance-priority evaluation

For vehicle control, both classes were collapsed into one `OBSTACLE_AVOIDANCE`
category. A prediction of the wrong obstacle subclass is therefore still a TP. NMS
was also class-agnostic. Full threshold results are stored in
`avoidance_priority_eval.json`.

| Candidate threshold | TP | FP | FN | Recall | Precision |
|---:|---:|---:|---:|---:|---:|
| 0.01 | 27 | 762 | 1 | 0.964 | 0.034 |
| 0.03 | 21 | 115 | 7 | 0.750 | 0.154 |
| 0.05 | 19 | 62 | 9 | 0.679 | 0.235 |
| 0.07 | 18 | 46 | 10 | 0.643 | 0.281 |
| 0.25 | 10 | 8 | 18 | 0.357 | 0.556 |

Because FN reduction is the first priority, confidence 0.25 must not be used as the
only rejection gate. A low threshold should retain candidates, followed by aligned
D435i depth validity, collision-corridor overlap, and short temporal confirmation to
remove background FP. Threshold 0.01 alone cannot trigger avoidance because it creates
about 27 FP per validation image; it is only suitable as a first-stage candidate floor.

The next offline validation should test a two-stage policy around candidate thresholds
0.01-0.03. Distance should come from robust valid-depth samples inside the candidate,
not bbox size. Left/center/right free-space depth must then select a safe side. Only
after recorded-video validation should this policy be connected to ROS2 control.
