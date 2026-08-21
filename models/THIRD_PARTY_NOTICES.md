# Third-party perception assets

The optional visual-perception challenge redistributes two official
Ultralytics assets:

- `yolo/yolov5n.onnx`: derived without retraining from the official
  `yolov5n.pt` COCO pretrained weight published by the
  [YOLOv5 v7.0 release](https://github.com/ultralytics/yolov5/releases/tag/v7.0);
  exported with the official v7.0 exporter, ONNX opset 12, 640 px input and
  raw `(1, 25200, 85)` output, then constant-folded with ONNX Simplifier
  `0.7.3` for OpenCV 4.5.4 compatibility.
- `perception_demo_poster/materials/textures/bus.jpg`: official demo image
  from the [Ultralytics Assets](https://github.com/ultralytics/assets)
  `v0.0.0` release.

YOLOv5 v7.0 source project license: GPL-3.0. These assets are included for the
non-commercial academic assessment and should be reviewed before reuse in a
different distribution context.

SHA-256:

- `yolov5n.onnx`:
  `ebfedcac4a87d78403de26fe3dc383c5c32ffe9d1ab3619d3aa709efd4b87cfb`
- `bus.jpg`:
  `c02019c4979c191eb739ddd944445ef408dad5679acab6fd520ef9d434bfbc63`
