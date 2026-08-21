# Copyright 2026 zzr
# SPDX-License-Identifier: Apache-2.0

"""OpenCV-DNN inference helpers for the bundled COCO YOLO model."""

from dataclasses import dataclass
from dataclasses import replace
from typing import Iterable
from typing import Optional

import cv2
import numpy as np


COCO_CLASS_NAMES = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
    'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
    'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
    'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
    'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
    'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
    'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
    'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
    'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
    'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush',
)


@dataclass(frozen=True)
class Detection:
    """One image-space object detection."""

    class_id: int
    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int
    distance_m: Optional[float] = None

    @property
    def width(self):
        """Return the clamped bounding-box width."""
        return max(0, self.x2 - self.x1)

    @property
    def height(self):
        """Return the clamped bounding-box height."""
        return max(0, self.y2 - self.y1)

    def with_distance(self, distance_m):
        """Return a copy carrying an RGB-D range estimate."""
        return replace(self, distance_m=distance_m)


def letterbox(image, image_size=640, fill_value=114):
    """Resize without distortion and return image, scale, left and top."""
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected a BGR image with shape HxWx3.')
    if image_size <= 0:
        raise ValueError('image_size must be positive.')

    height, width = image.shape[:2]
    scale = min(image_size / width, image_size / height)
    resized_width = max(1, round(width * scale))
    resized_height = max(1, round(height * scale))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )

    left = (image_size - resized_width) // 2
    top = (image_size - resized_height) // 2
    canvas = np.full(
        (image_size, image_size, 3),
        fill_value,
        dtype=np.uint8,
    )
    canvas[
        top:top + resized_height,
        left:left + resized_width,
    ] = resized
    return canvas, scale, left, top


class OpenCVDnnYolo:
    """Run a raw YOLO detection ONNX graph through OpenCV DNN."""

    def __init__(
        self,
        model_path,
        confidence_threshold=0.25,
        iou_threshold=0.45,
        image_size=640,
        max_detections=50,
        class_filter=None,
    ):
        if not 0.0 < confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be in (0, 1].')
        if not 0.0 < iou_threshold <= 1.0:
            raise ValueError('iou_threshold must be in (0, 1].')
        if image_size <= 0 or image_size % 32:
            raise ValueError('image_size must be a positive multiple of 32.')
        if max_detections <= 0:
            raise ValueError('max_detections must be positive.')

        self.model_path = str(model_path)
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.image_size = int(image_size)
        self.max_detections = int(max_detections)
        self.allowed_labels = self._normalise_filter(class_filter)
        self.net = cv2.dnn.readNetFromONNX(self.model_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    @staticmethod
    def _normalise_filter(class_filter):
        if not class_filter or '*' in class_filter:
            return None
        unknown = set(class_filter) - set(COCO_CLASS_NAMES)
        if unknown:
            raise ValueError(
                'Unknown COCO class labels: ' + ', '.join(sorted(unknown))
            )
        return set(class_filter)

    def detect(self, bgr_image):
        """Return post-NMS detections in coordinates of the input image."""
        padded, scale, left, top = letterbox(
            bgr_image,
            image_size=self.image_size,
        )
        blob = cv2.dnn.blobFromImage(
            padded,
            scalefactor=1.0 / 255.0,
            size=(self.image_size, self.image_size),
            swapRB=True,
            crop=False,
        )
        self.net.setInput(blob)
        output = self.net.forward()
        predictions = self._prediction_rows(output)
        return self._postprocess(
            predictions,
            bgr_image.shape[1],
            bgr_image.shape[0],
            scale,
            left,
            top,
        )

    @staticmethod
    def _prediction_rows(output):
        output = np.asarray(output)
        if output.ndim != 3 or output.shape[0] != 1:
            raise RuntimeError(
                f'Unexpected YOLO output shape: {tuple(output.shape)}'
            )

        supported_channels = {
            4 + len(COCO_CLASS_NAMES),
            5 + len(COCO_CLASS_NAMES),
        }
        if output.shape[1] in supported_channels:
            return output[0].T
        if output.shape[2] in supported_channels:
            return output[0]
        raise RuntimeError(
            'Expected a raw COCO YOLO tensor with 84 or 85 channels, got '
            f'{tuple(output.shape)}.'
        )

    def _postprocess(
        self,
        predictions,
        image_width,
        image_height,
        scale,
        left,
        top,
    ):
        if predictions.shape[1] == 5 + len(COCO_CLASS_NAMES):
            class_scores = predictions[:, 5:] * predictions[:, 4:5]
        else:
            class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[
            np.arange(len(class_scores)),
            class_ids,
        ]

        candidate_boxes = []
        candidate_confidences = []
        candidate_classes = []
        for prediction, confidence, class_id in zip(
            predictions,
            confidences,
            class_ids,
        ):
            confidence = float(confidence)
            class_id = int(class_id)
            label = COCO_CLASS_NAMES[class_id]
            if confidence < self.confidence_threshold:
                continue
            if self.allowed_labels is not None:
                if label not in self.allowed_labels:
                    continue

            center_x, center_y, width, height = prediction[:4]
            candidate_boxes.append([
                float(center_x - width / 2.0),
                float(center_y - height / 2.0),
                float(width),
                float(height),
            ])
            candidate_confidences.append(confidence)
            candidate_classes.append(class_id)

        if not candidate_boxes:
            return []

        indices = self._class_aware_nms_indices(
            candidate_boxes,
            candidate_confidences,
            candidate_classes,
        )
        if not indices:
            return []

        detections = []
        for index in indices:
            x, y, width, height = candidate_boxes[index]
            x1 = round((x - left) / scale)
            y1 = round((y - top) / scale)
            x2 = round((x + width - left) / scale)
            y2 = round((y + height - top) / scale)
            x1 = min(max(x1, 0), image_width - 1)
            y1 = min(max(y1, 0), image_height - 1)
            x2 = min(max(x2, x1 + 1), image_width)
            y2 = min(max(y2, y1 + 1), image_height)
            class_id = candidate_classes[index]
            detections.append(Detection(
                class_id=class_id,
                label=COCO_CLASS_NAMES[class_id],
                confidence=candidate_confidences[index],
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
            ))
            if len(detections) >= self.max_detections:
                break
        return detections

    def _class_aware_nms_indices(
        self,
        boxes,
        confidences,
        class_ids,
    ):
        kept = []
        for class_id in sorted(set(class_ids)):
            class_indices = [
                index
                for index, candidate_class in enumerate(class_ids)
                if candidate_class == class_id
            ]
            class_boxes = [boxes[index] for index in class_indices]
            class_confidences = [
                confidences[index] for index in class_indices
            ]
            local_indices = cv2.dnn.NMSBoxes(
                class_boxes,
                class_confidences,
                self.confidence_threshold,
                self.iou_threshold,
            )
            kept.extend(
                class_indices[int(local_index)]
                for local_index in np.asarray(local_indices).reshape(-1)
            )
        kept.sort(key=lambda index: confidences[index], reverse=True)
        return kept


def annotate_detections(image, detections: Iterable[Detection], latency_ms):
    """Draw non-destructive boxes and a compact inference status header."""
    annotated = image.copy()
    detections = list(detections)
    for detection in detections:
        color = _class_color(detection.class_id)
        cv2.rectangle(
            annotated,
            (detection.x1, detection.y1),
            (detection.x2, detection.y2),
            color,
            2,
        )
        label = f'{detection.label} {detection.confidence:.2f}'
        if detection.distance_m is not None:
            label += f' {detection.distance_m:.2f}m'
        text_size, baseline = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            1,
        )
        text_y = max(detection.y1, text_size[1] + baseline + 4)
        cv2.rectangle(
            annotated,
            (detection.x1, text_y - text_size[1] - baseline - 4),
            (detection.x1 + text_size[0] + 6, text_y),
            color,
            -1,
        )
        cv2.putText(
            annotated,
            label,
            (detection.x1 + 3, text_y - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    status = f'YOLO detections: {len(detections)} | {latency_ms:.1f} ms'
    cv2.rectangle(annotated, (0, 0), (360, 28), (28, 28, 28), -1)
    cv2.putText(
        annotated,
        status,
        (8, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (245, 245, 245),
        1,
        cv2.LINE_AA,
    )
    return annotated


def _class_color(class_id):
    palette = (
        (45, 210, 80),
        (40, 160, 240),
        (230, 110, 60),
        (190, 70, 210),
        (70, 200, 210),
        (235, 80, 120),
    )
    return palette[class_id % len(palette)]
