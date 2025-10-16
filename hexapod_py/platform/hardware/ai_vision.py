"""
AI Vision Processing Module

This module contains classes and functions for performing AI-based vision tasks,
such as object detection, on image frames.
"""

import cv2
import numpy as np
import os

# --- Hailo AI specific imports ---
# These libraries are part of the Hailo AI Software Suite.
# You must install the suite from the Hailo Developer Zone for your specific OS and hardware.
# See: https://hailo.ai/developer-zone/
try:
    from hailo_platform import (HEF, VDevice, HailoStreamInterface, InferVStreams, ConfigureParams)
    HAILO_AVAILABLE = True
except ImportError:
    HAILO_AVAILABLE = False

# --- Configuration ---
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- IMPORTANT ---
# You must download a compiled model (.hef file) and its corresponding labels file.
# This example uses yolov5m, which can be downloaded from the Hailo Model Zoo.
# Place the .hef and .txt files in this directory.
_MODEL_PATH = os.path.join(_CURRENT_DIR, "yolov5m_wo_spp_60p.hef")
_LABEL_PATH = os.path.join(_CURRENT_DIR, "coco_80_labels.txt")

# --- Post-processing Configuration for YOLOv5 ---
_CONFIDENCE_THRESHOLD = 0.4
_IOU_THRESHOLD = 0.45
_YOLO_ANCHORS = np.array([
    [[10, 13], [16, 30], [33, 23]],
    [[30, 61], [62, 45], [59, 119]],
    [[116, 90], [156, 198], [373, 326]]
], dtype=np.float32)


def _sigmoid(x):
    return 1. / (1. + np.exp(-x))

def _filter_and_process_detections(outputs, original_dims, input_dims, anchors):
    """Decodes YOLOv5 raw output tensors into bounding boxes."""
    boxes, scores, class_ids = [], [], []
    original_h, original_w = original_dims
    input_h, input_w = input_dims

    for i, (out, anchor) in enumerate(zip(outputs, anchors)):
        grid_h, grid_w, num_anchors, _ = out.shape
        
        # Reshape and decode
        out = out.reshape(grid_h, grid_w, num_anchors, -1)
        xy, wh, conf, cls_prob = np.split(out, [2, 4, 5], axis=-1)

        xy = (_sigmoid(xy) * 2 - 0.5 + np.mgrid[0:grid_w, 0:grid_h].transpose(1, 0, 2)[..., ::-1]) * (input_w / grid_w)
        wh = (np.power(_sigmoid(wh) * 2, 2) * anchor)
        
        conf = _sigmoid(conf)
        cls_prob = _sigmoid(cls_prob)
        
        # Filter by confidence
        mask = (conf > _CONFIDENCE_THRESHOLD).squeeze(-1)
        if not np.any(mask):
            continue

        xy, wh, conf, cls_prob = xy[mask], wh[mask], conf[mask], cls_prob[mask]

        # Convert to (x1, y1, x2, y2) format
        x1y1 = xy - wh / 2
        x2y2 = xy + wh / 2
        
        # Scale to original image dimensions
        scale_w, scale_h = original_w / input_w, original_h / input_h
        x1y1 *= [scale_w, scale_h]
        x2y2 *= [scale_w, scale_h]

        # Clip boxes to image boundaries
        x1y1 = np.maximum(x1y1, 0)
        x2y2 = np.minimum(x2y2, [original_w, original_h])

        final_boxes = np.concatenate([x1y1, x2y2], axis=-1)
        final_scores = (conf * cls_prob.max(axis=-1, keepdims=True)).flatten()
        final_class_ids = cls_prob.argmax(axis=-1)

        boxes.append(final_boxes)
        scores.append(final_scores)
        class_ids.append(final_class_ids)

    if not boxes:
        return [], [], []

    # Apply Non-Maximum Suppression (NMS)
    all_boxes = np.concatenate(boxes, axis=0)
    all_scores = np.concatenate(scores, axis=0)
    all_class_ids = np.concatenate(class_ids, axis=0)
    
    indices = cv2.dnn.NMSBoxes(all_boxes.tolist(), all_scores.tolist(), _CONFIDENCE_THRESHOLD, _IOU_THRESHOLD)
    
    if len(indices) == 0:
        return [], [], []
        
    return all_boxes[indices], all_scores[indices], all_class_ids[indices]

class ObjectDetector:
    """
    An object detector that uses a Hailo-8 AI accelerator to perform inference.
    """
    def __init__(self):
        self.vdevice = None
        self.network_group = None
        self.infer_model = None
        self.labels = None
        self.input_vstream_infos = None
        self.output_vstream_infos = None

        if not HAILO_AVAILABLE:
            print("!!! WARNING: Hailo AI SDK not found. AI Vision will not work.")
            return

        if not os.path.exists(_MODEL_PATH) or not os.path.exists(_LABEL_PATH):
            print("!!! ERROR: Hailo .hef model or label file not found.")
            print(f"!!! Searched for model: {_MODEL_PATH}")
            print(f"!!! Searched for labels: {_LABEL_PATH}")
            print("!!! Please download them from the Hailo Model Zoo and place them in the 'platform/hardware/' directory.")
            return

        try:
            print("Initializing Hailo-8 Object Detector...")
            self.labels = {i: line.strip() for i, line in enumerate(open(_LABEL_PATH))}
            
            self.vdevice = VDevice()
            hef = HEF(_MODEL_PATH)

            configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
            self.network_group = self.vdevice.configure(hef, configure_params)[0]
            self.infer_model = self.network_group.create_infer_model()

            self.input_vstream_infos = self.infer_model.get_input_vstream_infos()
            self.output_vstream_infos = self.infer_model.get_output_vstream_infos()

            print(f" - Model loaded: yolov5m")
            print(f" - Expected input shape: {self.input_vstream_infos[0].shape}")
            print(" - Hailo detector initialized successfully.")
        except Exception as e:
            print(f"!!! ERROR: Failed to initialize Hailo-8 device: {e}")
            self.vdevice = None

    def detect(self, frame: np.ndarray) -> np.ndarray:
        """
        Processes a frame to detect objects and draws visualizations.
        :param frame: The input image frame as a NumPy array (BGR format).
        :return: The frame with detection visualizations drawn on it (BGR format).
        """
        if not self.vdevice or not self.infer_model:
            return frame

        input_shape = self.input_vstream_infos[0].shape
        input_h, input_w = input_shape[1], input_shape[2]
        
        # 1. Pre-process frame
        frame_resized = cv2.resize(frame, (input_w, input_h), interpolation=cv2.INTER_AREA)
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

        # 2. Run inference
        with InferVStreams(self.network_group) as infer_pipeline:
            input_data = {self.input_vstream_infos[0].name: np.expand_dims(frame_rgb, axis=0)}
            with self.infer_model.main_context() as ctx:
                raw_outputs = infer_pipeline.infer(input_data)

        # 3. Post-process results
        # Sort outputs by shape to match anchor order
        sorted_outputs = sorted([raw_outputs[name] for name in raw_outputs], key=lambda x: x.shape[1], reverse=True)
        
        boxes, scores, class_ids = _filter_and_process_detections(
            sorted_outputs, frame.shape[:2], (input_h, input_w), _YOLO_ANCHORS
        )

        # 4. Draw detections on the original frame
        processed_frame = frame.copy()
        for box, score, class_id in zip(boxes, scores, class_ids):
            x1, y1, x2, y2 = map(int, box)
            label_text = f'{self.labels.get(class_id, "Unknown")} ({score:.2f})'
            cv2.rectangle(processed_frame, (x1, y1), (x2, y2), (36, 255, 12), 2)
            cv2.putText(processed_frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (36, 255, 12), 2)
        
        return processed_frame