"""YOLO26 decode path for the RK3588 NPU.

Drop-in replacement for `fpms_yolo_npu.decode` when the loaded model is
`yolo26n-rk3588.rknn`. Deploy alongside it in /home/ubuntu/yolo/.

Contract matches the v8 module exactly so the agent needs a one-line swap:

    boxes, scores, classes = decode(outputs, ih, iw)

    boxes   -> (N, 4) float32, [x1, y1, x2, y2] in ORIGINAL frame pixels
    scores  -> (N,)   float32
    classes -> (N,)   int32, COCO ids

WHAT THE EXPORTED GRAPH ACTUALLY EMITS
--------------------------------------
Read off models/yolo26n.onnx (the source the .rknn was converted from):

    input   images  : [1, 3, 640, 640]
    output  output0 : [1, 84, 8400]

8400 = 80*80 + 40*40 + 20*20 anchors. 84 = 4 box + 80 class.

The DFL reconstruction is *already folded into the graph* (the two Softmax
ops in model.23), so channels 0..3 arrive as xywh CENTRE form already
multiplied by the per-anchor stride constant [8]*6400 + [16]*1600 + [32]*400
-- i.e. in 640x640 input-pixel units. Channels 4..83 have already been
through Sigmoid, so they are probabilities; do not sigmoid them again.

The graph contains NO NonMaxSuppression and NO TopK (verified by op
histogram). It was exported with end2end=false because Ultralytics warns
end2end=true segfaults on RK3588.

So "NMS-free" here describes how YOLO26 was TRAINED, not the tensor you get
back. With end2end=false the head still emits all 8400 anchors and you must
threshold them yourself. A light NMS is retained below: duplicate suppression
is much weaker than v8's but is not identically zero, and the cost after
confidence thresholding is a handful of boxes.

Because the agent preprocesses with a plain cv2.resize(frame, (640, 640))
stretch -- NOT a letterbox -- rescaling to the source frame is just an
independent x and y ratio. Do not add padding compensation here unless the
agent's preprocess changes too.
"""

import numpy as np

MODEL = "/home/ubuntu/yolo/yolo26n-rk3588.rknn"

INPUT_SIZE = 640
NUM_CLASSES = 80
NUM_ANCHORS = 8400
NUM_CH = 4 + NUM_CLASSES          # 84

CONF_THRES = 0.35
IOU_THRES = 0.55
MAX_DETECTIONS = 50


def _as_anchor_major(raw):
    """Return the model output as a contiguous (8400, 84) float32 array.

    RKNN is not consistent about whether it hands back the ONNX layout
    (1, 84, 8400) or a channel-last (1, 8400, 84), and it may or may not keep
    the batch axis. Rather than trust one shape, locate the 84 axis.
    """
    a = np.asarray(raw)
    a = np.squeeze(a)             # drop batch / any stray unit axes

    if a.ndim != 2:
        raise ValueError(
            f"expected a 2-D output after squeeze, got shape {np.asarray(raw).shape}. "
            "If the RKNN export split the head into separate branch tensors this "
            "decoder does not apply -- re-export as a single output0."
        )

    if a.shape[1] == NUM_CH:          # (8400, 84)
        out = a
    elif a.shape[0] == NUM_CH:        # (84, 8400) -- the ONNX layout
        out = a.T
    else:
        raise ValueError(
            f"neither axis of {a.shape} is {NUM_CH} (4 box + {NUM_CLASSES} class)"
        )

    # .T yields a view with awkward strides; the max/argmax below are far
    # cheaper on a contiguous buffer.
    return np.ascontiguousarray(out, dtype=np.float32)


def _nms(boxes, scores, iou_thres):
    """Plain greedy NMS. Returns indices to keep, best score first."""
    if boxes.shape[0] == 0:
        return np.empty((0,), dtype=np.int32)

    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]

        xx1 = np.maximum(x1[i], x1[rest])
        yy1 = np.maximum(y1[i], y1[rest])
        xx2 = np.minimum(x2[i], x2[rest])
        yy2 = np.minimum(y2[i], y2[rest])

        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        union = areas[i] + areas[rest] - inter
        iou = np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)

        order = rest[iou <= iou_thres]

    return np.asarray(keep, dtype=np.int32)


def decode(outputs, ih, iw, conf_thres=CONF_THRES, iou_thres=IOU_THRES):
    """Decode one YOLO26 inference into boxes/scores/classes.

    outputs -- whatever rknn.inference() returned (a list of arrays)
    ih, iw  -- height/width of the ORIGINAL frame, for rescaling
    """
    raw = outputs[0] if isinstance(outputs, (list, tuple)) else outputs
    pred = _as_anchor_major(raw)              # (8400, 84)

    cls = pred[:, 4:]                          # already sigmoid'd

    # One pass for the winning class, a gather for its score. Doing argmax then
    # take_along_axis beats max()+argmax() -- this runs on every inferred frame
    # over ~672k floats, so the second full sweep is worth avoiding.
    class_ids = cls.argmax(axis=1)
    confs = np.take_along_axis(cls, class_ids[:, None], axis=1).reshape(-1)

    mask = confs >= conf_thres
    if not mask.any():
        return (np.empty((0, 4), np.float32),
                np.empty((0,), np.float32),
                np.empty((0,), np.int32))

    xywh = pred[mask, :4]
    confs = confs[mask]
    class_ids = class_ids[mask].astype(np.int32)

    # Centre form -> corners, still in 640x640 input space.
    cx, cy, w, h = xywh[:, 0], xywh[:, 1], xywh[:, 2], xywh[:, 3]
    half_w, half_h = w * 0.5, h * 0.5
    boxes = np.stack([cx - half_w, cy - half_h, cx + half_w, cy + half_h], axis=1)

    # Plain stretch resize in the agent -> independent per-axis ratio.
    boxes[:, 0::2] *= (iw / float(INPUT_SIZE))
    boxes[:, 1::2] *= (ih / float(INPUT_SIZE))

    np.clip(boxes[:, 0::2], 0, iw - 1, out=boxes[:, 0::2])
    np.clip(boxes[:, 1::2], 0, ih - 1, out=boxes[:, 1::2])

    # NMS per class, so an overlapping person and truck both survive. The
    # offset trick keeps it to a single NMS call.
    if boxes.shape[0] > 1:
        spread = max(ih, iw) + 1.0
        offset = class_ids.astype(np.float32) * spread
        keep = _nms(boxes + offset[:, None], confs, iou_thres)
    else:
        keep = np.zeros((1,), dtype=np.int32)

    keep = keep[:MAX_DETECTIONS]
    return boxes[keep], confs[keep], class_ids[keep]
