#!/usr/bin/env python3
"""FPMS YOLO - TREE + OBSTACLE. No flicker. Edge-based obstacle fallback."""
import cv2, numpy as np, time, threading
from flask import Flask, Response, jsonify
from rknnlite.api import RKNNLite

MODEL = "/home/ubuntu/yolo/yolov8n.rknn"
CONF = 0.22
NMS = 0.50
FPS_CAP = 8
MAX_DETS = 5

TREE_IDS = {58, 50, 75}
TREE_COLOR = (0, 220, 0)
OBS_COLOR = (0, 100, 255)
UNK_COLOR = (0, 180, 255)  # Unknown obstacle from edge detection

def softmax(x, axis=-1):
    e = np.exp(x - np.max(x, axis=axis, keepdims=True))
    return e / np.sum(e, axis=axis, keepdims=True)

def dfl(x):
    n, c, h, w = x.shape
    x = x.reshape(n, 4, c//4, h, w)
    x = softmax(x, axis=2)
    a = np.arange(c//4).reshape(1,1,-1,1,1).astype(np.float32)
    return np.sum(x * a, axis=2)

def decode(outputs, ih, iw):
    all_b, all_s, all_c = [], [], []
    for i, stride in enumerate([8, 16, 32]):
        bf = outputs[i*3]
        cf = outputs[i*3+1]
        of = outputs[i*3+2]
        _, _, h, w = cf.shape
        bd = dfl(bf)
        yv, xv = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        grid = np.stack([xv, yv], 0).reshape(1,2,h,w).astype(np.float32)
        x1y1 = (grid - bd[:,:2]) * stride
        x2y2 = (grid + bd[:,2:]) * stride
        sc = cf * of
        x1 = x1y1[:,0].reshape(-1)
        y1 = x1y1[:,1].reshape(-1)
        x2 = x2y2[:,0].reshape(-1)
        y2 = x2y2[:,1].reshape(-1)
        boxes = np.stack([x1,y1,x2,y2], 1)
        sc2 = sc[0].reshape(80,-1).T
        cid = sc2.argmax(1)
        conf = sc2.max(1)
        keep = conf > CONF
        if keep.sum():
            all_b.append(boxes[keep])
            all_s.append(conf[keep])
            all_c.append(cid[keep])
    if not all_b:
        return [],[],[]
    boxes = np.concatenate(all_b)
    scores = np.concatenate(all_s)
    classes = np.concatenate(all_c)
    boxes /= (640 / max(ih, iw))
    ki = []
    for c in np.unique(classes):
        m = classes == c
        cb, cs = boxes[m], scores[m]
        wh = cb[:,2:] - cb[:,:2]
        xywh = np.column_stack([cb[:,0], cb[:,1], wh[:,0], wh[:,1]])
        idx = cv2.dnn.NMSBoxes(xywh.tolist(), cs.tolist(), CONF, NMS)
        if len(idx):
            ki.extend(np.where(m)[0][idx.flatten()].tolist())
    if not ki:
        return [],[],[]
    ki = sorted(ki, key=lambda i: -scores[i])[:MAX_DETS]
    return boxes[ki], scores[ki], classes[ki]

def find_unknown_obstacles(frame, yolo_boxes):
    """Find large objects via edge detection that YOLO missed."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7, 7), 0)
    edges = cv2.Canny(blur, 30, 100)
    # Dilate to connect edges
    kernel = np.ones((9, 9), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=2)
    contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = frame.shape[:2]
    min_area = (w * h) * 0.02  # At least 2% of frame
    max_area = (w * h) * 0.6   # No more than 60%
    unknown = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area or area > max_area:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        # Skip if too flat (probably the table/floor edge)
        if bw > bh * 4 or bh > bw * 6:
            continue
        # Skip if it overlaps with a YOLO detection
        overlaps = False
        for yb in yolo_boxes:
            ox1 = max(x, yb[0]); oy1 = max(y, yb[1])
            ox2 = min(x+bw, yb[2]); oy2 = min(y+bh, yb[3])
            if ox2 > ox1 and oy2 > oy1:
                inter = (ox2-ox1) * (oy2-oy1)
                if inter > area * 0.3:
                    overlaps = True; break
        if not overlaps:
            unknown.append([x, y, x+bw, y+bh])
    # Return top 2 by area
    unknown.sort(key=lambda b: -(b[2]-b[0])*(b[3]-b[1]))
    return unknown[:2]

# Double buffer: always have a frame ready
jpg_a = None
jpg_b = None
buf_lock = threading.Lock()
dets_out = []
fps_out = 0.0

def cam_open():
    c = cv2.VideoCapture(0, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return c

def run():
    global jpg_a, jpg_b, dets_out, fps_out
    rknn = RKNNLite()
    rknn.load_rknn(MODEL)
    rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
    print("[YOLO] NPU ready")
    cap = cam_open()
    print("[YOLO] Camera open")
    fc, ft, dt = 0, time.time(), 1.0/FPS_CAP

    while True:
        t0 = time.time()
        try:
            try:
                ok, frame = cap.read()
            except:
                ok = False
            if not ok:
                try: cap.release()
                except: pass
                time.sleep(1.5)
                try: cap = cam_open(); print("[YOLO] Reconnected")
                except: pass
                continue

            h, w = frame.shape[:2]

            # YOLO inference
            s = 640/max(h,w)
            nh, nw = int(h*s), int(w*s)
            r = cv2.resize(frame, (nw, nh))
            p = np.full((640,640,3), 114, dtype=np.uint8)
            p[:nh,:nw] = r
            inp = np.expand_dims(cv2.cvtColor(p, cv2.COLOR_BGR2RGB), 0)
            out = rknn.inference(inputs=[inp])
            boxes, scores, classes = decode(out, h, w)

            # Process YOLO detections
            draw_dets = []
            yolo_boxes = []
            if len(boxes):
                for b, sc, ci in zip(boxes, scores, classes):
                    cid = int(ci)
                    x1,y1,x2,y2 = [max(0,int(v)) for v in b]
                    x2, y2 = min(w, x2), min(h, y2)
                    if (x2-x1) < 10 or (y2-y1) < 10:
                        continue
                    is_tree = cid in TREE_IDS
                    name = "TREE" if is_tree else "OBSTACLE"
                    col = TREE_COLOR if is_tree else OBS_COLOR
                    cv2.rectangle(frame, (x1,y1), (x2,y2), col, 4)
                    label = f"{name} {float(sc):.0%}"
                    (tw, th_), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
                    cv2.rectangle(frame, (x1, y1-th_-14), (x1+tw+12, y1), col, -1)
                    cv2.putText(frame, label, (x1+6, y1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,0), 2)
                    draw_dets.append({"name": name, "conf": round(float(sc),2)})
                    yolo_boxes.append([x1,y1,x2,y2])

            # Edge-based fallback: find large objects YOLO missed
            unknowns = find_unknown_obstacles(frame, yolo_boxes)
            for ub in unknowns:
                x1,y1,x2,y2 = ub
                cv2.rectangle(frame, (x1,y1), (x2,y2), UNK_COLOR, 3)
                cv2.rectangle(frame, (x1, y1-30), (x1+200, y1), UNK_COLOR, -1)
                cv2.putText(frame, "OBSTACLE", (x1+6, y1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,0), 2)
                draw_dets.append({"name": "OBSTACLE", "conf": 0.5})

            # FPS
            fc += 1
            if fc >= 8:
                fps_out = fc / (time.time() - ft)
                fc = 0; ft = time.time()

            trees = sum(1 for d in draw_dets if d["name"] == "TREE")
            obs = sum(1 for d in draw_dets if d["name"] == "OBSTACLE")
            bar = f"NPU {fps_out:.0f}FPS | {trees} TREE | {obs} OBSTACLE"
            cv2.rectangle(frame, (0,0), (520, 42), (0,0,0), -1)
            cv2.putText(frame, bar, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

            ok2, jpg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok2:
                with buf_lock:
                    jpg_a = jpg.tobytes()
                    dets_out = draw_dets

        except Exception as e:
            print(f"[YOLO] {e}")
            try: cap.release()
            except: pass
            time.sleep(3)
            try: cap = cam_open()
            except: pass

        el = time.time() - t0
        if el < dt:
            time.sleep(dt - el)

app = Flask(__name__)

HTML = '''<!doctype html><html><head><title>FPMS AI VISION</title>
<style>
*{margin:0;box-sizing:border-box}
body{background:#000;color:#fff;font-family:Consolas,monospace}
.bar{background:#080810;padding:12px 24px;border-bottom:3px solid #ff4b70;display:flex;justify-content:space-between;align-items:center}
.bar h1{color:#ff4b70;font-size:30px;letter-spacing:4px}
.bar .st{font-size:22px}
.wrap{display:flex;height:calc(100vh - 60px)}
.cam{flex:1;background:#000;display:flex;align-items:center;justify-content:center}
.cam img{width:100%;height:100%;object-fit:contain}
.side{width:360px;background:#060610;border-left:2px solid #1d4163;padding:16px;overflow-y:auto}
.h{color:#0af;font-size:14px;letter-spacing:3px;font-weight:900;border-bottom:2px solid #28506f;padding-bottom:8px;margin-bottom:12px}
.det{padding:12px;margin-bottom:10px;border-radius:10px;border:3px solid}
.det.tree{border-color:#0c0;background:#001a00}
.det.obs{border-color:#05f;background:#000a1a}
.det .n{font-size:26px;font-weight:bold}
.det.tree .n{color:#0c0}
.det.obs .n{color:#05f}
.det .c{font-size:16px;color:#aaa;margin-top:4px}
.empty{color:#444;text-align:center;padding:30px;font-size:18px}
</style></head><body>
<div class="bar">
<h1>FPMS AI VISION</h1>
<div class="st" id="st">--</div>
</div>
<div class="wrap">
<div class="cam"><img src="/stream"></div>
<div class="side">
<div class="h">DETECTIONS</div>
<div id="d"><div class="empty">Scanning...</div></div>
</div>
</div>
<script>
async function p(){try{
var r=await(await fetch('/d')).json();
var t=r.d.filter(function(x){return x.name==='TREE'}).length;
var o=r.d.filter(function(x){return x.name==='OBSTACLE'}).length;
document.getElementById('st').innerHTML=
'<span style="color:#0c0">'+t+' TREE</span> &bull; <span style="color:#05f">'+o+' OBSTACLE</span> &bull; '+r.f.toFixed(0)+' FPS';
var el=document.getElementById('d');
if(!r.d.length){el.innerHTML='<div class="empty">No objects in view</div>'}
else{el.innerHTML=r.d.map(function(x){
var cls=x.name==='TREE'?'tree':'obs';
return '<div class="det '+cls+'"><div class="n">'+x.name+'</div><div class="c">'+Math.round(x.conf*100)+'%</div></div>'
}).join('')}
}catch(e){}setTimeout(p,500)}p();
</script></body></html>'''

@app.route('/')
def index():
    return HTML

@app.route('/d')
def api_d():
    with buf_lock:
        return jsonify(d=dets_out, f=fps_out)

@app.route('/stream')
def stream():
    def gen():
        last = None
        while True:
            with buf_lock:
                j = jpg_a
            # Only send if we have a NEW frame (prevents flicker)
            if j and j is not last:
                yield b'--f\r\nContent-Type: image/jpeg\r\n\r\n' + j + b'\r\n'
                last = j
            time.sleep(0.1)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=f')

if __name__ == '__main__':
    threading.Thread(target=run, daemon=True).start()
    print("[YOLO] http://0.0.0.0:8086")
    app.run(host='0.0.0.0', port=8086, threaded=True, debug=False)
