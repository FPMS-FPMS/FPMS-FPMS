"""Measure where the camera pipeline's time actually goes on the Pi."""
import os

import paramiko

BENCH = r'''
import time, sys, json
sys.path.insert(0, "/home/ubuntu/yolo")
import cv2, numpy as np
import fpms_yolo_npu as y
from rknnlite.api import RKNNLite

r = RKNNLite()
r.load_rknn(y.MODEL); r.init_runtime()

cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
for _ in range(3): cap.read()          # warm up

N = 30
t_cap = t_pre = t_inf = t_dec = t_jpg = 0.0
for _ in range(N):
    a = time.perf_counter(); ok, frame = cap.read(); b = time.perf_counter()
    if not ok: continue
    img = cv2.resize(frame, (640, 640)); img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    c = time.perf_counter()
    out = r.inference(inputs=[np.expand_dims(img, 0)])
    d = time.perf_counter()
    boxes, scores, classes = y.decode(out, frame.shape[0], frame.shape[1])
    e = time.perf_counter()
    cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 55])
    f = time.perf_counter()
    t_cap += b-a; t_pre += c-b; t_inf += d-c; t_dec += e-d; t_jpg += f-e

tot = t_cap+t_pre+t_inf+t_dec+t_jpg
print("  per frame over %d frames (ms):" % N)
print("    capture   %6.1f" % (t_cap/N*1000))
print("    preprocess%6.1f" % (t_pre/N*1000))
print("    NPU infer %6.1f" % (t_inf/N*1000))
print("    decode+NMS%6.1f" % (t_dec/N*1000))
print("    jpeg enc  %6.1f" % (t_jpg/N*1000))
print("    TOTAL     %6.1f  -> theoretical %.1f fps" % (tot/N*1000, N/tot))
cap.release(); r.release()
'''

# Credentials come from the environment — never hardcode. This file is committed.
#   export FPMS_PI_PASS=...   (optionally FPMS_PI_HOST / FPMS_PI_USER)
_HOST = os.environ.get("FPMS_PI_HOST", "192.168.137.73")
_USER = os.environ.get("FPMS_PI_USER", "ubuntu")
_PASS = os.environ.get("FPMS_PI_PASS") or ""
if not _PASS:
    raise SystemExit(
        "FPMS_PI_PASS is not set. Export it before running this benchmark; "
        "the password is deliberately not stored in the repo."
    )

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(_HOST, username=_USER, password=_PASS, timeout=20)

# Pause the agent so it isn't competing for the camera and NPU.
c.exec_command("sudo systemctl stop fpms-rover-agent")[1].channel.recv_exit_status()
sftp = c.open_sftp()
with sftp.file("/home/ubuntu/bench_npu.py", "w") as f:
    f.write(BENCH)
sftp.close()

_i, o, e = c.exec_command("cd /home/ubuntu && python3 bench_npu.py 2>&1 | grep -vE '^[IWE] RKNN|^W rknn|VIDIOC|obsensor|video4linux'", timeout=300)
print(o.read().decode("utf-8", "replace"))
print(e.read().decode("utf-8", "replace")[:500])

print("=== model + tooling on the Pi ===")
for cmd in ["ls -la /home/ubuntu/yolo/*.rknn /home/ubuntu/yolo/*.pt 2>/dev/null",
            "pip3 list 2>/dev/null | grep -iE 'ultralytics|rknn|onnx'",
            "free -m | head -2",
            "cat /sys/kernel/debug/rknpu/version 2>/dev/null || echo 'npu version n/a'"]:
    _i, o, e = c.exec_command(cmd, timeout=60)
    print(o.read().decode("utf-8", "replace").strip() or "(none)")
    print()

c.exec_command("sudo systemctl start fpms-rover-agent")[1].channel.recv_exit_status()
print("agent restarted")
c.close()
