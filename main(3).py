from maix import camera, display, image, nn, app, uart, pinmap
import time, struct

# ---------- 状态定义 ----------
STATE_SCAN  = 0   # 未找到A4，云台左右扫描
STATE_TRACK = 1   # 已找到A4，云台锁定追踪

# ---------- 卡尔曼滤波器 ----------
class KalmanFilter1D:
    """一维卡尔曼滤波器，用于平滑目标坐标"""
    def __init__(self, process_noise=0.01, measurement_noise=2.0):
        self.x = 0.0      # 状态估计值
        self.P = 1.0       # 估计误差协方差
        self.Q = process_noise       # 过程噪声
        self.R = measurement_noise   # 测量噪声
        self.initialized = False

    def update(self, measurement):
        if not self.initialized:
            self.x = measurement
            self.initialized = True
            return self.x

        # 预测
        P_pred = self.P + self.Q

        # 更新
        K = P_pred / (P_pred + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * P_pred

        return self.x

    def reset(self):
        self.initialized = False

# 创建dx/dy的卡尔曼滤波器
# process_noise: 越小越平滑但响应慢
# measurement_noise: 越大越信任预测值
kf_dx = KalmanFilter1D(process_noise=0.05, measurement_noise=3.0)
kf_dy = KalmanFilter1D(process_noise=0.05, measurement_noise=3.0)

# ---------- 串口配置 ----------
pinmap.set_pin_function("A21", "UART4_TX")
pinmap.set_pin_function("A22", "UART4_RX")
serial = uart.UART("/dev/ttyS4", 115200)

# ---------- 模型加载 ----------
model_path = "/mycode/25E/best.mud"
detector = nn.YOLO11(model=model_path)

dis = display.Display()

# ---------- 自适应分辨率 ----------
# 检测使用模型原始分辨率，采集和显示使用更高分辨率
det_w = detector.input_width()
det_h = detector.input_height()

# 尝试使用更高采集分辨率，回退到模型分辨率
HI_RES_W = 640
HI_RES_H = 480
try:
    cam = camera.Camera(HI_RES_W, HI_RES_H, detector.input_format())
    cam_w, cam_h = HI_RES_W, HI_RES_H
except Exception:
    cam = camera.Camera(det_w, det_h, detector.input_format())
    cam_w, cam_h = det_w, det_h

scale_x = cam_w / det_w
scale_y = cam_h / det_h

img_w = cam_w
img_h = cam_h
half_w = img_w // 2
half_h = img_h // 2

# ---------- 追踪参数 ----------
LOCK_FRAMES = 10
LOCK_THRESH = 15
lock_count = 0
locked = False

# ---------- FPS ----------
fps_t0 = time.time()
fps_count = 0
fps_val = 0

def send_cmd(cmd, dx, dy):
    """
    协议: 帧头 0xAA 0xBB + cmd(1B) + dx(2B signed LE) + dy(2B signed LE) + checksum(1B)
    cmd: 0=扫描  1=追踪
    dx/dy: A4中心偏离图像中心的偏移，正值=目标在画面右/下方
    """
    data = struct.pack("<Bhh", cmd, dx, dy)
    checksum = sum(data) & 0xFF
    serial.write(b'\xAA\xBB' + data + bytes([checksum]))

while not app.need_exit():
    img = cam.read()

    # 缩放到模型输入分辨率进行检测
    if cam_w != det_w or cam_h != det_h:
        det_img = img.resize(det_w, det_h)
    else:
        det_img = img
    objs = detector.detect(det_img, conf_th=0.1, iou_th=0.45)

    a4_list = [o for o in objs if o.class_id == 0]
    a4 = max(a4_list, key=lambda o: o.score) if a4_list else None

    if a4:
        # 将检测坐标映射回高分辨率图像
        rx = int(a4.x * scale_x)
        ry = int(a4.y * scale_y)
        rw = int(a4.w * scale_x)
        rh = int(a4.h * scale_y)
        target_cx = rx + rw // 2
        target_cy = ry + rh // 2

        # 卡尔曼滤波平滑坐标
        dx_raw = target_cx - half_w
        dy_raw = target_cy - half_h
        dx = int(kf_dx.update(dx_raw))
        dy = int(kf_dy.update(dy_raw))

        img.draw_rect(rx, ry, rw, rh, color=image.COLOR_GREEN)
        img.draw_cross(target_cx, target_cy, color=image.COLOR_GREEN, size=10)
        img.draw_circle(target_cx, target_cy, 3, color=image.COLOR_GREEN)

        lock_count += 1
        if lock_count >= LOCK_FRAMES and abs(dx) < LOCK_THRESH and abs(dy) < LOCK_THRESH:
            locked = True

        send_cmd(STATE_TRACK, dx, dy)
        msg = f'TRACK dx:{dx} dy:{dy}'
        if locked:
            msg += ' LOCKED'
        img.draw_string(4, 4, msg, color=image.COLOR_YELLOW)

    else:
        lock_count = 0
        locked = False
        kf_dx.reset()  # 丢失目标时重置滤波器
        kf_dy.reset()
        send_cmd(STATE_SCAN, 0, 0)
        img.draw_string(4, 4, 'SCAN', color=image.COLOR_YELLOW)

    img.draw_cross(half_w, half_h, color=image.COLOR_WHITE, size=8)

    # FPS计算
    fps_count += 1
    if fps_count >= 10:
        t1 = time.time()
        fps_val = fps_count / (t1 - fps_t0)
        fps_t0 = t1
        fps_count = 0
    img.draw_string(half_w - 30, img_h - 20, f'FPS:{fps_val:.1f}', color=image.COLOR_WHITE)

    dis.show(img)
