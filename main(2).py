from maix import camera, display, image, nn, app, uart, pinmap
import struct

# ---------- 状态定义 ----------
STATE_SCAN  = 0   # 未找到A4，云台左右扫描
STATE_TRACK = 1   # 已找到A4，云台锁定追踪

# ---------- 串口配置 ----------
pinmap.set_pin_function("A21", "UART4_TX")
pinmap.set_pin_function("A22", "UART4_RX")
serial = uart.UART("/dev/ttyS4", 115200)

# ---------- 模型加载 ----------
model_path = "/mycode/25E/best.mud"
detector = nn.YOLO11(model=model_path)

cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
dis = display.Display()

img_w = detector.input_width()
img_h = detector.input_height()
half_w = img_w // 2
half_h = img_h // 2

# ---------- 追踪参数 ----------
LOCK_FRAMES = 10
LOCK_THRESH = 15
lock_count = 0
locked = False

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
    objs = detector.detect(img, conf_th=0.1, iou_th=0.45)

    a4_list = [o for o in objs if o.class_id == 0]
    a4 = max(a4_list, key=lambda o: o.score) if a4_list else None

    if a4:
        target_cx = a4.x + a4.w // 2
        target_cy = a4.y + a4.h // 2
        dx = target_cx - half_w
        dy = target_cy - half_h

        img.draw_rect(a4.x, a4.y, a4.w, a4.h, color=image.COLOR_GREEN)
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
        send_cmd(STATE_SCAN, 0, 0)
        img.draw_string(4, 4, 'SCAN', color=image.COLOR_YELLOW)

    img.draw_cross(half_w, half_h, color=image.COLOR_WHITE, size=8)
    dis.show(img)
