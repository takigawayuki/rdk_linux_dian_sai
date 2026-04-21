import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tkinter as tk
import threading
import queue
import time
import json
import math
import cv2
import numpy as np
from PIL import Image, ImageTk

from Drivers.camera import Camera
from Drivers.my_serial import MySerial

# ── 配置 ──────────────────────────────────────────────
BASE_POINT    = (320, 180)
SERIAL_PORT   = "/dev/ttyS1"
BAUDRATE      = 115200
SEND_INTERVAL = 0.05

LOG_MAX_LINES = 200
FONT          = ("song", 11)
FONT_LG       = ("song", 18)
FONT_SM       = ("song", 9)

CONFIG_FILE = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'Test', 'center_get_params.json'))

SLIDER_DEFAULTS = {'threshold': 144, 'canny_low': 50,
                   'canny_high': 150, 'min_area': 500, 'blur_kernel': 5}
# ──────────────────────────────────────────────────────


def _calc_center(pts):
    pts = np.array(pts, dtype=np.float32)
    if len(pts) != 4:
        return None
    d1s, d1e = pts[0], pts[2]
    d2s, d2e = pts[1], pts[3]
    a1, b1 = d1e[1]-d1s[1], d1s[0]-d1e[0]
    c1 = d1e[0]*d1s[1] - d1s[0]*d1e[1]
    a2, b2 = d2e[1]-d2s[1], d2s[0]-d2e[0]
    c2 = d2e[0]*d2s[1] - d2s[0]*d2e[1]
    denom = a1*b2 - a2*b1
    if denom != 0:
        x = (b1*c2 - b2*c1) / denom
        y = (a2*c1 - a1*c2) / denom
    else:
        x, y = np.mean(pts[:, 0]), np.mean(pts[:, 1])
    return (int(round(x)), int(round(y)))


def process(frame, thresh_val, canny_low, canny_high, min_area, blur_k):
    """返回 (thresh_bgr, edges_bgr, result_bgr, center)"""
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (blur_k, blur_k), 0)
    _, thresh_img = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY)
    edges   = cv2.Canny(thresh_img, canny_low, canny_high)
    result  = frame.copy()
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_contour = best_approx = best_center = None
    best_score   = -1
    h, w = thresh_img.shape[:2]

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        cv2.drawContours(result, [contour], -1, (80, 80, 80), 1)
        perimeter = cv2.arcLength(contour, True)
        approx    = cv2.approxPolyDP(contour, 0.01*perimeter, True)
        if len(approx) != 4:
            continue
        pts = approx.reshape(4, 2).astype(int)
        bt  = 5
        if not all(bt < p[0] < w-bt and bt < p[1] < h-bt for p in pts):
            continue
        angles = []
        for i in range(4):
            v1 = pts[(i-1)%4] - pts[i]
            v2 = pts[(i+1)%4] - pts[i]
            a  = abs(math.degrees(math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])))
            angles.append(360-a if a > 180 else a)
        if not all(70 < a < 110 for a in angles):
            cv2.drawContours(result, [approx], -1, (255, 100, 0), 1)
            continue
        lengths = [math.hypot(*(pts[i]-pts[(i+1)%4]).tolist()) for i in range(4)]
        if max(lengths)/min(lengths) > 5:
            continue
        score = 0.6*(100 - sum(abs(a-90) for a in angles)/4) + \
                0.4*min(100, area/((w*h)/2)*100)
        cv2.drawContours(result, [approx], -1, (0, 255, 255), 1)
        if score > best_score:
            best_score   = score
            best_contour = contour
            best_approx  = approx
            M = cv2.moments(contour)
            best_center  = _calc_center(pts) if M['m00'] != 0 else None

    if best_contour is not None:
        cv2.drawContours(result, [best_approx], -1, (0, 255, 0), 2)
        if best_center:
            cv2.circle(result, best_center, 6, (0, 0, 255), -1)

    status = f'Center:{best_center}' if best_center else 'Center:None'
    cv2.putText(result, status, (10, h-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    thresh_bgr = cv2.cvtColor(thresh_img, cv2.COLOR_GRAY2BGR)
    edges_bgr  = cv2.cvtColor(edges,      cv2.COLOR_GRAY2BGR)
    return thresh_bgr, edges_bgr, result, best_center


def update_centerget_file(thresh, canny_low, canny_high, min_area):
    path = os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', 'Algorithm', 'CenterGet.py'))
    try:
        with open(path, 'r') as f:
            lines = f.readlines()
        with open(path, 'w') as f:
            for line in lines:
                if line.startswith('threshold_value ='):
                    f.write(f'threshold_value = {thresh}\n')
                elif line.startswith('canny_low_threshold ='):
                    f.write(f'canny_low_threshold = {canny_low}\n')
                elif line.startswith('canny_high_threshold ='):
                    f.write(f'canny_high_threshold = {canny_high}\n')
                elif 'if area <' in line:
                    indent = line[:len(line)-len(line.lstrip())]
                    f.write(f'{indent}if area < {min_area}:\n')
                else:
                    f.write(line)
    except Exception as e:
        print(f'更新 CenterGet.py 失败: {e}')


class GimbalApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("云台控制面板")
        self.root.configure(bg="#1e1e2e")
        self.root.attributes("-fullscreen", True)

        self._running = False
        self._worker_thread = None
        self._log_queue = queue.Queue()
        self._frame_queue = queue.Queue(maxsize=2)

        # 加载滑块参数
        p = SLIDER_DEFAULTS.copy()
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, 'r') as f:
                    p.update(json.load(f))
            except:
                pass

        self._sv_thresh  = tk.IntVar(value=p['threshold'])
        self._sv_clow    = tk.IntVar(value=p['canny_low'])
        self._sv_chigh   = tk.IntVar(value=p['canny_high'])
        self._sv_minarea = tk.IntVar(value=p['min_area'])
        self._sv_blur    = tk.IntVar(value=p['blur_kernel'])

        self._build_ui()
        self._poll_log()
        self._poll_frame()

    def _build_ui(self):
        # 顶部标题栏
        title_bar = tk.Frame(self.root, bg="#181825", height=50)
        title_bar.pack(fill=tk.X, side=tk.TOP)
        tk.Label(title_bar, text="云台自动瞄准控制系统",
                 bg="#181825", fg="#cdd6f4", font=FONT_LG
                 ).pack(side=tk.LEFT, padx=20, pady=8)
        tk.Button(title_bar, text="退出", bg="#f38ba8", fg="white",
                  font=FONT, relief=tk.FLAT, padx=14, pady=4,
                  command=self._on_exit).pack(side=tk.RIGHT, padx=12, pady=6)

        # 主体区域
        body = tk.Frame(self.root, bg="#1e1e2e")
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        self._build_left_panel(body)
        self._build_right_panel(body)

    def _build_left_panel(self, parent):
        left = tk.Frame(parent, bg="#1e1e2e", width=300)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))
        left.pack_propagate(False)

        # 状态指示灯
        sf = tk.Frame(left, bg="#313244")
        sf.pack(fill=tk.X, pady=(10, 6), ipady=8, ipadx=10)
        self._status_dot = tk.Label(sf, text="●", fg="#6c7086",
                                     bg="#313244", font=("song", 20))
        self._status_dot.pack(side=tk.LEFT, padx=(14, 6))
        self._status_label = tk.Label(sf, text="已停止", bg="#313244",
                                       fg="#6c7086", font=("song", 14))
        self._status_label.pack(side=tk.LEFT)

        # 大启动/停止按钮
        self._toggle_btn = tk.Button(
            left, text="启动", bg="#a6e3a1", fg="#1e1e2e",
            font=("song", 22), relief=tk.FLAT, bd=0,
            activebackground="#94d3a2", command=self._toggle, height=3)
        self._toggle_btn.pack(fill=tk.X, pady=8, ipady=6)

        # 参数信息区
        info_frame = tk.Frame(left, bg="#313244")
        info_frame.pack(fill=tk.X, pady=(0, 8))
        self._fps_label    = self._info_row(info_frame, "FPS", "—")
        self._serial_label = self._info_row(info_frame, "串口", SERIAL_PORT)
        self._target_label = self._info_row(info_frame, "目标", "—")
        self._deta_label   = self._info_row(info_frame, "偏差", "—")

        # 日志区
        tk.Label(left, text="运行日志", bg="#1e1e2e", fg="#a6adc8",
                 font=FONT_SM).pack(anchor=tk.W, pady=(4, 2))
        log_frame = tk.Frame(left, bg="#11111b")
        log_frame.pack(fill=tk.BOTH, expand=True)
        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._log_text = tk.Text(log_frame, bg="#11111b", fg="#cdd6f4",
                                 font=FONT_SM, relief=tk.FLAT, state=tk.DISABLED,
                                 yscrollcommand=scrollbar.set, wrap=tk.WORD)
        self._log_text.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self._log_text.yview)
        self._log_text.tag_config("ok",   foreground="#a6e3a1")
        self._log_text.tag_config("warn", foreground="#f9e2af")
        self._log_text.tag_config("err",  foreground="#f38ba8")
        self._log_text.tag_config("info", foreground="#89dceb")

    def _info_row(self, parent, label_text, value_text):
        row = tk.Frame(parent, bg="#313244")
        row.pack(fill=tk.X, padx=10, pady=3)
        tk.Label(row, text=label_text+":", bg="#313244", fg="#a6adc8",
                 font=FONT_SM, width=6, anchor=tk.W).pack(side=tk.LEFT)
        val = tk.Label(row, text=value_text, bg="#313244", fg="#cdd6f4",
                       font=FONT_SM, anchor=tk.W)
        val.pack(side=tk.LEFT, fill=tk.X, expand=True)
        return val

    def _build_right_panel(self, parent):
        right = tk.Frame(parent, bg="#1e1e2e")
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 2×2 画面网格
        grid = tk.Frame(right, bg="#1e1e2e")
        grid.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        # 左上：原始画面（带检测标注）
        tl = tk.Frame(grid, bg="#313244", bd=1, relief=tk.SOLID)
        tl.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)
        tk.Label(tl, text="Camera", bg="#313244", fg="#a6adc8",
                 font=FONT_SM).pack()
        self._cam_orig = tk.Label(tl, bg="#11111b")
        self._cam_orig.pack(fill=tk.BOTH, expand=True)

        # 右上：Threshold
        tr = tk.Frame(grid, bg="#313244", bd=1, relief=tk.SOLID)
        tr.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)
        tk.Label(tr, text="Threshold", bg="#313244", fg="#a6adc8",
                 font=FONT_SM).pack()
        self._cam_thresh = tk.Label(tr, bg="#11111b")
        self._cam_thresh.pack(fill=tk.BOTH, expand=True)

        # 左下：Edges
        bl = tk.Frame(grid, bg="#313244", bd=1, relief=tk.SOLID)
        bl.grid(row=1, column=0, sticky="nsew", padx=2, pady=2)
        tk.Label(bl, text="Edges", bg="#313244", fg="#a6adc8",
                 font=FONT_SM).pack()
        self._cam_edges = tk.Label(bl, bg="#11111b")
        self._cam_edges.pack(fill=tk.BOTH, expand=True)

        # 右下：Result (contours)
        br = tk.Frame(grid, bg="#313244", bd=1, relief=tk.SOLID)
        br.grid(row=1, column=1, sticky="nsew", padx=2, pady=2)
        tk.Label(br, text="Result", bg="#313244", fg="#a6adc8",
                 font=FONT_SM).pack()
        self._cam_result = tk.Label(br, bg="#11111b")
        self._cam_result.pack(fill=tk.BOTH, expand=True)

        grid.rowconfigure(0, weight=1)
        grid.rowconfigure(1, weight=1)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)

        # 底部滑块区
        slider_frame = tk.Frame(right, bg="#313244")
        slider_frame.pack(fill=tk.X, pady=(8, 0), ipady=6)

        self._add_slider(slider_frame, "Threshold", self._sv_thresh, 0, 255, 0)
        self._add_slider(slider_frame, "Canny Low", self._sv_clow, 0, 500, 1)
        self._add_slider(slider_frame, "Canny High", self._sv_chigh, 0, 500, 2)
        self._add_slider(slider_frame, "Min Area", self._sv_minarea, 0, 5000, 3)
        self._add_slider(slider_frame, "Blur Kernel", self._sv_blur, 1, 21, 4)

        # 保存按钮
        tk.Button(slider_frame, text="保存参数", bg="#89dceb", fg="#1e1e2e",
                  font=FONT_SM, relief=tk.FLAT, padx=10, pady=4,
                  command=self._save_params
                  ).grid(row=5, column=0, columnspan=2, pady=(6, 0))

        self._show_no_signal()

    def _add_slider(self, parent, label, var, from_, to, row):
        tk.Label(parent, text=label, bg="#313244", fg="#a6adc8",
                 font=FONT_SM, width=12, anchor=tk.W
                 ).grid(row=row, column=0, padx=(10, 4), pady=2, sticky=tk.W)
        slider = tk.Scale(parent, from_=from_, to=to, orient=tk.HORIZONTAL,
                          variable=var, bg="#313244", fg="#cdd6f4",
                          troughcolor="#11111b", highlightthickness=0,
                          font=FONT_SM, length=300)
        slider.grid(row=row, column=1, padx=(0, 10), pady=2, sticky=tk.EW)
        parent.columnconfigure(1, weight=1)

    # ── 控制逻辑 ──────────────────────────────────────

    def _toggle(self):
        if self._running:
            self._stop()
        else:
            self._start()

    def _start(self):
        self._running = True
        self._set_status(running=True)
        self._log("系统启动中…", "info")
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    def _stop(self):
        self._running = False
        self._set_status(running=False)
        self._log("正在停止…", "warn")

    def _on_exit(self):
        self._running = False
        time.sleep(0.15)
        self.root.destroy()

    def _save_params(self):
        t  = self._sv_thresh.get()
        cl = self._sv_clow.get()
        ch = self._sv_chigh.get()
        ma = self._sv_minarea.get()
        bk = self._sv_blur.get()
        params = {'threshold': t, 'canny_low': cl, 'canny_high': ch,
                  'min_area': ma, 'blur_kernel': bk}
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(params, f, indent=2)
        except Exception as e:
            self._log(f"保存失败: {e}", "err")
            return
        update_centerget_file(t, cl, ch, ma)
        self._log("参数已保存并写入 CenterGet.py", "ok")

    # ── 后台工作线程 ──────────────────────────────────

    def _worker(self):
        cap = Camera()
        if not cap.open():
            self._log("摄像头打开失败", "err")
            self._running = False
            self.root.after(0, lambda: self._set_status(running=False))
            return
        self._log("摄像头已打开", "ok")

        ser = MySerial(port=SERIAL_PORT, baudrate=BAUDRATE)
        if not ser.open():
            self._log(f"串口 {SERIAL_PORT} 打开失败", "err")
            cap.close()
            self._running = False
            self.root.after(0, lambda: self._set_status(running=False))
            return
        self._log(f"串口 {SERIAL_PORT} 已打开", "ok")

        last_send = time.time()
        fps = 0.0
        frame_count = 0
        fps_ts = time.time()

        while self._running:
            frame = cap.capture()
            if frame is None:
                self._log("获取图像帧失败", "warn")
                time.sleep(0.01)
                continue

            frame_count += 1
            now = time.time()
            elapsed = now - fps_ts
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_ts = now
                self.root.after(0, lambda f=fps: self._fps_label.config(text=f"{f:.1f}"))

            # 读取滑块参数
            t  = self._sv_thresh.get()
            cl = self._sv_clow.get()
            ch = self._sv_chigh.get()
            ma = self._sv_minarea.get()
            bk = self._sv_blur.get()
            bk = max(1, bk if bk % 2 == 1 else bk + 1)

            thresh_img, edges_img, result_img, center = process(frame, t, cl, ch, ma, bk)

            # 原始画面标注
            orig = frame.copy()
            if center is not None:
                cv2.circle(orig, center, 5, (0, 0, 255), -1)
            cv2.circle(orig, BASE_POINT, 3, (0, 255, 0), -1)
            cv2.putText(orig, f"FPS:{fps:.1f}", (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # 发送控制
            if now - last_send >= SEND_INTERVAL:
                if center is not None:
                    dx = -(BASE_POINT[0] - center[0]) * 0.001
                    dy = +(BASE_POINT[1] - center[1]) * 0.001
                    ok = ser.send_deta(dx, dy)
                    self._log(f"发送: dx={dx:.4f}, dy={dy:.4f}", "ok" if ok else "err")
                    self.root.after(0, lambda d=f"({dx:.4f},{dy:.4f})": self._deta_label.config(text=d))
                    self.root.after(0, lambda: self._target_label.config(text="已检测到", fg="#a6e3a1"))
                else:
                    ser.send_data("DETA:0.0000,0.0000\n")
                    self._log("未检测到目标", "warn")
                    self.root.after(0, lambda: self._target_label.config(text="未检测到", fg="#f9e2af"))
                    self.root.after(0, lambda: self._deta_label.config(text="—"))
                last_send = now

            if not self._frame_queue.full():
                self._frame_queue.put_nowait((orig, thresh_img, edges_img, result_img))

        cap.close()
        ser.close()
        self._log("系统已停止", "info")
        self.root.after(0, lambda: self._show_no_signal())
        self.root.after(0, lambda: self._target_label.config(text="—", fg="#cdd6f4"))
        self.root.after(0, lambda: self._deta_label.config(text="—"))
        self.root.after(0, lambda: self._fps_label.config(text="—"))

    # ── UI 更新 ───────────────────────────────────────

    def _set_status(self, running: bool):
        if running:
            self._status_dot.config(fg="#a6e3a1")
            self._status_label.config(text="运行中", fg="#a6e3a1")
            self._toggle_btn.config(text="停止", bg="#f38ba8",
                                    activebackground="#e38ba8", font=("song", 22))
        else:
            self._status_dot.config(fg="#6c7086")
            self._status_label.config(text="已停止", fg="#6c7086")
            self._toggle_btn.config(text="启动", bg="#a6e3a1",
                                    activebackground="#94d3a2", font=("song", 22))

    def _log(self, msg: str, tag: str = "info"):
        ts = time.strftime("%H:%M:%S")
        self._log_queue.put((f"[{ts}] {msg}\n", tag))

    def _poll_log(self):
        try:
            while True:
                text, tag = self._log_queue.get_nowait()
                self._log_text.config(state=tk.NORMAL)
                self._log_text.insert(tk.END, text, tag)
                lines = int(self._log_text.index(tk.END).split(".")[0])
                if lines > LOG_MAX_LINES:
                    self._log_text.delete("1.0", f"{lines - LOG_MAX_LINES}.0")
                self._log_text.config(state=tk.DISABLED)
                self._log_text.see(tk.END)
        except queue.Empty:
            pass
        self.root.after(50, self._poll_log)

    def _poll_frame(self):
        try:
            orig, thresh, edges, result = self._frame_queue.get_nowait()
            self._update_cam(self._cam_orig,   orig)
            self._update_cam(self._cam_thresh, thresh)
            self._update_cam(self._cam_edges,  edges)
            self._update_cam(self._cam_result, result)
        except queue.Empty:
            pass
        self.root.after(33, self._poll_frame)

    def _update_cam(self, label: tk.Label, frame: np.ndarray):
        w = label.winfo_width()
        h = label.winfo_height()
        if w < 10 or h < 10:
            w, h = 320, 180
        img = cv2.resize(frame, (w, h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(Image.fromarray(img))
        label.config(image=photo)
        label.image = photo

    def _show_no_signal(self):
        for label in (self._cam_orig, self._cam_thresh,
                      self._cam_edges, self._cam_result):
            w = max(label.winfo_width(), 320)
            h = max(label.winfo_height(), 180)
            ph = np.full((h, w, 3), (17, 17, 27), dtype=np.uint8)
            cv2.putText(ph, "NO SIGNAL", (w//2-80, h//2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (108, 112, 134), 2)
            img = cv2.cvtColor(ph, cv2.COLOR_BGR2RGB)
            photo = ImageTk.PhotoImage(Image.fromarray(img))
            label.config(image=photo)
            label.image = photo


def main():
    root = tk.Tk()
    app = GimbalApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
