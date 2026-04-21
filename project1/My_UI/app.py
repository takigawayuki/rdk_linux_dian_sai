import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tkinter as tk
import threading
import queue
import time
import cv2
import numpy as np
from PIL import Image, ImageTk

from Drivers.camera import Camera
from Drivers.my_serial import MySerial
from Algorithm.CenterGet import CenterGet

# ── 配置 ──────────────────────────────────────────────
BASE_POINT   = (320, 180)
SERIAL_PORT  = "/dev/ttyS1"
BAUDRATE     = 115200
SEND_INTERVAL = 0.05   # 50ms

LOG_MAX_LINES = 200     # 日志区最多保留行数
CAMERA_W, CAMERA_H = 640, 360
UI_CAM_W, UI_CAM_H = 480, 270   # UI 中摄像头显示尺寸
# ──────────────────────────────────────────────────────


class GimbalApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("云台控制面板")
        self.root.configure(bg="#1e1e2e")
        self.root.attributes("-fullscreen", True)   # 触摸屏全屏

        self._running = False
        self._worker_thread: threading.Thread | None = None
        self._log_queue: queue.Queue = queue.Queue()
        self._frame_queue: queue.Queue = queue.Queue(maxsize=2)

        self._build_ui()
        self._poll_log()
        self._poll_frame()

    # ── UI 构建 ───────────────────────────────────────

    def _build_ui(self):
        # 顶部标题栏
        title_bar = tk.Frame(self.root, bg="#181825", height=50)
        title_bar.pack(fill=tk.X, side=tk.TOP)
        tk.Label(
            title_bar, text="云台自动瞄准控制系统",
            bg="#181825", fg="#cdd6f4",
            font=("song", 16)
        ).pack(side=tk.LEFT, padx=20, pady=8)

        # 退出按钮（触摸屏右上角）
        tk.Button(
            title_bar, text="退出",
            bg="#f38ba8", fg="white",
            font=("song", 13),
            relief=tk.FLAT, padx=14, pady=4,
            command=self._on_exit
        ).pack(side=tk.RIGHT, padx=12, pady=6)

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
        status_frame = tk.Frame(left, bg="#313244", bd=0, relief=tk.FLAT)
        status_frame.pack(fill=tk.X, pady=(10, 6), ipady=10, ipadx=10)

        self._status_dot = tk.Label(
            status_frame, text="●", fg="#6c7086",
            bg="#313244", font=("song", 22)
        )
        self._status_dot.pack(side=tk.LEFT, padx=(14, 6))

        self._status_label = tk.Label(
            status_frame, text="已停止",
            bg="#313244", fg="#6c7086",
            font=("song", 14)
        )
        self._status_label.pack(side=tk.LEFT)

        # 大启动/停止按钮
        self._toggle_btn = tk.Button(
            left, text="启动",
            bg="#a6e3a1", fg="#1e1e2e",
            font=("song", 22),
            relief=tk.FLAT, bd=0,
            activebackground="#94d3a2",
            command=self._toggle,
            height=3
        )
        self._toggle_btn.pack(fill=tk.X, pady=8, ipady=6)

        # 参数信息区
        info_frame = tk.Frame(left, bg="#313244")
        info_frame.pack(fill=tk.X, pady=(0, 8))

        self._fps_label = self._info_row(info_frame, "摄像头 FPS", "—")
        self._serial_label = self._info_row(info_frame, "串口", SERIAL_PORT)
        self._target_label = self._info_row(info_frame, "目标状态", "—")
        self._deta_label = self._info_row(info_frame, "偏差 (x, y)", "—")

        # 日志区
        tk.Label(
            left, text="运行日志",
            bg="#1e1e2e", fg="#a6adc8",
            font=("song", 11)
        ).pack(anchor=tk.W, pady=(4, 2))

        log_frame = tk.Frame(left, bg="#11111b")
        log_frame.pack(fill=tk.BOTH, expand=True)

        scrollbar = tk.Scrollbar(log_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self._log_text = tk.Text(
            log_frame, bg="#11111b", fg="#cdd6f4",
            font=("song", 10),
            relief=tk.FLAT, state=tk.DISABLED,
            yscrollcommand=scrollbar.set,
            wrap=tk.WORD
        )
        self._log_text.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self._log_text.yview)

        # 日志颜色标签
        self._log_text.tag_config("ok",   foreground="#a6e3a1")
        self._log_text.tag_config("warn", foreground="#f9e2af")
        self._log_text.tag_config("err",  foreground="#f38ba8")
        self._log_text.tag_config("info", foreground="#89dceb")

    def _info_row(self, parent, label_text, value_text):
        row = tk.Frame(parent, bg="#313244")
        row.pack(fill=tk.X, padx=10, pady=3)
        tk.Label(
            row, text=label_text + "：",
            bg="#313244", fg="#a6adc8",
            font=("song", 11),
            width=10, anchor=tk.W
        ).pack(side=tk.LEFT)
        val = tk.Label(
            row, text=value_text,
            bg="#313244", fg="#cdd6f4",
            font=("song", 11),
            anchor=tk.W
        )
        val.pack(side=tk.LEFT, fill=tk.X, expand=True)
        return val

    def _build_right_panel(self, parent):
        right = tk.Frame(parent, bg="#1e1e2e")
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 摄像头画面
        cam_outer = tk.Frame(right, bg="#313244", bd=2, relief=tk.FLAT)
        cam_outer.pack(fill=tk.BOTH, expand=True, pady=(10, 0))

        self._cam_label = tk.Label(cam_outer, bg="#11111b")
        self._cam_label.pack(fill=tk.BOTH, expand=True)

        # 无信号占位图
        self._show_no_signal()

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

            # FPS
            frame_count += 1
            now = time.time()
            elapsed = now - fps_ts
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                fps_ts = now
                self.root.after(0, lambda f=fps: self._fps_label.config(text=f"{f:.1f}"))

            # 目标检测
            center = CenterGet(frame)

            # 发送控制
            if now - last_send >= SEND_INTERVAL:
                if center is not None:
                    dx = -(BASE_POINT[0] - center[0]) * 0.001
                    dy = +(BASE_POINT[1] - center[1]) * 0.001
                    ok = ser.send_deta(dx, dy)
                    msg = f"发送偏差: dx={dx:.4f}, dy={dy:.4f}"
                    self._log(msg, "ok" if ok else "err")
                    self.root.after(0, lambda d=f"({dx:.4f}, {dy:.4f})": self._deta_label.config(text=d))
                    self.root.after(0, lambda: self._target_label.config(text="已检测到", fg="#a6e3a1"))
                else:
                    ser.send_data("DETA:0.0000,0.0000\n")
                    self._log("未检测到目标", "warn")
                    self.root.after(0, lambda: self._target_label.config(text="未检测到", fg="#f9e2af"))
                    self.root.after(0, lambda: self._deta_label.config(text="—"))
                last_send = now

            # 推送画面到队列
            annotated = frame.copy()
            if center is not None:
                cv2.circle(annotated, center, 5, (0, 0, 255), -1)
            cv2.circle(annotated, BASE_POINT, 3, (0, 255, 0), -1)
            cv2.putText(annotated, f"FPS:{fps:.1f}", (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            if not self._frame_queue.full():
                self._frame_queue.put_nowait(annotated)

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
            self._toggle_btn.config(text="停止", bg="#f38ba8", activebackground="#e38ba8",
                                    font=("song", 22))
        else:
            self._status_dot.config(fg="#6c7086")
            self._status_label.config(text="已停止", fg="#6c7086")
            self._toggle_btn.config(text="启动", bg="#a6e3a1", activebackground="#94d3a2",
                                    font=("song", 22))

    def _log(self, msg: str, tag: str = "info"):
        ts = time.strftime("%H:%M:%S")
        self._log_queue.put((f"[{ts}] {msg}\n", tag))

    def _poll_log(self):
        try:
            while True:
                text, tag = self._log_queue.get_nowait()
                self._log_text.config(state=tk.NORMAL)
                self._log_text.insert(tk.END, text, tag)
                # 超出行数时裁剪顶部
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
            frame = self._frame_queue.get_nowait()
            self._update_cam(frame)
        except queue.Empty:
            pass
        self.root.after(33, self._poll_frame)   # ~30fps 刷新 UI

    def _update_cam(self, frame: np.ndarray):
        # 自适应显示区域大小
        w = self._cam_label.winfo_width()
        h = self._cam_label.winfo_height()
        if w < 10 or h < 10:
            w, h = UI_CAM_W, UI_CAM_H
        img = cv2.resize(frame, (w, h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(Image.fromarray(img))
        self._cam_label.config(image=photo)
        self._cam_label.image = photo   # 防止 GC

    def _show_no_signal(self):
        w = max(self._cam_label.winfo_width(), UI_CAM_W)
        h = max(self._cam_label.winfo_height(), UI_CAM_H)
        placeholder = np.zeros((h, w, 3), dtype=np.uint8)
        placeholder[:] = (17, 17, 27)
        cv2.putText(placeholder, "NO SIGNAL", (w // 2 - 100, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (108, 112, 134), 2)
        img = cv2.cvtColor(placeholder, cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(Image.fromarray(img))
        self._cam_label.config(image=photo)
        self._cam_label.image = photo


def main():
    root = tk.Tk()
    app = GimbalApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
