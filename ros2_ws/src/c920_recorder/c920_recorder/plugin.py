"""rqt UI: latest-frame preview and asynchronous recording finalization."""
import cv2
import os
import json
from pathlib import Path
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation
from python_qt_binding.QtCore import QTimer, QUrl, Qt
from python_qt_binding.QtGui import QImage, QPixmap, QDesktopServices
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QCheckBox
from rqt_gui_py.plugin import Plugin
from .recording import Recorder
from .stop_line import candidates


class RecorderPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('C920Recorder')
        # Keep recorder callbacks out of rqt's shared MultiThreadedExecutor.
        self.ros_node = rclpy.create_node('c920_recorder_' + str(id(self)))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.ros_node)
        self.recorder = Recorder(self.ros_node, **json.loads(os.environ.get("C920_RECORDER_OPTIONS", "{}")))
        self.spin_stop = threading.Event()
        def spin():
            while not self.spin_stop.is_set() and rclpy.ok():
                self.executor.spin_once(timeout_sec=.05)
        self.spinner = threading.Thread(target=spin, name='c920-ros')
        self.spinner.start()
        self.widget = QWidget()
        self.widget.setWindowTitle('C920 Recorder')
        self.widget.setFocusPolicy(Qt.NoFocus)
        layout = QVBoxLayout(self.widget)
        self.preview = QLabel('Waiting for RGB')
        self.preview.setFixedSize(640, 480)
        layout.addWidget(self.preview)
        self.status = QLabel('Initializing')
        self.status.setWordWrap(True)
        self.status.setMinimumHeight(70)
        layout.addWidget(self.status)
        self.location = QLabel(str(self.recorder.root))
        self.location.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.location.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(self.location)
        buttons = QHBoxLayout()
        layout.addLayout(buttons)
        self.start_button = QPushButton('녹화 시작')
        self.stop_button = QPushButton('녹화 종료')
        self.folder_button = QPushButton('저장 폴더 열기')
        self.play_button = QPushButton('RGB MP4 재생')
        for button, action in ((self.start_button, self.start), (self.stop_button, lambda: self.recorder.stop()),
                               (self.folder_button, self.open_folder), (self.play_button, self.play)):
            # Mouse-only controls: SPACE must not activate a focused recorder button.
            button.setFocusPolicy(Qt.NoFocus)
            buttons.addWidget(button)
            button.clicked.connect(action)
        self.autoplay = QCheckBox('PASS일 때 자동 재생')
        # Opening another application can move focus away from keyboard teleop.
        self.autoplay.setChecked(True)
        self.autoplay.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(self.autoplay)
        layout.addWidget(QLabel('흰색 정지선 RGB 후보 · 거리 정보 없음 · 검출 정확도 미검증'))
        self.seen_result = None
        self.ui_error = None
        self.timer = QTimer(self.widget)
        self.timer.timeout.connect(self.update)
        self.timer.start(100)
        context.add_widget(self.widget)

    def start(self):
        try:
            self.recorder.start()
            self.ui_error = None
        except Exception as exc:
            self.ui_error = str(exc)

    def open_folder(self):
        if self.recorder.path:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.recorder.path)))

    def play(self):
        if self.recorder.path and not self.recorder.writer and not self.recorder.finishing:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.recorder.path / 'color.mp4')))

    def update(self):
        rec = self.recorder
        if rec.preview is not None:
            frame = rec.preview.copy()
            boxes = candidates(frame)
            for x, y, w, h in boxes:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
            self.preview.setPixmap(QPixmap.fromImage(image).scaled(640,480,Qt.KeepAspectRatio,Qt.SmoothTransformation))
        active = rec.writer is not None
        self.start_button.setEnabled(not active and not rec.finishing)
        self.stop_button.setEnabled(active)
        self.folder_button.setEnabled(rec.path is not None)
        self.play_button.setEnabled(rec.result is not None and not active and not rec.finishing)
        if rec.path:
            self.location.setText(str(rec.path))
        if rec.finishing:
            text = (rec.stop_details or {}).get('message', 'STOP') + ' · FINALIZING: 저장 및 검증 중'
        elif active:
            text = f'RECORDING: saved={rec.writer.count} queue={rec.writer.queue.qsize()}/{rec.writer.queue.maxsize} peak={rec.writer.queue.peak}'
        elif rec.result:
            text = (rec.stop_details or {}).get('message', '') + ' · ' + rec.result['status'] + ': ' + '; '.join(rec.result.get('failures', []) + rec.result.get('warnings', []))
            if self.seen_result is not rec.result:
                self.seen_result = rec.result
                if rec.result['status'] == 'PASS' and self.autoplay.isChecked():
                    self.play()
        else:
            try:
                text = rec.ready() or 'READY'
            except rclpy_implementation.InvalidHandle:
                self.timer.stop()
                return
        self.status.setText(f'RX {rec.receive_fps:.2f} Hz · ' + (self.ui_error or rec.error or text))

    def shutdown_plugin(self):
        self.timer.stop()
        self.spin_stop.set()
        self.spinner.join()
        self.executor.shutdown()
        self.recorder.shutdown()
        self.ros_node.destroy_node()
