"""Explicit opt-in test: existing real camera, Qt preview, start/stop, verification.

Run with PYTHONNOUSERSITE=1 QT_QPA_PLATFORM=offscreen python3 this_file.py.
Not collected by pytest. Creates a new 5-second recording; does not launch camera.
"""
import argparse
import json
import time
import rclpy
from python_qt_binding.QtCore import QObject
from python_qt_binding.QtWidgets import QApplication
from d435i_recorder.plugin import RecorderPlugin


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=5.)
    options = parser.parse_args()
    app = QApplication([])
    rclpy.init()
    node = rclpy.create_node('d435i_ui_smoke')
    class Context(QObject):
        def __init__(self):
            super().__init__()
            self.node = node
        def add_widget(self, widget):
            widget.show()
    context = Context()
    plugin = RecorderPlugin(context)
    plugin.autoplay.setChecked(False)
    def pump():
        rclpy.spin_once(node, timeout_sec=.01)
        app.processEvents()
    try:
        deadline = time.monotonic() + 20
        while plugin.recorder.ready() and time.monotonic() < deadline:
            pump()
        assert plugin.recorder.ready() is None, plugin.recorder.ready()
        plugin.update()
        assert plugin.preview.pixmap() is not None
        assert not plugin.preview.pixmap().isNull()
        plugin.start_button.click()
        assert plugin.recorder.writer is not None, plugin.ui_error
        deadline = time.monotonic() + options.seconds
        while time.monotonic() < deadline:
            pump()
            assert plugin.recorder.writer is not None, plugin.recorder.stop_details
        plugin.stop_button.click()
        deadline = time.monotonic() + 30
        while plugin.recorder.finishing and time.monotonic() < deadline:
            pump()
        plugin.update()
        assert plugin.recorder.result is not None
        assert plugin.recorder.result['status'] != 'FAIL', plugin.recorder.result
        assert plugin.play_button.isEnabled()
        assert plugin.folder_button.isEnabled()
        image_path = plugin.recorder.path / 'preview_ui.png'
        assert plugin.widget.grab().save(str(image_path))
        print(json.dumps(dict(path=str(plugin.recorder.path), status=plugin.status.text(),
                              preview_rendered=True, start_stop_buttons=True,
                              play_folder_enabled=True, result=plugin.recorder.result), indent=2))
    finally:
        plugin.shutdown_plugin()
        plugin.widget.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
