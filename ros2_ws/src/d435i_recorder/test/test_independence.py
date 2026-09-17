"""No live vehicle or motor commands: verify recorder boundaries with test doubles."""
import os
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def test_camera_only_ros_connections():
    from d435i_recorder.recording import Recorder, TOPICS, PREFIX
    subscriptions, clients, profiles = [], [], {}
    def subscribe(typ, topic, callback, qos):
        subscriptions.append(topic)
        profiles[topic] = qos
    node = SimpleNamespace(
        create_subscription=subscribe,
        create_client=lambda typ, service: clients.append(service),
        create_timer=lambda period, callback: None,
    )
    # No create_publisher method: an attempted publisher fails this test.
    Recorder(node)
    assert set(subscriptions) == set(TOPICS.values())
    assert len(subscriptions) == 6
    from rclpy.qos import ReliabilityPolicy
    assert profiles[TOPICS['rgb']].reliability == ReliabilityPolicy.RELIABLE
    assert profiles[TOPICS['depth']].reliability == ReliabilityPolicy.RELIABLE
    assert profiles[TOPICS['imu']].depth == 400
    assert all(topic.startswith('/camera/camera/') for topic in subscriptions)
    assert clients == [PREFIX+'/device_info', PREFIX+'/get_parameters']


def test_teleop_keys_do_not_activate_recorder(monkeypatch):
    from python_qt_binding.QtCore import QObject, Qt
    from python_qt_binding.QtWidgets import QApplication, QShortcut
    from python_qt_binding.QtTest import QTest
    from d435i_recorder import plugin as module

    app = QApplication.instance() or QApplication([])
    rec = Mock()
    rec.writer = None
    rec.finishing = False
    rec.path = None
    monkeypatch.setattr(module, 'Recorder', lambda node: rec)
    monkeypatch.setattr(module.rclpy, 'create_node', lambda name: Mock())
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(module, 'SingleThreadedExecutor', Mock)

    class Context(QObject):
        node = None
        def add_widget(self, widget):
            self.widget = widget

    context = Context()
    plugin = module.RecorderPlugin(context)
    plugin.timer.stop()
    assert plugin.recorder is rec
    assert plugin.ros_node is not context.node
    try:
        assert not plugin.autoplay.isChecked()
        assert not plugin.widget.findChildren(QShortcut)
        controls = [plugin.start_button, plugin.stop_button, plugin.folder_button,
                    plugin.play_button, plugin.autoplay]
        assert all(control.focusPolicy() == Qt.NoFocus for control in controls)
        assert all(control.shortcut().isEmpty() for control in controls)
        plugin.widget.show()
        app.processEvents()
        for key in (Qt.Key_W, Qt.Key_A, Qt.Key_S, Qt.Key_D, Qt.Key_Space, Qt.Key_X):
            QTest.keyClick(plugin.widget, key)
        rec.start.assert_not_called()
        rec.stop.assert_not_called()
        assert not plugin.autoplay.isChecked()
        QTest.mouseClick(plugin.start_button, Qt.LeftButton)
        rec.start.assert_called_once()
    finally:
        plugin.shutdown_plugin()
        plugin.widget.close()
