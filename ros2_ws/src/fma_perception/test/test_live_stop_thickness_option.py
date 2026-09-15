"""CLI validation without camera/GUI access."""
import importlib.util
from pathlib import Path
import sys
from unittest.mock import MagicMock
import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
sys.path.insert(0, str(TOOLS))
spec = importlib.util.spec_from_file_location('live_thickness_test', TOOLS / 'live_c920_lane_stop.py')
live = importlib.util.module_from_spec(spec)
spec.loader.exec_module(live)


@pytest.mark.parametrize('value,expected', [(None, .30), ('0.30', .30)])
def test_live_min_thickness_configuration(monkeypatch, value, expected):
    captured = []
    class Done(Exception): pass
    def detector(**kwargs):
        captured.append(kwargs['config'])
        raise Done
    monkeypatch.setattr(live, 'StopLineDetector', detector)
    camera = MagicMock(side_effect=AssertionError('No hardware'))
    monkeypatch.setattr(live.cv2, 'VideoCapture', camera)
    with pytest.raises(Done): live.main([] if value is None else ['--stop-min-thickness-m', value])
    assert captured[0].min_thickness_m == expected
    assert captured[0].max_thickness_m == .55
    # Keep the generic detector default unchanged; only live competition/field diagnostics default to 0.30 m.
    assert live.StopLineConfig().min_thickness_m == .04
    camera.assert_not_called()


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf', '0.56'])
def test_bad_thickness_before_camera(monkeypatch, value):
    camera = MagicMock(side_effect=AssertionError('No hardware'))
    monkeypatch.setattr(live.cv2, 'VideoCapture', camera)
    with pytest.raises(ValueError): live.main(['--stop-min-thickness-m', value])
    camera.assert_not_called()
