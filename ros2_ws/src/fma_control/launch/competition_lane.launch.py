"""Competition tracker with the existing gated control/STOP launch wiring."""
import importlib.util
from pathlib import Path


def generate_launch_description():
    spec = importlib.util.spec_from_file_location('field_stop_launch', Path(__file__).with_name('lane_stop_test.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description(competition=True)
