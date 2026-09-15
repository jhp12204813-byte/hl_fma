"""C920 controls for competition perception."""
import shutil
import subprocess


def _set_control(device, control):
    result = subprocess.run(
        ["v4l2-ctl", "-d", str(device), f"--set-ctrl={control}"],
        text=True,
        capture_output=True,
        timeout=3,
    )
    if result.returncode:
        raise RuntimeError(
            f"C920 control failed ({control}): "
            + (result.stderr.strip() or result.stdout.strip())
        )


def apply_c920_manual_controls(device, exposure=156, gain=0):
    if not shutil.which("v4l2-ctl"):
        raise RuntimeError("v4l2-ctl not found")

    # These controls are writable on this C920.
    _set_control(device, "auto_exposure=1")
    _set_control(device, "exposure_dynamic_framerate=0")
    _set_control(device, f"gain={gain}")

    result = subprocess.run(
        [
            "v4l2-ctl", "-d", str(device),
            "--get-ctrl=auto_exposure,exposure_time_absolute,"
            "gain,exposure_dynamic_framerate",
        ],
        text=True,
        capture_output=True,
        timeout=3,
    )

    if result.returncode:
        raise RuntimeError(
            "C920 control verification failed: "
            + (result.stderr.strip() or result.stdout.strip())
        )

    output = result.stdout.strip()

    required = [
        "auto_exposure: 1",
        f"exposure_time_absolute: {exposure}",
        f"gain: {gain}",
        "exposure_dynamic_framerate: 0",
    ]

    missing = [item for item in required if item not in output]
    if missing:
        raise RuntimeError(
            "C920 competition controls are not in the validated state:\n"
            + output
        )

    return output
