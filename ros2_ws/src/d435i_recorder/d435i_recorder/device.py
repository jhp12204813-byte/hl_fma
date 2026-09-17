"""Read SDK information without opening/starting a sensor stream."""
import ctypes as C
from pathlib import Path


def probe(serial='142122070689'):
    libraries = sorted(Path('/opt/ros/humble/lib').glob('**/librealsense2.so.*'))
    if not libraries:
        raise RuntimeError('Installed librealsense shared library missing')
    lib = C.CDLL(str(libraries[-1]))
    pointer, integer = C.c_void_p, C.c_int
    error_type = C.POINTER(pointer)
    def bind(name, result, arguments):
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = result, arguments
        return fn
    message = bind('rs2_get_error_message', C.c_char_p, [pointer])
    free_error = bind('rs2_free_error', None, [pointer])
    def call(name, result, arguments, *values):
        error = pointer()
        value = bind(name, result, arguments + [error_type])(*values, C.byref(error))
        if error.value:
            text = message(error).decode()
            free_error(error)
            raise RuntimeError(text)
        return value
    def release(name, value):
        if value:
            bind(name, None, [pointer])(value)
    version = call('rs2_get_api_version', integer, [])
    context = devices = device = sensors = sensor = None
    try:
        context = call('rs2_create_context', pointer, [integer], version)
        devices = call('rs2_query_devices', pointer, [pointer], context)
        for index in range(call('rs2_get_device_count', integer, [pointer], devices)):
            device = call('rs2_create_device', pointer, [pointer, integer], devices, index)
            identity = call('rs2_get_device_info', C.c_char_p, [pointer, integer], device, 1).decode()
            if identity != serial:
                release('rs2_delete_device', device)
                device = None
                continue
            sensors = call('rs2_query_sensors', pointer, [pointer], device)
            for i in range(call('rs2_get_sensors_count', integer, [pointer], sensors)):
                sensor = call('rs2_create_sensor', pointer, [pointer, integer], sensors, i)
                try:
                    scale = call('rs2_get_depth_scale', C.c_float, [pointer], sensor)
                    return dict(device_depth_scale_m=float(scale),
                                device_depth_scale_source='live SDK rs2_get_depth_scale; no stream opened',
                                librealsense_version=f'{version//10000}.{version//100%100}.{version%100}',
                                librealsense_version_source='live SDK rs2_get_api_version')
                except RuntimeError:
                    pass
                finally:
                    release('rs2_delete_sensor', sensor)
                    sensor = None
            raise RuntimeError('No depth sensor found')
        raise RuntimeError('Front serial not found by SDK')
    finally:
        release('rs2_delete_sensor', sensor)
        release('rs2_delete_sensor_list', sensors)
        release('rs2_delete_device', device)
        release('rs2_delete_device_list', devices)
        release('rs2_delete_context', context)
