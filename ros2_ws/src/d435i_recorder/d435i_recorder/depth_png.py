"""16-bit grayscale PNG, filter None and zlib level 0; lossless, bounded CPU cost.

Avoids libpng's per-row filter search. Bigger files (~615 KB at 640x480)
trade ~18.5 MB/s at 30 FPS for a lower encoding cost. No depth quantization.
"""
import struct
import zlib
import numpy as np


def _chunk(kind, data):
    return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind + data))


def write_depth_png(path, depth):
    if depth.dtype != np.uint16 or depth.ndim != 2:
        raise ValueError('depth must be a native uint16 plane')
    height, width = depth.shape
    scanlines = np.empty((height, width * 2 + 1), dtype=np.uint8)
    scanlines[:, 0] = 0  # PNG filter None, not a lossy conversion.
    scanlines[:, 1:] = depth.astype('>u2').view(np.uint8).reshape(height, width * 2)
    data = (b'\x89PNG\r\n\x1a\n'
            + _chunk(b'IHDR', struct.pack('!2I5B', width, height, 16, 0, 0, 0, 0))
            + _chunk(b'IDAT', zlib.compress(scanlines.tobytes(), level=0))
            + _chunk(b'IEND', b''))
    with open(path, 'xb') as handle:
        handle.write(data)
