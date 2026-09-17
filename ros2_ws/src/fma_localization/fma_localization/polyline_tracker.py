"""Pure geometry for dense GPS polyline/lookahead tracking."""

from dataclasses import dataclass
import math


EARTH_RADIUS_M = 6371000.0


def valid_coordinates(latitude, longitude):
    return (
        isinstance(latitude, (int, float))
        and not isinstance(latitude, bool)
        and isinstance(longitude, (int, float))
        and not isinstance(longitude, bool)
        and math.isfinite(latitude)
        and math.isfinite(longitude)
        and -90.0 <= latitude <= 90.0
        and -180.0 <= longitude <= 180.0
    )


@dataclass(frozen=True)
class RoutePoint:
    latitude: float
    longitude: float


@dataclass(frozen=True)
class Projection:
    segment_index: int
    t: float
    s_m: float
    distance_m: float
    x_m: float
    y_m: float


@dataclass(frozen=True)
class LookaheadTarget:
    segment_index: int
    s_m: float
    latitude: float
    longitude: float
    x_m: float
    y_m: float


@dataclass(frozen=True)
class TrackResult:
    projection: Projection
    progress_s_m: float
    target: LookaheadTarget
    finished: bool


class PolylineTracker:
    """Track monotonically along an ordered GPS polyline.

    x = east, y = north in a local metric frame.
    """

    def __init__(self, points):
        if not isinstance(points, (list, tuple)) or len(points) < 2:
            raise ValueError('Polyline requires at least 2 points')

        checked = []
        for index, point in enumerate(points):
            if not isinstance(point, RoutePoint):
                raise ValueError(f'Invalid route point at index {index}')
            if not valid_coordinates(point.latitude, point.longitude):
                raise ValueError(f'Invalid coordinates at index {index}')
            checked.append(point)

        self.points = tuple(checked)
        self.origin_lat = self.points[0].latitude
        self.origin_lon = self.points[0].longitude
        self.origin_lat_rad = math.radians(self.origin_lat)

        self.xy = tuple(
            self._to_xy(p.latitude, p.longitude)
            for p in self.points
        )

        cumulative = [0.0]

        for index in range(len(self.xy) - 1):
            x1, y1 = self.xy[index]
            x2, y2 = self.xy[index + 1]

            length = math.hypot(x2 - x1, y2 - y1)

            if length < 0.05:
                raise ValueError(
                    f'Route points {index} and {index + 1} '
                    f'are too close ({length:.3f} m)'
                )

            cumulative.append(cumulative[-1] + length)

        self.cumulative_s = tuple(cumulative)
        self.total_length_m = self.cumulative_s[-1]

    def _to_xy(self, latitude, longitude):
        lat_rad = math.radians(latitude)
        lon_delta_rad = math.radians(longitude - self.origin_lon)

        x = (
            EARTH_RADIUS_M
            * lon_delta_rad
            * math.cos(self.origin_lat_rad)
        )
        y = (
            EARTH_RADIUS_M
            * (lat_rad - self.origin_lat_rad)
        )

        return x, y

    def _to_latlon(self, x_m, y_m):
        latitude = self.origin_lat + math.degrees(
            y_m / EARTH_RADIUS_M
        )

        longitude = self.origin_lon + math.degrees(
            x_m
            / (EARTH_RADIUS_M * math.cos(self.origin_lat_rad))
        )

        return latitude, longitude

    def project(
        self,
        latitude,
        longitude,
        start_segment=0,
        forward_segments=30,
    ):
        if not valid_coordinates(latitude, longitude):
            raise ValueError('Invalid current coordinates')

        if type(start_segment) is not int:
            raise ValueError('start_segment must be int')

        if type(forward_segments) is not int or forward_segments < 1:
            raise ValueError('forward_segments must be positive int')

        px, py = self._to_xy(latitude, longitude)

        first = max(0, min(start_segment, len(self.xy) - 2))
        last = min(
            len(self.xy) - 2,
            first + forward_segments,
        )

        best = None

        for index in range(first, last + 1):
            x1, y1 = self.xy[index]
            x2, y2 = self.xy[index + 1]

            dx = x2 - x1
            dy = y2 - y1
            length_sq = dx * dx + dy * dy

            t = (
                ((px - x1) * dx + (py - y1) * dy)
                / length_sq
            )
            t = max(0.0, min(1.0, t))

            qx = x1 + t * dx
            qy = y1 + t * dy

            distance = math.hypot(px - qx, py - qy)

            segment_length = math.sqrt(length_sq)
            s_m = (
                self.cumulative_s[index]
                + t * segment_length
            )

            candidate = Projection(
                segment_index=index,
                t=t,
                s_m=s_m,
                distance_m=distance,
                x_m=qx,
                y_m=qy,
            )

            if (
                best is None
                or candidate.distance_m < best.distance_m
            ):
                best = candidate

        return best

    def point_at_s(self, s_m):
        if not isinstance(s_m, (int, float)) or not math.isfinite(s_m):
            raise ValueError('s_m must be finite')

        s_m = max(0.0, min(float(s_m), self.total_length_m))

        if s_m >= self.total_length_m:
            index = len(self.xy) - 2
            x_m, y_m = self.xy[-1]
            latitude, longitude = self._to_latlon(x_m, y_m)

            return LookaheadTarget(
                segment_index=index,
                s_m=self.total_length_m,
                latitude=latitude,
                longitude=longitude,
                x_m=x_m,
                y_m=y_m,
            )

        index = 0

        while (
            index < len(self.cumulative_s) - 2
            and self.cumulative_s[index + 1] < s_m
        ):
            index += 1

        start_s = self.cumulative_s[index]
        end_s = self.cumulative_s[index + 1]
        segment_length = end_s - start_s

        t = (s_m - start_s) / segment_length

        x1, y1 = self.xy[index]
        x2, y2 = self.xy[index + 1]

        x_m = x1 + t * (x2 - x1)
        y_m = y1 + t * (y2 - y1)

        latitude, longitude = self._to_latlon(x_m, y_m)

        return LookaheadTarget(
            segment_index=index,
            s_m=s_m,
            latitude=latitude,
            longitude=longitude,
            x_m=x_m,
            y_m=y_m,
        )

    def track(
        self,
        latitude,
        longitude,
        previous_s_m=0.0,
        previous_segment=0,
        lookahead_m=1.8,
        forward_segments=30,
        finish_tolerance_m=1.0,
    ):
        numeric = (
            previous_s_m,
            lookahead_m,
            finish_tolerance_m,
        )

        if not all(
            isinstance(v, (int, float))
            and not isinstance(v, bool)
            and math.isfinite(v)
            for v in numeric
        ):
            raise ValueError('Tracking parameters must be finite')

        if previous_s_m < 0:
            raise ValueError('previous_s_m must be nonnegative')

        if lookahead_m <= 0:
            raise ValueError('lookahead_m must be positive')

        if finish_tolerance_m < 0:
            raise ValueError(
                'finish_tolerance_m must be nonnegative'
            )

        # Search one segment behind the remembered segment to avoid
        # numerical trouble exactly at a segment boundary.
        search_start = max(0, previous_segment - 1)

        projection = self.project(
            latitude,
            longitude,
            start_segment=search_start,
            forward_segments=forward_segments,
        )

        # Route progress never moves backwards.
        progress_s_m = max(
            float(previous_s_m),
            projection.s_m,
        )

        target_s_m = min(
            self.total_length_m,
            progress_s_m + lookahead_m,
        )

        target = self.point_at_s(target_s_m)

        remaining_m = self.total_length_m - progress_s_m
        finished = remaining_m <= finish_tolerance_m

        return TrackResult(
            projection=projection,
            progress_s_m=progress_s_m,
            target=target,
            finished=finished,
        )
