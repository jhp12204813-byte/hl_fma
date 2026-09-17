"""Sequential waypoint triggers; no ROS graph, steering or maneuver control."""
from dataclasses import dataclass
import math

import yaml
from fma_interfaces.msg import MissionState


MISSION_NAMES = (
    'RAMP', 'INTERSECTION_STRAIGHT_1', 'S_CURVE', 'INTERSECTION_STRAIGHT_2',
    'PERPENDICULAR_PARKING', 'INTERSECTION_LEFT', 'CHILD_DUMMY',
    'PARALLEL_PARKING', 'INTERSECTION_RIGHT', 'SIGNAL_CAR', 'LANE_CHANGE', 'FINISH')


def valid_coordinates(latitude, longitude):
    return (isinstance(latitude, (int, float)) and not isinstance(latitude, bool)
            and isinstance(longitude, (int, float)) and not isinstance(longitude, bool)
            and math.isfinite(latitude) and math.isfinite(longitude)
            and -90 <= latitude <= 90 and -180 <= longitude <= 180)


def haversine_m(lat1, lon1, lat2, lon2):
    if not valid_coordinates(lat1, lon1) or not valid_coordinates(lat2, lon2):
        raise ValueError('Invalid latitude/longitude')
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2)**2
    return 6371000.0 * 2 * math.asin(math.sqrt(max(0., min(1., a))))


@dataclass(frozen=True)
class Waypoint:
    id: str
    latitude: float
    longitude: float
    activation_radius_m: float
    missions: tuple
    type: str = 'mission'
    number: int = 0
    role: str = ''
    waypoint_type: str = ''
    mission: str = None
    mission_exit: str = None
    mission_entry: str = None
    recommended_control_mode: str = 'LANE'
    hard_point: bool = True
    phase: str = None
    completion_policy: str = 'gps'



def load_legacy_waypoints(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list):
        raise ValueError('Expected waypoints list')
    if len(data['waypoints']) != 11:
        raise ValueError('Expected exactly WP01..WP11')
    result = []
    expected = [(name,) for name in MISSION_NAMES[:9]] + [
        ('SIGNAL_CAR', 'LANE_CHANGE'), ('FINISH',)]
    for index, row in enumerate(data['waypoints']):
        if not isinstance(row, dict):
            raise ValueError('Invalid waypoint record')
        radius = row.get('activation_radius_m')
        if (not valid_coordinates(row.get('latitude'), row.get('longitude'))
                or not isinstance(radius, (int, float)) or isinstance(radius, bool)
                or not math.isfinite(radius) or radius <= 0):
            raise ValueError('Invalid coordinates or activation radius')
        if (row.get('id') != f'WP{index + 1:02d}'
                or row.get('missions') != list(expected[index])
                or row.get('type', 'mission') != ('finish' if index == 10 else 'mission')):
            raise ValueError('Waypoint order, missions or finish type mismatch')
        result.append(Waypoint(row['id'], row['latitude'], row['longitude'], radius,
                               tuple(row['missions']), row.get('type', 'mission')))
    return tuple(result)


class LegacyWaypointProgress:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.target_index = 0
        self.mission_index = 0
        self.state = 'START'

    @property
    def target(self):
        return self.waypoints[self.target_index]

    @property
    def active(self):
        return self.state not in ('START', 'NORMAL_DRIVE', 'FINISH')

    def start(self):
        if self.state == 'START':
            self.state = 'NORMAL_DRIVE'

    def gps(self, latitude, longitude, fix_valid=True):
        if self.state != 'NORMAL_DRIVE' or not fix_valid or not valid_coordinates(latitude, longitude):
            return None
        target = self.target  # Only this target is ever measured; never nearest-neighbor.
        distance = haversine_m(latitude, longitude, target.latitude, target.longitude)
        if distance <= target.activation_radius_m:
            self.mission_index = 0
            self.state = target.missions[0]
        return distance

    def complete(self, mission, completed):
        if not self.active or not completed or mission != getattr(MissionState, self.state):
            return False
        if self.mission_index + 1 < len(self.target.missions):
            self.mission_index += 1
            self.state = self.target.missions[self.mission_index]
        else:
            self.target_index += 1
            self.mission_index = 0
            self.state = 'NORMAL_DRIVE'
        return True


WAYPOINT_TYPES = ('route', 'mission_approach', 'mission_entry', 'mission_guide',
                  'mission_exit', 'mission_transition')
COMPLETION_POLICIES = ('gps', 'status', 'gps_and_status')
COURSE_MISSIONS = MISSION_NAMES[:10]
BOUNDARIES = ((4, 5), (22, 24), (30, 33), (35, 37), (37, 40),
              (51, 55), (67, 76), (81, 82), (83, 86), (89, 90))
SOFT_NUMBERS = {31, 32, *range(68, 76)}



def load_competition_dense_waypoints(data):
    """Load a variable-length dense GPS route.

    This loader is intentionally separate from the legacy 1..90 competition
    validation. Dense route IDs are P001, P002, ... and the number of points
    is not fixed.
    """
    rows = data.get('waypoints')
    if not isinstance(rows, list):
        raise ValueError('competition_dense requires a waypoints list')
    if not rows:
        raise ValueError('competition_dense route is empty')

    defaults = data.get('defaults', {})
    if not isinstance(defaults, dict):
        raise ValueError('competition_dense defaults must be a mapping')

    default_radius = defaults.get('activation_radius_m', 2.0)
    if (type(default_radius) not in (float, int)
            or not math.isfinite(default_radius)
            or default_radius <= 0):
        raise ValueError('Invalid competition_dense default activation radius')

    route = data.get('route', {})
    if not isinstance(route, dict):
        raise ValueError('competition_dense route must be a mapping')
    if route.get('controller') != 'polyline_lookahead':
        raise ValueError(
            'competition_dense route controller must be polyline_lookahead'
        )

    lookahead = defaults.get('lookahead_m', 1.8)
    if (type(lookahead) not in (float, int)
            or not math.isfinite(lookahead)
            or lookahead <= 0):
        raise ValueError('Invalid competition_dense lookahead')

    result = []

    for index, row in enumerate(rows, 1):
        if not isinstance(row, dict):
            raise ValueError(
                f'Invalid competition_dense waypoint record at {index}'
            )

        expected_id = f'P{index:03d}'
        if row.get('id') != expected_id:
            raise ValueError(
                f'competition_dense waypoint IDs must be ordered '
                f'P001..; expected {expected_id}'
            )

        lat = row.get('latitude')
        lon = row.get('longitude')
        if not valid_coordinates(lat, lon):
            raise ValueError(
                f'Invalid competition_dense coordinates at {expected_id}'
            )

        radius = row.get('activation_radius_m', default_radius)
        if (type(radius) not in (float, int)
                or not math.isfinite(radius)
                or radius <= 0):
            raise ValueError(
                f'Invalid activation radius at {expected_id}'
            )

        kind = row.get('waypoint_type', 'route')
        if kind != 'route':
            raise ValueError(
                'competition_dense currently accepts route points only'
            )

        mission_entry = row.get('mission_entry')
        mission_exit = row.get('mission_exit')

        if (
            mission_entry is not None
            and mission_entry not in COURSE_MISSIONS
        ):
            raise ValueError(
                f'Invalid dense mission_entry at {expected_id}: '
                f'{mission_entry}'
            )

        if (
            mission_exit is not None
            and mission_exit not in COURSE_MISSIONS
        ):
            raise ValueError(
                f'Invalid dense mission_exit at {expected_id}: '
                f'{mission_exit}'
            )

        result.append(Waypoint(
            id=expected_id,
            latitude=float(lat),
            longitude=float(lon),
            activation_radius_m=float(radius),
            missions=(),
            number=index,
            role=row.get('role', 'dense_route'),
            waypoint_type='route',
            mission=None,
            mission_entry=mission_entry,
            mission_exit=mission_exit,
            recommended_control_mode='GPS',
            hard_point=True,
            phase=None,
            completion_policy='gps',
        ))

    if len({w.id for w in result}) != len(result):
        raise ValueError('Duplicate competition_dense waypoint id')

    return tuple(result)


def load_school_test_waypoints(data):
    rows = data.get('waypoints')
    if not isinstance(rows, list) or len(rows) != 10:
        raise ValueError('school_test requires exactly 10 waypoints (4..13)')

    result = []
    for number, row in enumerate(rows, 4):
        if not isinstance(row, dict):
            raise ValueError('Invalid school_test waypoint record')
        if type(row.get('number')) is not int or row['number'] != number:
            raise ValueError('school_test waypoint numbers must be ordered 4..13')

        lat = row.get('latitude')
        lon = row.get('longitude')
        if not valid_coordinates(lat, lon):
            raise ValueError(f'Invalid school_test coordinates at {number}')

        radius = row.get('activation_radius_m')
        if (type(radius) not in (float, int)
                or not math.isfinite(radius) or radius <= 0):
            raise ValueError('Invalid school_test activation radius')

        if row.get('waypoint_type') != 'route':
            raise ValueError('school_test waypoints must all be route')
        if row.get('mission') is not None:
            raise ValueError('school_test must not contain missions')
        if row.get('recommended_control_mode') != 'GPS':
            raise ValueError('school_test route control mode must be GPS')
        if row.get('hard_point') is not True:
            raise ValueError('school_test waypoints must be hard points')
        if row.get('phase') is not None:
            raise ValueError('school_test route phase must be null')

        waypoint_id = row.get('id')
        if not isinstance(waypoint_id, str) or not waypoint_id:
            raise ValueError('Missing school_test waypoint id')

        result.append(Waypoint(
            waypoint_id,
            lat,
            lon,
            radius,
            (),
            number=number,
            role=row.get('role', ''),
            waypoint_type='route',
            mission=None,
            recommended_control_mode='GPS',
            hard_point=True,
            phase=None,
            completion_policy='gps',
        ))

    if len({w.id for w in result}) != 10:
        raise ValueError('Duplicate school_test waypoint id')

    return tuple(result)


def load_waypoints(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list):
        raise ValueError('Expected waypoints list')

    if data.get('profile') == 'competition_dense':
        return load_competition_dense_waypoints(data)

    if data.get('profile') == 'school_test':
        return load_school_test_waypoints(data)

    rows = data['waypoints']
    if rows and isinstance(rows[0], dict) and 'number' not in rows[0]:
        return load_legacy_waypoints(path)
    if len(rows) != 90:
        raise ValueError('Expected exactly numbers 1..90')
    policies = data.get('completion_policies', {})
    if (not isinstance(policies, dict) or set(policies) != set(COURSE_MISSIONS)
            or any(v not in COMPLETION_POLICIES for v in policies.values())):
        raise ValueError('Explicit completion policy required for every course mission')
    result = []
    for number, row in enumerate(rows, 1):
        if not isinstance(row, dict) or type(row.get('number')) is not int or row['number'] != number:
            raise ValueError('Waypoint numbers must be ordered 1..90')
        lat, lon = row.get('latitude'), row.get('longitude')
        if number == 57:
            if lat is not None or lon is not None:
                raise ValueError('Waypoint 57 coordinates must remain null')
        elif not valid_coordinates(lat, lon):
            raise ValueError(f'Invalid coordinates at {number}')
        radius = row.get('activation_radius_m')
        if (type(radius) not in (float, int) or not math.isfinite(radius) or radius <= 0):
            raise ValueError('Invalid activation radius')
        kind = row.get('waypoint_type')
        if kind not in WAYPOINT_TYPES or type(row.get('hard_point')) is not bool:
            raise ValueError('Invalid waypoint type or hard_point')
        if row['hard_point'] != (number not in SOFT_NUMBERS):
            raise ValueError('Invalid hard/soft waypoint assignment')
        if not isinstance(row.get('id'), str) or not row['id']:
            raise ValueError('Missing waypoint id')
        if row.get('recommended_control_mode') not in ('LANE', 'GPS', 'STOP', 'OBSTACLE', 'REVERSE', 'FINISH'):
            raise ValueError('Invalid control mode')
        if row.get('phase') is not None and not isinstance(row['phase'], str):
            raise ValueError('Invalid phase')
        mission = row.get('mission')
        if mission is not None and mission not in COURSE_MISSIONS:
            raise ValueError('Invalid mission')
        if kind == 'route' and mission is not None:
            raise ValueError('Route must not activate a mission')
        if kind not in ('route', 'mission_transition') and mission is None:
            raise ValueError('Mission waypoint requires mission')
        if (kind == 'mission_transition') != (number == 37):
            raise ValueError('Only waypoint 37 is mission_transition')
        if number == 37 and (row.get('mission_exit'), row.get('mission_entry')) != (
                'INTERSECTION_STRAIGHT_2', 'PERPENDICULAR_PARKING'):
            raise ValueError('Invalid waypoint 37 transition')
        result.append(Waypoint(row['id'], lat, lon, radius, (), number=number,
            role=row.get('role', ''), waypoint_type=kind, mission=mission,
            mission_exit=row.get('mission_exit'), mission_entry=row.get('mission_entry'),
            recommended_control_mode=row['recommended_control_mode'],
            hard_point=row['hard_point'], phase=row.get('phase'),
            completion_policy=policies.get(row.get('mission_entry') or mission, 'gps')))
    if len({w.id for w in result}) != 90:
        raise ValueError('Duplicate waypoint id')
    for mission, (entry, end) in zip(COURSE_MISSIONS, BOUNDARIES):
        first, last = result[entry - 1], result[end - 1]
        if ((first.mission_entry if entry == 37 else first.mission) != mission
                or first.waypoint_type != ('mission_transition' if entry == 37 else 'mission_entry')
                or (last.mission_exit if end == 37 else last.mission) != mission
                or last.waypoint_type != ('mission_transition' if end == 37 else 'mission_exit')):
            raise ValueError('Mission boundary mismatch')
        for w in result[entry:end - 1]:
            if w.waypoint_type != 'mission_guide' or w.mission != mission:
                raise ValueError('Mission guide mismatch')
    approaches = {3: 'RAMP', 21: 'INTERSECTION_STRAIGHT_1', 29: 'S_CURVE',
                  34: 'INTERSECTION_STRAIGHT_2', 66: 'CHILD_DUMMY',
                  80: 'PARALLEL_PARKING', 88: 'SIGNAL_CAR'}
    mission_numbers = {n for entry, end in BOUNDARIES for n in range(entry, end + 1)}
    for w in result:
        if w.number in approaches:
            if w.waypoint_type != 'mission_approach' or w.mission != approaches[w.number]:
                raise ValueError('Approach mismatch')
        elif w.number not in mission_numbers and w.waypoint_type != 'route':
            raise ValueError('Expected ordinary route point')
    return tuple(result)


class WaypointProgress(LegacyWaypointProgress):
    """Ordered boundary events, independent of maneuver control.

    Status-only completion substitutes for exit GPS, once the exit is eligible.
    It never skips preceding hard route/guide points. Soft guides can be bypassed.
    """
    def __init__(self, waypoints):
        super().__init__(waypoints)
        self.legacy = not waypoints[0].number
        self.phase = None
        self.policy = 'gps'
        self.status_received = False
        self.exit_reached = False
        self.processed = set()
        self.events = []

    def _advance(self, skipped=False):
        w = self.target
        self.processed.add(w.number)
        self.events.append((w, 'skipped' if skipped else 'reached'))
        if self.target_index < len(self.waypoints) - 1:
            self.target_index += 1
        if self.target.number == 57 and 57 not in self.processed:
            self._advance(skipped=True)

    def _enter(self, mission, waypoint):
        self.state = mission
        self.phase = waypoint.phase
        self.policy = waypoint.completion_policy
        self.status_received = False
        self.exit_reached = False

    def _boundary(self):
        w = self.target
        if w.waypoint_type not in ('mission_exit', 'mission_transition'):
            return False
        expected = w.mission_exit if w.waypoint_type == 'mission_transition' else w.mission
        if self.state != expected or w.number in self.processed:
            return False
        ready = {'gps': self.exit_reached, 'status': self.status_received,
                 'gps_and_status': self.exit_reached and self.status_received}[self.policy]
        if not ready:
            return False
        self.phase = w.phase
        if w.waypoint_type == 'mission_transition':
            self._enter(w.mission_entry, w)
        else:
            self.state = 'FINISH' if w.number == 90 else 'NORMAL_DRIVE'
            self.phase = w.phase
            self.status_received = False
            self.exit_reached = False
        self._advance()
        return True

    def _status_boundary(self):
        if self.policy != 'status' or not self.status_received:
            return False
        while not self.target.hard_point:
            self._advance(skipped=True)
        return self._boundary()

    def gps(self, latitude, longitude, fix_valid=True):
        if self.legacy:
            return super().gps(latitude, longitude, fix_valid)
        if self.state in ('START', 'FINISH') or not fix_valid or not valid_coordinates(latitude, longitude):
            return None
        # Look ahead only through contiguous soft guides, including their next boundary.
        start = self.target_index
        end = start
        while not self.waypoints[end].hard_point and end < len(self.waypoints) - 1:
            end += 1
        matched = None
        distance = None
        for index in range(start, end + 1):
            w = self.waypoints[index]
            d = haversine_m(latitude, longitude, w.latitude, w.longitude)
            if index == start:
                distance = d
            if d <= w.activation_radius_m:
                matched = index
                break
        if matched is None:
            return distance
        while self.target_index < matched:
            self._advance(skipped=True)
        w = self.target
        if w.waypoint_type in ('mission_exit', 'mission_transition'):
            self.exit_reached = True
            # Transition APPROACH belongs to the next mission, only after commit.
            if w.waypoint_type == 'mission_exit':
                self.phase = w.phase
            self._boundary()
        else:
            if w.waypoint_type == 'mission_entry':
                self._enter(w.mission, w)
            elif w.waypoint_type == 'mission_guide':
                parking_phases = ('APPROACH', 'ALIGN', 'REVERSE', 'PARKED', 'COMPLETE')
                if (self.state not in ('PERPENDICULAR_PARKING', 'PARALLEL_PARKING')
                        or self.phase not in parking_phases
                        or parking_phases.index(w.phase) >= parking_phases.index(self.phase)):
                    self.phase = w.phase
            elif w.waypoint_type == 'route':
                self.phase = None

            final_route = (
                w.waypoint_type == 'route'
                and self.target_index == len(self.waypoints) - 1
            )

            self._advance()

            if final_route:
                self.state = 'FINISH'

            self._status_boundary()
        return distance

    def complete(self, mission, completed):
        if self.legacy:
            return super().complete(mission, completed)
        if (not self.active or not completed or mission != getattr(MissionState, self.state)
                or self.status_received):
            return False
        self.status_received = True
        self._status_boundary()
        self._boundary()
        return True


class DenseRouteProgress:
    """Progress state for variable-length polyline competition routes.

    Driving geometry is owned by dense_gps_controller.
    This class only consumes monotonic route progress and owns mission state.
    Mission anchors will be added separately.
    """

    def __init__(self, waypoints):
        if not waypoints or len(waypoints) < 2:
            raise ValueError(
                'Dense route requires at least 2 waypoints'
            )

        for index, waypoint in enumerate(waypoints, 1):
            if waypoint.number != index:
                raise ValueError(
                    'Dense waypoint numbers must be sequential'
                )
            if waypoint.id != f'P{index:03d}':
                raise ValueError(
                    'Dense waypoint IDs must be P001..'
                )
            if waypoint.waypoint_type != 'route':
                raise ValueError(
                    'DenseRouteProgress currently accepts route points only'
                )

        self.waypoints = tuple(waypoints)

        self.target_index = 0
        self.segment_index = 0

        self.progress_s_m = 0.0
        self.total_length_m = None

        self.state = 'START'
        self.phase = None

        # Compatibility with MissionManagerNode diagnostics.
        self.legacy = False
        self.events = []

    @property
    def target(self):
        return self.waypoints[self.target_index]

    @property
    def active(self):
        return self.state not in (
            'START',
            'NORMAL_DRIVE',
            'FINISH',
        )

    def start(self):
        if self.state != 'START':
            return

        self.state = 'NORMAL_DRIVE'

        # P001 is the initial dense route point.
        self._process_boundary(self.waypoints[0])

    def gps(self, latitude, longitude, fix_valid=True):
        # Dense driving progress comes from /mission/route_progress,
        # never from point-to-point waypoint activation.
        return None

    def _process_boundary(self, waypoint):
        changed = False

        # Mission exit is processed before a possible entry at
        # the same route point.
        if waypoint.mission_exit is not None:
            if self.state == waypoint.mission_exit:
                self.state = 'NORMAL_DRIVE'
                self.phase = None
                self.events.append((waypoint, 'mission_exit'))
                changed = True

        if waypoint.mission_entry is not None:
            self.state = waypoint.mission_entry
            self.phase = waypoint.phase
            self.events.append((waypoint, 'mission_entry'))
            changed = True

        return changed

    def route_progress(
        self,
        segment_index,
        progress_s_m,
        total_length_m,
        finished,
    ):
        if self.state == 'FINISH':
            return False

        if (
            type(segment_index) is not int
            or segment_index < 0
            or segment_index > len(self.waypoints) - 2
        ):
            raise ValueError('Invalid dense segment_index')

        if (
            type(progress_s_m) not in (int, float)
            or isinstance(progress_s_m, bool)
            or not math.isfinite(progress_s_m)
            or progress_s_m < 0
        ):
            raise ValueError('Invalid dense progress_s_m')

        if (
            type(total_length_m) not in (int, float)
            or isinstance(total_length_m, bool)
            or not math.isfinite(total_length_m)
            or total_length_m <= 0
        ):
            raise ValueError('Invalid dense total_length_m')

        if type(finished) is not bool:
            raise ValueError('Invalid dense finished flag')

        progress_s_m = float(progress_s_m)
        total_length_m = float(total_length_m)

        if progress_s_m > total_length_m + 1e-6:
            raise ValueError(
                'Dense progress exceeds route length'
            )

        if self.total_length_m is None:
            self.total_length_m = total_length_m
        elif abs(
            total_length_m - self.total_length_m
        ) > 0.5:
            raise ValueError(
                'Dense route length changed unexpectedly'
            )

        # Never allow delayed/out-of-order messages to move progress backward.
        if (
            progress_s_m < self.progress_s_m
            or segment_index < self.segment_index
        ):
            return False

        previous_segment_index = self.segment_index

        changed = (
            progress_s_m != self.progress_s_m
            or segment_index != self.segment_index
            or finished
        )

        self.progress_s_m = progress_s_m
        self.segment_index = segment_index

        # segment 0 = P001 -> P002.
        # When segment index advances, every crossed waypoint is
        # processed in order for mission entry/exit.
        boundary_changed = False

        if segment_index > previous_segment_index:
            for waypoint_index in range(
                previous_segment_index + 1,
                segment_index + 1,
            ):
                waypoint = self.waypoints[waypoint_index]
                if self._process_boundary(waypoint):
                    boundary_changed = True

        # Segment 0 = P001 -> P002, so P002 is the diagnostic target.
        self.target_index = min(
            segment_index + 1,
            len(self.waypoints) - 1,
        )

        if finished:
            self.target_index = len(self.waypoints) - 1
            self.state = 'FINISH'

        return changed or boundary_changed

    def complete(self, mission, completed):
        # Mission completion handling will be enabled when dense
        # mission anchors are added.
        return False
