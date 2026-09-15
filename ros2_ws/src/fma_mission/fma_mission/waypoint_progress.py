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


def load_waypoints(path):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list):
        raise ValueError('Expected waypoints list')
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
            self._advance()
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
