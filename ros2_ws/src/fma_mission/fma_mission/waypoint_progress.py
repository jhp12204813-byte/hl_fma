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


def load_waypoints(path):
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


class WaypointProgress:
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
