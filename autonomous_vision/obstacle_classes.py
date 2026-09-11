"""Obstacle-only IDs; never use these IDs to rewrite legacy annotations."""
CLASS_NAMES = ["child_dummy", "vehicle_obstacle"]
LEGACY_CLASS_MAPPING = {5: 0, 6: 1}
CLASS_DESCRIPTIONS = ["어린이 더미 + 모자 + 받침대/바퀴의 보이는 전체 (별도 성인 제외, 객체당 박스 1개)", "차량 형태 장애물의 보이는 차체 전체 (객체당 박스 1개)"]
