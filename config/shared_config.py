import threading
from config.config import (
    MOVE_STEP, TOLERANCE, LOOKAHEAD_MIN, LOOKAHEAD_MAX, GOAL_WEIGHT,
    OBSTACLE_RADIUS, SPEED_FACTOR, STEERING_SMOOTHING, HEADING_SMOOTHING, WEIGHT_FACTORS
)

# 로깅 설정
import logging
logging.basicConfig(level=logging.DEBUG)

# 전역 상태
SHARED = {
    'speed_data': [],
    'del_playerPos_x': [],
    'del_playerPos_z': [],
    'player_pos': [],
    'lidar_points': [],  # 필터링된 LiDAR 포인트
    'obstacle_clusters': [],  # DBSCAN 클러스터
    'tank_tar_val_kh': 0.0,
    'pid': {
        'kp': 0.5,
        'ki': 0.0,
        'kd': 0.0,
        'integral': 0.0,
        'prev_error': 0.0
    },
    'CONFIG_PARAMS': {
        'MOVE_STEP': MOVE_STEP,
        'TOLERANCE': TOLERANCE,
        'LOOKAHEAD_MIN': LOOKAHEAD_MIN,
        'LOOKAHEAD_MAX': LOOKAHEAD_MAX,
        'GOAL_WEIGHT': GOAL_WEIGHT,
        'OBSTACLE_RADIUS': OBSTACLE_RADIUS,
        'SPEED_FACTOR': SPEED_FACTOR,
        'STEERING_SMOOTHING': STEERING_SMOOTHING,
        'HEADING_SMOOTHING': HEADING_SMOOTHING,
        'WEIGHT_FACTORS': WEIGHT_FACTORS.copy(),
        'DBSCAN_EPS': 1.0,  # layout.py에서 기본값
        'DBSCAN_MIN_SAMPLES': 3  # layout.py에서 기본값
    }
}

# 스레드 안전성을 위한 락
SHARED_LOCK = threading.Lock()

# 서버 설정
SERVER_CONFIG = {
    'flask_host': '0.0.0.0',
    'flask_port': 5030,
    'dash_port': 8050
}

# 그래프 설정
GRAPH_CONFIG = {
    'max_points': 100,
    'max_speed': 80
}