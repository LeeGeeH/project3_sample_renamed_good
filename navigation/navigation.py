import math
from navigation.position_handler import PositionHandler
from navigation.pid_controller import PIDController
from navigation.purepursuit import PurePursuit
from navigation.obstacle.obstacle_handler import ObstacleHandler  # 수정됨

class Navigation:
    def __init__(self):
        self.position_handler = PositionHandler()
        self.controller = PIDController()
        self.pure_pursuit = PurePursuit()
        self.obstacle_handler = ObstacleHandler()  # 수정됨
        self.destination = None
        self.start_mode = "start"

    def init_simulation(self):
        """시뮬레이션 초기화."""
        self.position_handler = PositionHandler()
        self.controller = PIDController()
        self.pure_pursuit = PurePursuit()
        self.obstacle_handler = ObstacleHandler()
        self.destination = None
        self.start_mode = "start"
        return {"status": "OK", "message": "Simulation initialized"}

    def set_destination(self, destination_str):
        """목적지를 설정하고 초기 거리를 계산."""
        try:
            x, y, z = map(float, destination_str.split(","))
            self.destination = (x, z)
            self.controller.reset_integral()
            self.pure_pursuit.initial_distance = None

            if self.position_handler.current_position:
                curr_x, curr_z = self.position_handler.current_position
                self.pure_pursuit.initial_distance = math.sqrt((x - curr_x) ** 2 + (z - curr_z) ** 2)

            return {
                "status": "OK",
                "destination": {"x": x, "y": y, "z": z},
                "initial_distance": self.pure_pursuit.initial_distance
            }
        except Exception as e:
            return {"status": "ERROR", "message": str(e)}

    def update_obstacle(self, obstacle_data):
        """장애물 데이터 업데이트."""
        return self.obstacle_handler.update_obstacle(obstacle_data)

    def get_move(self):
        """장애물 회피 여부를 먼저 판단하고, Pure Pursuit로 이동 명령 계산."""
        if self.start_mode == "pause":
            return {"move": "STOP", "weight": 1.0}

        # --- 장애물 회피 우선 판단 ---
        avoidance_command = self.obstacle_handler.get_avoidance_command(
            self.position_handler.current_position,
            self.position_handler.current_heading
        )
        if avoidance_command:
            return avoidance_command

        # --- 장애물 없으면 순수 주행 ---
        command, new_position = self.pure_pursuit.compute_move(
            self.position_handler.current_position,
            self.position_handler.current_heading,
            self.position_handler.current_speed_kh,
            self.destination,
            self.controller,
            self.obstacle_handler
        )

        if new_position:
            self.position_handler.current_position = new_position

        return command
