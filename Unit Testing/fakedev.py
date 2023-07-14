import threading
from mavlink2resthelper import GPS_GLOBAL_ORIGIN_ID, Mavlink2RestHelper
import time
from enum import Enum
from loguru import logger
from typing import Any, Dict, List

DVL_DOWN = 1
DVL_FORWARD = 2


class MessageType(str, Enum):
    POSITION_DELTA = "POSITION_DELTA"
    POSITION_ESTIMATE = "POSITION_ESTIMATE"
    SPEED_ESTIMATE = "SPEED_ESTIMATE"

    @staticmethod
    def contains(value):
        return value in set(item.value for item in MessageType)


class DvlDriver(threading.Thread):
    status = "Starting"
    version = ""
    mav = Mavlink2RestHelper()
    rangefinder = True
    reset_counter = 0

    def __init__(self) -> None:
        threading.Thread.__init__(self)
        self.should_send = MessageType.POSITION_DELTA
        self.current_orientation = DVL_DOWN
        self.vel_data = """{
              "time": 106.3935775756836,
              "vx": -3.713480691658333e-05,
              "vy": 5.703703573090024e-05,
              "vz": 2.4990416932269e-05,
              "fom": 0.00016016385052353144,
              "altitude": 0.4949815273284912,
              "velocity_valid": true,
              "status": 0,
              "format": "json_v3.1",
              "type": "velocity",
              "time_of_validity": 1638191471563017,
              "time_of_transmission": 1638191471752336
            }"""
        self.pos_data = """{
              "ts": 49056.809,
              "x": 12.43563613697886467,
              "y": 64.617631152402609587,
              "z": 1.767641898933798075,
              "std": 0.001959984190762043,
              "roll": 0.6173566579818726,
              "pitch": 0.6173566579818726,
              "yaw": 0.6173566579818726,
              "type": "position_local",
              "status": 0,
              "format": "json_v3.1"
            }
        """

    def setup_mavlink(self) -> None:
        """
        Sets up mavlink streamrates so we have the needed messages at the
        appropriate rates
        """
        # self.report_status("Setting up MAVLink streams...")
        self.mav.ensure_message_frequency("ATTITUDE", 30, 5)

    def setup_params(self) -> None:
        """
        Sets up the required params for DVL integration
        """
        self.mav.set_param("AHRS_EKF_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        # TODO: Check if really required. It doesn't look like the ekf2 stops at all
        self.mav.set_param("EK2_ENABLE", "MAV_PARAM_TYPE_UINT8", 0)

        self.mav.set_param("EK3_ENABLE", "MAV_PARAM_TYPE_UINT8", 1)
        self.mav.set_param("VISO_TYPE", "MAV_PARAM_TYPE_UINT8", 1)
        self.mav.set_param("EK3_GPS_TYPE", "MAV_PARAM_TYPE_UINT8", 3)
        self.mav.set_param("EK3_SRC1_POSXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
        self.mav.set_param("EK3_SRC1_VELXY", "MAV_PARAM_TYPE_UINT8", 6)  # EXTNAV
        self.mav.set_param("EK3_SRC1_POSZ", "MAV_PARAM_TYPE_UINT8", 1)  # BARO
        # 测距仪
        if self.rangefinder:
            self.mav.set_param("RNGFND1_TYPE", "MAV_PARAM_TYPE_UINT8", 10)  # MAVLINK

    def handle_velocity(self, data: Dict[str, Any]) -> None:
        # extract velocity data from the DVL JSON
        vx, vy, vz, alt, valid, fom = (
            data["vx"],
            data["vy"],
            data["vz"],
            data["altitude"],
            data["velocity_valid"],
            data["fom"],
        )
        dt = data["time"] / 1000
        dx = dt * vx
        dy = dt * vy
        dz = dt * vz

        # fom is the standard deviation. scaling it to a confidence from 0-100%
        # 0 is a very good measurement, 0.4 is considered a inaccurate measurement
        _fom_max = 0.4
        confidence = 100 * (1 - min(_fom_max, fom) / _fom_max) if valid else 0
        # confidence = 100 if valid else 0

        if not valid:
            logger.info("Invalid  dvl reading, ignoring it.")
            return

        if self.rangefinder and alt > 0.05:
            self.mav.send_rangefinder(alt)

        # 位置增量
        if self.should_send == MessageType.POSITION_DELTA:
            dRoll, dPitch, dYaw = [
                current_angle - last_angle
                for (current_angle, last_angle) in zip(self.current_attitude, self.last_attitude)
            ]
            if self.current_orientation == DVL_DOWN:
                position_delta = [dx, dy, dz]
                attitude_delta = [dRoll, dPitch, dYaw]
            elif self.current_orientation == DVL_FORWARD:
                position_delta = [dz, dy, -dx]
                attitude_delta = [dYaw, dPitch, -dRoll]
            self.mav.send_vision(position_delta, attitude_delta, dt=data["time"] * 1e3, confidence=confidence)
        # 速度增量 底层使用的是VISION_SPEED_ESTIMATE
        elif self.should_send == MessageType.SPEED_ESTIMATE:
            velocity = [vx, vy, vz] if self.current_orientation == DVL_DOWN else [vz, vy, -vx]  # DVL_FORWARD
            self.mav.send_vision_speed_estimate(velocity)
        self.last_attitude = self.current_attitude

    def handle_position_local(self, data):
        self.current_attitude = data["roll"], data["pitch"], data["yaw"]
        if self.should_send == MessageType.POSITION_ESTIMATE:
            x, y, z = data["x"], data["y"], data["z"]
            self.timestamp = data["ts"]
            self.mav.send_vision_position_estimate(
                self.timestamp, [x, y, z], self.current_attitude, reset_counter=self.reset_counter
            )

    def run(self) -> None:
        self.setup_mavlink()
        self.setup_params()
        print("设置完成")
        while True:
            time.sleep(0.2)
            # self.handle_velocity(self.vel_data)
            # self.handle_position_local(self.pos_data)

