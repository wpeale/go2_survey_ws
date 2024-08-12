import csv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import SetBool
from sensor_msgs.msg import NavSatFix

from go2_interfaces.msg import Go2State

LOG_RATE = 1.0
SAVE_RATE = 10.0


class FluxSurveyLoggerNode(Node):
    def __init__(self):
        super().__init__("flux_survey_logger_node")
        self._is_recording = False
        self._data = []
        self._srv = self.create_service(SetBool, "record", self._record_callback)

        self._start_time = None
        self._go2_mode = None
        self._co2_ppm_value = None
        self._lat_value = None
        self._lon_value = None
        self._alt_value = None

        self._record_timer = self.create_timer(LOG_RATE, self._log_callback)
        self._save_timer = self.create_timer(SAVE_RATE, self._save_callback)

        self.create_subscription(Float32, "co2_ppm", self._k30_callback, 10)
        self.create_subscription(NavSatFix, "gps/fix", self._gps_callback, 10)
        self.create_subscription(Go2State, "robot0/go2_states", self._state_callback, 10)

    def _log_callback(self):
        if self._is_data_ready() and self._is_recording:
            current_time = self.get_clock().now().nanoseconds / 1000000000.0
            time_since_start = current_time - self._start_time
            self._data.append(
                [
                    time_since_start,
                    self._go2_mode,
                    self._lat_value,
                    self._lon_value,
                    self._alt_value,
                    self._co2_ppm_value,
                ]
            )

    def _save_callback(self):
        if self._is_recording:
            self._save_to_csv("sensor_data_checkpoint.csv")

    def _k30_callback(self, msg):
        self._co2_ppm_value = msg.data

    def _gps_callback(self, msg):
        self._lat_value = msg.latitude
        self._lon_value = msg.longitude
        self._alt_value = msg.altitude

    def _state_callback(self, msg):
        self._go2_mode = msg.mode
        
    def _record_callback(self, request, response):
        if request.data:
            self._start_recording()
            response.success = True
            response.message = "Started recording"
        else:
            self._stop_recording()
            response.success = True
            response.message = "Stopped recording"
        return response

    def _start_recording(self):
        self._data.clear()
        self._is_recording = True
        self._start_time = self.get_clock().now().nanoseconds / 1000000000.0
        self.get_logger().info("Started recording")

    def _stop_recording(self):
        self._is_recording = False
        self.get_logger().info("Stopped recording")
        self._save_to_csv("sensor_data.csv")

    def _save_to_csv(self, filename):
        with open(filename, mode="w") as file:
            writer = csv.writer(file)
            for row in self._data:
                writer.writerow(row)
        self.get_logger().info(f"Data saved to {filename}")

    def _is_data_ready(self):
        return (
            self._co2_ppm_value is not None
            and self._go2_mode is not None
            and self._start_time is not None
            and self._lat_value is not None
            and self._lon_value is not None
            and self._alt_value is not None
        )
    


def main(args=None):
    rclpy.init(args=args)
    logger_node = FluxSurveyLoggerNode()
    rclpy.spin(logger_node)
    logger_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
