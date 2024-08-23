import rclpy
import argparse
import signal
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import SetBool

from go2_survey_interfaces.action import Survey

class SurveyClient(Node):
    def __init__(self, mission_file_path):
        super().__init__('go2_survey_client')
        with open(mission_file_path, "r") as mission_file:
            self._param_dict = yaml.safe_load(mission_file)["survey_mission"]
        self._action_client = ActionClient(self, Survey, 'survey')
        self._record_client = self.create_client(SetBool, "record")
        self._goal_handle = None  # To store the goal handle

    def start_recording(self):
        self.get_logger().info('Starting recording')
        req = SetBool.Request()
        req.data = True
        self._record_client.call_async(req)
        
    def send_goal(self):
        goal_msg = Survey.Goal()
        goal_msg.eastings = [float(pt["easting"]) for pt in self._param_dict["vertices"]]
        goal_msg.northings = [float(pt["northing"]) for pt in self._param_dict["vertices"]]
        goal_msg.grid_spacing = self._grid_spacing
        goal_msg.offset = self._offset
        goal_msg.north_aligned = self._north_aligned
        goal_msg.sample_time = self._sample_time
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Survey goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Survey goal accepted')

    def cancel_goal(self):
        if self._goal_handle:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        self.get_logger().info('Goal cancelled')
        rclpy.shutdown()

    @property
    def _grid_spacing(self):
        return self._param_dict["grid_spacing"]

    @property
    def _offset(self):
        return self._param_dict["offset"]

    @property
    def _north_aligned(self):
        return self._param_dict["north_aligned"]

    @property
    def _sample_time(self):
        return self._param_dict["sample_time"]



def main(args=None):
    parser = argparse.ArgumentParser(description='Begin a survey mission as described in a YAML mission file.')
    parser.add_argument('yaml_path', type=str, help='Path to the YAML mission plan file.')
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    node = SurveyClient(parsed_args.yaml_path)

    node.start_recording()
    node.send_goal()

    def signal_handler(sig, frame):
        node.cancel_goal()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
