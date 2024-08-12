import os
import time

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Path

from std_srvs.srv import Empty, SetBool

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from nav2_simple_commander.robot_navigator import BasicNavigator

from go2_interfaces.msg import Go2State

from go2_survey.planning_utils import survey_mission_planner
from go2_survey_interfaces.action import Survey

UTM_FRAME = "utm"
MAP_FRAME = "map"
VEHICLE_FRAME = "robot0/base_link"

SURVEY_UPDATE_RATE = 1.0

DWELL_TIME = 30.0


class CO2SurveyNode(Node):
    def __init__(self, mission_file_path):
        super().__init__("co2_survey_node")

        self._mission_planner = survey_mission_planner.SurveyMissionPlanner(
            mission_file_path
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._navigator = BasicNavigator("basic_navigator")

        self._survey_poly = None
        self._survey_path = None
        self._poly_publisher = self.create_publisher(PolygonStamped, "survey_area", 10)
        self._path_publisher = self.create_publisher(Path, "survey_path", 10)

        self._go2_mode = None
        self.create_subscription(Go2State, "robot0/go2_states", self._state_callback, 10)
        
        self._action_client = ActionServer(self, Survey, 'survey', self._survey_callback, goal_callback=self._goal_callback, cancel_callback=self._cancel_callback)

        self._sit_client = self.create_client(Empty, 'robot0/sit')
        self._stand_client = self.create_client(Empty, 'robot0/stand')

    def _state_callback(self, msg):
        self._go2_mode = msg.mode

    def _goal_callback(self, goal_request):
        self.get_logger().info('Received goal request.')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, cancel_request):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

    def _survey_callback(self, goal_handle):
        path_msg = self._mission_planner.get_survey_path(
            self._tf_buffer, self._current_position
        )
        self._path_publisher.publish(path_msg)
        
        #self._sit_client.call_async(Empty.Request())
        #time.sleep(2)
        #self._stand_client.call_async(Empty.Request())
        #time.sleep(2)

        for pose in path_msg.poses[1:]:            
            if goal_handle.is_cancel_requested:
                break

            self._navigator.goToPose(pose)
            while not self._navigator.isTaskComplete() and not goal_handle.is_cancel_requested:
                self._tick()

            self.get_logger().info('RECORDING MEASUREMENT')

            time.sleep(3)
            self._sit_client.call_async(Empty.Request())
            self.get_logger().info('Sent sit cmd')
                
            
            dwell_start = time.time()
            while time.time() - dwell_start < DWELL_TIME and not goal_handle.is_cancel_requested:
                self._tick()

            self._stand_client.call_async(Empty.Request())
            self.get_logger().info('Sent stand cmd')
            time.sleep(3)
                
        if goal_handle.is_cancel_requested:
            self._navigator.cancelTask()
            goal_handle.canceled()
            print("canceled")
        else:
            print("wps completed successfully")

        return Survey.Result()

    def _tick(self):
        self._publish_area_poly()
        time.sleep(0.1)

    def _wait_for_map_trans(self):
        success = self._tf_buffer.can_transform(
            MAP_FRAME, UTM_FRAME, rclpy.time.Time()
        ) and self._tf_buffer.can_transform(MAP_FRAME, VEHICLE_FRAME, rclpy.time.Time())

        print(success)
        print(self._tf_buffer.all_frames_as_string())
        return success

    def _publish_area_poly(self):        
        poly_msg = self._mission_planner.get_survey_area_polygon(self._tf_buffer)
        self._poly_publisher.publish(poly_msg)


    @property
    def _current_position(self):
        trans = self._tf_buffer.lookup_transform(
            MAP_FRAME, VEHICLE_FRAME, rclpy.time.Time()
        )
        return [trans.transform.translation.x, trans.transform.translation.y]

    @property
    def _is_prone(self):
        return self._go2_mode is not None and self._go2_mode == 7


def main(args=None):
    rclpy.init(args=args)

    default_yaml_file_path = os.path.join(
        get_package_share_directory("go2_survey"), "config", "survey_area.yaml"
    )

    node = CO2SurveyNode(default_yaml_file_path)
    rclpy.spin(node)
    node.start_survey()
    

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
