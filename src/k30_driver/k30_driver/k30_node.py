import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import argparse

from . import k30

SENSOR_UPDATE_RATE = 2.0

class K30Node(Node):
    def __init__(self):
        super().__init__('k30_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._publisher = self.create_publisher(Float32, 'co2_ppm', 10)
        self._update_timer = self.create_timer(SENSOR_UPDATE_RATE, self._update_callback)
        self._sensor = k30.K30Sensor(serial_port=self._port)

    def _update_callback(self):
        sensor_value = self._sensor.read_co2_ppm()
        msg = Float32()
        msg.data = sensor_value
        self._publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    sensor_node = K30Node()
    rclpy.spin(sensor_node)
    sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
