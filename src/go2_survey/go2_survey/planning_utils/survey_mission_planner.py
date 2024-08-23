import yaml
from go2_survey.planning_utils import planning_utils

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PolygonStamped, Point32, PoseStamped
from tf2_geometry_msgs import PointStamped
from nav_msgs.msg import Path

UTM_FRAME = "utm"
MAP_FRAME = "map"


class SurveyMissionPlanner:
    """
    Plans a survey mission based on provided parameters.
    """

    def __init__(self, vertices, grid_spacing=1.0, offset=0.0, north_aligned=True) -> None:
        self._vertices = vertices
        self._grid_spacing = grid_spacing
        self._offset = offset
        self._north_aligned = north_aligned
        
    def get_survey_area_polygon(self, tf_buffer):
        map_vertices = self._get_map_vertices(tf_buffer)
        poly_stamped = PolygonStamped()
        poly_stamped.header = Header(frame_id=MAP_FRAME)
        for vertex in map_vertices:
            point32 = _point_to_point32(vertex)
            poly_stamped.polygon.points.append(point32)
        return poly_stamped

    def get_survey_path(self, tf_buffer, current_position):
        map_vertices = self._get_map_vertices(tf_buffer)
        route = planning_utils.generate_survey_path(
            _points_to_tuples(map_vertices),
            current_position,
            self._grid_spacing,
            self._offset,
            self._north_aligned,
        )
        path_msg = self._generate_path_msg(route)
        return path_msg

    def _get_map_vertices(self, tf_buffer):
        map_vertices = self._transform_vertices(tf_buffer, self._vertices)
        return map_vertices

    def _transform_vertices(self, tf_buffer, vertices):
        map_vertices = []
        for vertex in vertices:
            stamped_vertex = PointStamped()
            stamped_vertex.header = Header(frame_id=UTM_FRAME)
            stamped_vertex.point = vertex
            stamped_vertex_map = tf_buffer.transform(stamped_vertex, MAP_FRAME)
            map_vertices.append(stamped_vertex_map.point)

        return map_vertices

    def _generate_path_msg(self, route):
        # define Path message
        path_msg = Path()

        # Create some PoseStamped messages
        for pt in route:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        # Set the frame and stamp, then publish
        path_msg.header.frame_id = "map"
        return path_msg

def _point_to_point32(point):
    point32 = Point32()
    point32.x = point.x
    point32.y = point.y
    point32.z = point.z
    return point32


def _points_to_tuples(points):
    return [[pt.x, pt.y] for pt in points]
