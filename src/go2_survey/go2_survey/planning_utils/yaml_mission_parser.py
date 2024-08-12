import yaml
from .planning_utils import planning_utils

from std_msgs import Header
from geometry_msgs import Point, PolygonStamped, Point32, PoseStamped
from tf2_geometry_msgs import PointStamped
from nav_msgs.msg import Path

UTM_FRAME = "utm"
MAP_FRAME = "map"


class SurveyMissionPlanner:
    """
    Plans a survey mission based on params in a yaml file
    """

    def __init__(self, mission_file_path: str) -> None:
        with open(mission_file_path, "r") as mission_file:
            self._param_dict = yaml.safe_load(mission_file)["survey_mission"]

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
            self._alignment_dir,
        )
        path_msg = self._generate_path_msg(route)
        return path_msg

    def _get_map_vertices(self, tf_buffer):
        vertices = []
        for vertex in self.vertex_dict["vertices"]:
            easting, northing = vertex["easting"], vertex["northing"]
            pt = Point(x=easting, y=northing, z=0.0)
            vertices.append(pt)

        map_vertices = self._transform_vertices(tf_buffer, vertices)
        return map_vertices

    def _transform_vertices(self, tf_buffer, vertices):
        map_vertices = []
        for vertex in vertices:
            stamped_vertex = PointStamped()
            stamped_vertex.header = Header(frame_id=UTM_FRAME)
            stamped_vertex.point = vertex
            stamped_vertex_map = self.tf_buffer.transform(stamped_vertex, MAP_FRAME)
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

    @property
    def _grid_spacing(self):
        return self._param_dict["grid_spacing"]

    @property
    def _offset(self):
        return self._param_dict["offset"]

    @property
    def _alignment_dir(self):
        return self._param_dict["alignment_dir"]


def _point_to_point32(point):
    point32 = Point32()
    point32.x = point.x
    point32.y = point.y
    point32.z = point.z
    return point32


def _points_to_tuples(points):
    return [[pt.x, pt.y] for pt in points]
