import numpy as np
import shapely.geometry as sg


def generate_survey_path(
    survey_area, robot_position, grid_spacing=1.0, offset=0.0, north_aligned=True
):
    survey_points = _generate_survey_points(
        survey_area, grid_spacing, offset, north_aligned
    )
    start_point = _get_start_point(survey_points, robot_position, north_aligned)
    current_point = start_point
    survey_points.remove(start_point)
    route = [robot_position, start_point]

    if north_aligned:
        alignment_metric = lambda a, b: abs(b[0] - a[0])
    else:
        alignment_metric = lambda a, b: abs(b[1] - a[1])

    if north_aligned:
        row_dist = (
            lambda a, b: -np.sign(a[1] - route[-2][1]) * (b[1] - a[1])
            if a[0] != b[0]
            else abs(b[1] - a[1])
        )
    else:
        row_dist = (
            lambda a, b: -np.sign(a[0] - route[-2][0]) * (b[0] - a[0])
            if a[1] != b[1]
            else abs(b[0] - a[0])
        )

    def next_criterion(point):
        return [alignment_metric(current_point, point), row_dist(current_point, point)]

    while survey_points:
        survey_points.sort(key=next_criterion)
        next_point = survey_points.pop(0)
        route.append(next_point)
        current_point = next_point

    return route


def _generate_survey_points(survey_area, grid_spacing, offset, north_aligned):
    area_poly = sg.Polygon(survey_area)
    minx, miny, maxx, maxy = area_poly.bounds

    if north_aligned:
        x = np.arange(minx + offset, maxx, grid_spacing)
        y = np.arange(miny, maxy, grid_spacing)
    else:
        x = np.arange(minx, maxx, grid_spacing)
        y = np.arange(miny + offset, maxy, grid_spacing)

    X, Y = np.meshgrid(x, y)
    grid = np.array([X.ravel(), Y.ravel()]).T
    pruned_pts = [pt for pt in grid if area_poly.contains(sg.Point(*pt))]

    return pruned_pts


def _get_start_point(survey_points, robot_position, north_aligned):
    if north_aligned:
        row_coord = lambda pt: pt[0]
        col_coord = lambda pt: pt[1]
    else:
        row_coord = lambda pt: pt[1]
        col_coord = lambda pt: pt[0]

    rows = [row_coord(pt) for pt in survey_points]
    max_row = [pt for pt in survey_points if row_coord(pt) == max(rows)]
    min_row = [pt for pt in survey_points if row_coord(pt) == min(rows)]

    possible_start_pts = []
    possible_start_pts.append(max(max_row, key=col_coord))
    possible_start_pts.append(max(min_row, key=col_coord))
    possible_start_pts.append(min(max_row, key=col_coord))
    possible_start_pts.append(min(min_row, key=col_coord))

    return min(possible_start_pts, key=lambda pt: _distance(pt, robot_position))


def _distance(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
