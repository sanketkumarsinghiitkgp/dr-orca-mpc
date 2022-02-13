import cvxpy as cp
import numpy as np
from sympy import *
from sympy.geometry import *

eps = 1e-6
inf = 1e30
def pointToColumnArray(point):
    return np.array([[N(point.x),N(point.y)]], dtype=np.float64).T


def columnArrayToPoint(_point):
    return Point(_point[0,0], _point[1,0])


def findRegion(_center, radius, _point, _tangent_point_1, _tangent_point_2):
    # Region 1 - no intersection with minor sector boundary (outside VO)
    # Region 2 - 1 point of intersection with minor sector boundary + circle composite (projection on VO boundary is the projection on the circle)
    # Region 3 - >=2 points of intersection with minor sector boundary+circle composite (projection on VO boundary is the projection i)
    center = columnArrayToPoint(_center)
    point = columnArrayToPoint(_point)
    tangent_point_1 = columnArrayToPoint(_tangent_point_1)
    tangent_point_2 = columnArrayToPoint(_tangent_point_2)
    r = Segment(Point(0,0), point)
    rad_1 = Segment(center, tangent_point_1)
    rad_2 = Segment(center, tangent_point_2)
    circle = Circle(center, radius)
    s_c = intersection(r, circle)
    s_1 = intersection(r, rad_1)
    s_2 = intersection(r, rad_2)
    cnt = len(s_c)+len(s_1)+len(s_2)
    if cnt == 0:
        return 1
    elif cnt == 1:
        return 2
    elif cnt == 2 or cnt == 3:
        return 3
    else:
        assert(false)
        return -1


def projectOnCircle(center, radius, point):
    #can be implemented using library too
    ray_vec_norm = np.linalg.norm(center-point)
    if(ray_vec_norm<eps):
        pass #TODO handle this case
    ray_vec = point-center
    ray_unit_vec = ray_vec/ray_vec_norm
    return center+ray_unit_vec*radius


def projectOnLine(point_on_line, point):
    r = Ray(columnArrayToPoint(point_on_line), columnArrayToPoint(2*point_on_line))
    projected_point = r.projection(columnArrayToPoint(point))
    if r.contains(projected_point):
        print(N(projected_point))
        return pointToColumnArray(projected_point)
    else:
        return None


def findTangentPoints(_center, radius):
    center = columnArrayToPoint(_center)
    circle = Circle(center, radius)
    line_1, line_2 = circle.tangent_lines(Point(0,0))
    return pointToColumnArray(line_1.p2), pointToColumnArray(line_2.p2)


def projectOnVO(center, radius, point):
    projection_on_circle = projectOnCircle(center, radius, point)
    if np.linalg.norm(point-np.zeros((2,1)))<eps:
        return projection_on_circle
    tangent_point_1, tangent_point_2 = findTangentPoints(center, radius)
    projection_on_line_1 = projectOnLine(tangent_point_1, point)
    projection_on_line_2 = projectOnLine(tangent_point_2, point)
    region = findRegion(center, radius, point, tangent_point_1, tangent_point_2)
    print(region)
    dist_c = np.linalg.norm(point - projection_on_circle)
    dist_1 = np.linalg.norm(point - projection_on_line_1) if np.any(projection_on_line_1!=None) else inf
    dist_2 = np.linalg.norm(point - projection_on_line_2) if np.any(projection_on_line_2!=None) else inf
    if region == 1:
        min_dist = min(dist_c, dist_1, dist_2)
        if min_dist == dist_c:
            return projection_on_circle
        elif min_dist == dist_1:
            return projection_on_line_1
        elif min_dist == dist_2:
            return projection_on_line_2
        else:
            assert(false)
    elif region == 2:
        return projection_on_circle
    elif region == 3:
        min_dist = min(dist_1, dist_2)
        if min_dist == dist_1:
            return projection_on_line_1
        elif min_dist == dist_2:
            return projection_on_line_2
        else:
            assert(false)
    else:
        assert(false)
    
    return projected_point

