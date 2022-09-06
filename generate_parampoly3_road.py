# Songyang Yan 2022.9.5

import numpy as np
from lxml import etree
from numpy.ma import arange
from scipy.interpolate import splev, splprep
from shapely.geometry import LineString
from scipy import linalg
import validation

# Constants
rounding_precision = 10
interpolation_distance = 0.01
smoothness = 0
min_num_nodes = 20


def get_u(x, y, x0, y0, hdg):
    """

    Args:
        x (float): current x
        y (float): current y
        x0 (float): x of the start point of this segment 
        y0 (float): y of the start point of this segment 
        hdg (float): hdg of the (x0, y0)

    Returns:
        float: u
    """
    phi = hdg
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    tan_phi = np.tan(phi)
    return (x + y*tan_phi - x0 - y0*tan_phi)/(cos_phi + sin_phi*tan_phi)

def get_v(x, y, x0, y0, hdg):
    """

    Args:
        x (float): current x
        y (float): current y
        x0 (float): x of the start point of this segment 
        y0 (float): y of the start point of this segment 
        hdg (float): hdg of the (x0, y0)

    Returns:
        float: v
    """
    phi = hdg
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    tan_phi = np.tan(phi)
    return (x0*tan_phi - y0 + y - x*tan_phi)/(sin_phi*tan_phi + cos_phi)

def get_param_dict(x0, y0, xs, ys, hdg, ps):
    """Get aU (0), bU, cU, dU, aV (0), bV, cV, dV 

    Args:
        x0 (float): x of the start point of this segment 
        y0 (float): y of the start point of this segment 
        xs (list[float]): 3 points' x. Length must be 3
        ys (list[float]): 3 points' y. Length must be 3
        hdg (float): hdg of the (x0, y0)
        ps (list[float]): distance between each point in (xs, ys) and (x0, y0)

    Returns:
        tuple: us, vs
    """
    u0 = get_u(xs[0], ys[0], x0, y0, hdg)
    u1 = get_u(xs[1], ys[1], x0, y0, hdg)
    u2 = get_u(xs[2], ys[2], x0, y0, hdg)
    
#     print(u0, u1, u2)
    
    a = np.array([[ps[0],ps[0]**2,ps[0]**3],
                  [ps[1],ps[1]**2,ps[1]**3],
                  [ps[2],ps[2]**2,ps[2]**3]])
    b = np.array([u0,u1,u2])
    us = linalg.solve(a,b)
    us = np.insert(us, 0, 0, axis=0)
    
    
    v0 = get_v(xs[0], ys[0], x0, y0, hdg)
    v1 = get_v(xs[1], ys[1], x0, y0, hdg)
    v2 = get_v(xs[2], ys[2], x0, y0, hdg)
    
#     print(v0, v1, v2)
    
    a = np.array([[ps[0],ps[0]**2,ps[0]**3],
                  [ps[1],ps[1]**2,ps[1]**3],
                  [ps[2],ps[2]**2,ps[2]**3]])
    b = np.array([v0,v1,v2])
    vs = linalg.solve(a,b)
    vs = np.insert(vs, 0, 0, axis=0)
    return us, vs

def interpolate(x, y):
    """Interpolate the road points using cubic splines and ensure we handle 4F tuples for compatibility.
       Copy from SBST tool-competition repo

    Args:
        x (list): list of x
        y (list): list of y

    Returns:
        tuple: new x and new y
    """
    old_x_vals = list(x)
    old_y_vals = list(y)

    # This is an approximation based on whatever input is given
    test_road_lenght = LineString([(t[0], t[1]) for t in zip(old_x_vals, old_y_vals)]).length
    num_nodes = int(test_road_lenght / interpolation_distance)
    if num_nodes < min_num_nodes:
        num_nodes = min_num_nodes

    assert len(old_x_vals) >= 2, "You need at leas two road points to define a road"
    assert len(old_y_vals) >= 2, "You need at leas two road points to define a road"

    if len(old_x_vals) == 2:
        # With two points the only option is a straight segment
        k = 1
    elif len(old_x_vals) == 3:
        # With three points we use an arc, using linear interpolation will result in invalid road tests
        k = 2
    else:
        # Otheriwse, use cubic splines
        k = 3

    pos_tck, pos_u = splprep([old_x_vals, old_y_vals], s= smoothness, k=k)

    step_size = 1 / num_nodes
    unew = arange(0, 1 + step_size, step_size)

    new_x_vals, new_y_vals = splev(unew, pos_tck)
    
    
    return new_x_vals, new_y_vals

def init_xodr_tree(path):
    tree = etree.parse(path)
    return tree

def get_geometry_parampoly3_node(s, x, y, hdg, length, us, vs):
    geometry = etree.Element("geometry", s=str(s), x=str(x), y=str(y), hdg=str(hdg), length=str(length))
    paramPoly3 = etree.Element("paramPoly3", aU=str(us[0]), bU=str(us[1]), cU=str(us[2]), dU=str(us[3]),
                                             aV=str(vs[0]), bV=str(vs[1]), cV=str(vs[2]), dV=str(vs[3]),
                                            pRange="arcLength")
    geometry.append(paramPoly3)
    return geometry

if __name__=="__main__":
    template_path = "template.xodr"
    output_path = "/home/tay/Applications/esmini/output.xodr"
    # output_path = "output.xodr"

    # Input x/y points
    # limits 55-56 46.7-47.2
    # 155.8 ok 159.34
    old_xs= [0, 30, 40, 50, 150, 155 ]
    old_ys= [0, 20, 30, 40, 100, 180 ]
    # old_xs= [10, 30, 40, 50, 150, 30 ]
    # old_ys= [20, 20, 30, 40, 100, 180 ]

    xs, ys = interpolate(old_xs, old_ys)
    xs = np.array(xs)
    ys = np.array(ys)

    # get length list
    length_list = np.sqrt(np.diff(ys)**2 + np.diff(xs)**2)
    length_list = np.insert(length_list, 0, 0, axis=0)
    s_list = np.cumsum(length_list)

    # get hdg list
    diff_xs = np.diff(xs)
    diff_xs = np.where(diff_xs==0, diff_xs, 1e-15)

    ks = np.diff(ys)/diff_xs
    ks = np.insert(ks, 0, ks[0], axis=0)

    hdg_list = np.arctan(ks)

    # Tree

    tree = init_xodr_tree(template_path)
    road = tree.find('road')
    planView = road.find('planView')

    # Set road length
    road.attrib["length"] = str(s_list[-1])

    # Generate every segment
    segment_length = 20
    n_window_size = int(segment_length/interpolation_distance)

    n_total = len(xs)

    # A random choice
    index_point_1 = 2
    index_point_2 = int(n_window_size//2)
    # index_point_2 = 4

    for segment_id in range(int(n_total // n_window_size)):
        index_start = segment_id*n_window_size
        if segment_id == n_total // n_window_size - 1:
            index_end = n_total - 1 # the last segment
        else:
            index_end = (segment_id+1)*n_window_size - 1
            
        # calculate params
        
        index_points = [index_start + index_point_1, index_start + index_point_2, index_end]
        xs_seg = np.take(xs, index_points)
        ys_seg = np.take(ys, index_points)
        ps_seg = np.take(s_list, index_points) - s_list[index_start] # important!
        
        hdg = hdg_list[index_start]
        
        us, vs = get_param_dict(xs[index_start],ys[index_start], xs_seg, ys_seg, hdg , ps_seg)
        
        
        this_length = s_list[index_end] - s_list[index_start]

        # Append geometry nodes
        geometry_node = get_geometry_parampoly3_node(s_list[index_start], xs[index_start],ys[index_start], hdg, this_length, us, vs)
        planView.append(geometry_node)

    # Output file
    tree.write(output_path, pretty_print=True, method='xml', xml_declaration=True, encoding='utf-8')

    # validation
    TSHD_RADIUS = 47
    mr = validation.min_radius(list(zip(xs, ys)), int(5/interpolation_distance))
    if TSHD_RADIUS > mr > 0.0:
        check = True
    else:
        check = False
    print("min radius:{}".format(mr))
    print("is too sharp:{}".format(check))