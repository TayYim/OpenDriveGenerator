import numpy as np
from scipy.interpolate import CubicSpline
from lxml import etree
from numpy.ma import arange
from scipy.interpolate import splev, splprep
from shapely.geometry import LineString

# Constants
rounding_precision = 3
interpolation_distance = 0.002
smoothness = 0
min_num_nodes = 20


def init_xodr_tree(path):
    tree = etree.parse(path)
    return tree

def get_geometry_line_node(s, x, y, hdg, length):
    geometry = etree.Element("geometry", s=str(s), x=str(x), y=str(y), hdg=str(hdg), length=str(length))
    geometry.append(etree.Element("line"))
    return geometry

def interpolate(x, y):
    """
        Interpolate the road points using cubic splines and ensure we handle 4F tuples for compatibility
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
    _, der_vals = splev(unew, pos_tck, der=1)

    s = [0]
    x_start_list = []
    y_start_list = []
    phi_list = []
    length_list = []

    for i in range(len(new_x_vals)-1):
        x_start = new_x_vals[i]
        x_end = new_x_vals[i+1]
        y_start = new_y_vals[i]
        y_end = new_y_vals[i+1]
        x_start_list.append(x_start)
        y_start_list.append(y_start)
        
        # der = der_vals[i]
        der = (y_end - y_start) / (x_end-x_start)
        phi = np.arctan(der)
        # phi = np.pi/2 - phi
        if i==0:
            print(der, np.arctan(der), phi)
        phi_list.append(float(phi))
        
        # Get distance
        length = np.sqrt( (x_start-x_end)**2 + (y_start-y_end)**2 )
        s.append(s[i] + length)
        length_list.append(float(length))

    return (s, x_start_list, y_start_list, phi_list, length_list)


# def fit_curve_to_lines(x, y, xx):
#     cs_xy = CubicSpline(x, y)
#     s = [0]
#     x_start_list = []
#     y_start_list = []
#     phi_list = []
#     length_list = []

#     for i in range(len(xx)-1):
#         x_start = xx[i]
#         x_end = xx[i+1]
#         y_start = float(cs_xy(x_start))
#         y_end = float(cs_xy(x_end))
#         x_start_list.append(x_start)
#         y_start_list.append(y_start)
        
#         tan_phi = cs_xy.derivative(1)(x_start)
#         phi = np.arctan(tan_phi)
#         phi_list.append(float(phi))
        
#         # Get distance
#         length = np.sqrt( (x_start-x_end)**2 + (y_start-y_end)**2 )
#         s.append(s[i] + length)
#         length_list.append(float(length))

#     return (s, x_start_list, y_start_list, phi_list, length_list)

if __name__=="__main__":
    template_path = "template.xodr"
    output_path = "/home/tay/Applications/esmini/output.xodr"
    # output_path = "output.xodr"

    # Input x/y points
    x= [0, 30, 40, 50, 150, 30 ]
    y= [0, 20, 30, 40, 100, 180 ]
    # segment = 0.01
    # xx = np.arange(x[0], x[-1], segment)

    # s, x_start_list, y_start_list, phi_list, length_list = fit_curve_to_lines(x, y, xx)

    s, x_start_list, y_start_list, phi_list, length_list = interpolate(x, y)

    tree = init_xodr_tree(template_path)
    road = tree.find('road')
    planView = road.find('planView')

    # Set road length
    road.attrib["length"] = str(s[-1])

    # Append geometry nodes
    for i in range(len(x_start_list)):
        geometry_node = get_geometry_line_node(s[i], x_start_list[i], y_start_list[i], phi_list[i], length_list[i])
        planView.append(geometry_node)

    # Output file
    tree.write(output_path, pretty_print=True, method='xml', xml_declaration=True, encoding='utf-8')