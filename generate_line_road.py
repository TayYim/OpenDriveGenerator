import numpy as np
from scipy.interpolate import CubicSpline
from lxml import etree


def init_xodr_tree(path):
    tree = etree.parse(path)
    return tree

def get_geometry_line_node(s, x, y, hdg, length):
    geometry = etree.Element("geometry", s=str(s), x=str(x), y=str(y), hdg=str(hdg), length=str(length))
    geometry.append(etree.Element("line"))
    return geometry

def fit_curve_to_lines(x, y, xx):
    cs_xy = CubicSpline(x, y)
    s = [0]
    x_start_list = []
    y_start_list = []
    phi_list = []
    length_list = []

    for i in range(len(xx)-1):
        x_start = xx[i]
        x_end = xx[i+1]
        y_start = float(cs_xy(x_start))
        y_end = float(cs_xy(x_end))
        x_start_list.append(x_start)
        y_start_list.append(y_start)
        
        tan_phi = cs_xy.derivative(1)(x_start)
        phi = np.arctan(tan_phi)
        phi_list.append(float(phi))
        
        # Get distance
        length = np.sqrt( (x_start-x_end)**2 + (y_start-y_end)**2 )
        s.append(s[i] + length)
        length_list.append(float(length))

    return (s, x_start_list, y_start_list, phi_list, length_list)

if __name__=="__main__":
    template_path = "template.xodr"
    output_path = "output.xodr"

    # Input x/y points
    x= np.array([0, 1, 2, 3, 4, 5, 6, 7])*10
    y= np.array([3, 4, 3.5, 2, 1, 1.5, 1.25, 0.9])*10
    segment = 1
    xx = np.arange(x[0], x[-1], segment)

    s, x_start_list, y_start_list, phi_list, length_list = fit_curve_to_lines(x, y, xx)

    tree = init_xodr_tree(template_path)
    road = tree.find('road')
    planView = road.find('planView')

    # Set road length
    road.attrib["length"] = str(s[-1])

    # Append geometry nodes
    for i in range(len(xx) - 1):
        geometry_node = get_geometry_line_node(s[i], x_start_list[i], y_start_list[i], phi_list[i], length_list[i])
        planView.append(geometry_node)

    # Output file
    tree.write(output_path, pretty_print=True, method='xml', xml_declaration=True, encoding='utf-8')