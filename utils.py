import numpy as np
def find_line_intersections(bin_pos, initial_position, bin_dimensions):
    """
    Finds the intersection points of the line defined by bin_pos and initial_position
    with the boundaries of the bin.

    Args:
        bin_pos (list): Position of the bin center.
        initial_position (list): Initial position of the line.
        bin_dimensions (dict): Dictionary defining the bin dimensions with 'bottom_left' and 'top_right'.

    Returns:
        list: Intersection points of the line with the bin boundaries.
    """
    line_dir = np.array(bin_pos) - np.array(initial_position)  # Line direction vector
    line_dir /= np.linalg.norm(line_dir)  # Normalize the direction vector
    line_point = np.array(initial_position)

    intersections = []
    # Define bin boundary planes
    planes = [
        {'normal': [1, 0, 0], 'point': [bin_dimensions['bottom_left'][0], 0, 0]},  # Left plane
        {'normal': [-1, 0, 0], 'point': [bin_dimensions['top_right'][0], 0, 0]},  # Right plane
        {'normal': [0, 1, 0], 'point': [0, bin_dimensions['bottom_left'][1], 0]},  # Front plane
        {'normal': [0, -1, 0], 'point': [0, bin_dimensions['top_right'][1], 0]},  # Back plane
    ]

    # Loop through each plane and compute intersection
    for plane in planes:
        normal = np.array(plane['normal'])
        plane_point = np.array(plane['point'])

        # Compute t (intersection parameter) for the line-plane intersection
        denominator = np.dot(normal, line_dir)
        if np.abs(denominator) > 1e-6:  # Check if the line is not parallel to the plane
            t = np.dot(normal, (plane_point - line_point)) / denominator
            intersection_point = line_point + t * line_dir

            # Check if the intersection is within the bin bounds
            if (bin_dimensions['bottom_left'][0] <= intersection_point[0] <= bin_dimensions['top_right'][0] and
                bin_dimensions['bottom_left'][1] <= intersection_point[1] <= bin_dimensions['top_right'][1] and
                bin_dimensions['bottom_left'][2] <= intersection_point[2] <= bin_dimensions['top_right'][2]):
                intersections.append(intersection_point)

    return intersections

def velocity_to_rotation_matrix(velocity_vector):
   """
   Construct a rotation matrix where the y-axis aligns with the given velocity vector.


   Args:
       velocity_vector (np.ndarray): Velocity vector in world frame.


   Returns:
       np.ndarray: 3x3 rotation matrix.
   """
   # Normalize the velocity vector to get the y-axis
   y_axis = velocity_vector / np.linalg.norm(velocity_vector)


   # Define a reference z-axis (world z-axis)
   z_axis_world = np.array([0, 0, 1])


   # Compute the x-axis using the cross product of z and y
   x_axis = np.cross(z_axis_world, y_axis)


   # Handle the case where the velocity vector is parallel to the z-axis
   if np.linalg.norm(x_axis) < 1e-6:
       z_axis_world = np.array([1, 0, 0])  # Choose x-axis as reference
       x_axis = np.cross(z_axis_world, y_axis)


   x_axis /= np.linalg.norm(x_axis)  # Normalize the x-axis


   # Recompute the z-axis to ensure orthogonality
   z_axis = np.cross(y_axis, x_axis)
   z_axis /= np.linalg.norm(z_axis)


   # Construct the rotation matrix
   rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))
   return rotation_matrix




def rotation_matrix_to_quaternion(rotation_matrix):
   """
   Converts a 3x3 rotation matrix to a quaternion.
  
   Args:
       rotation_matrix (numpy.ndarray): 3x3 rotation matrix.
      
   Returns:
       numpy.ndarray: Quaternion [w, x, y, z].
   """
   R = rotation_matrix
   trace = np.trace(R)


   if trace > 0:
       S = np.sqrt(trace + 1.0) * 2  # S=4*qw
       w = 0.25 * S
       x = (R[2, 1] - R[1, 2]) / S
       y = (R[0, 2] - R[2, 0]) / S
       z = (R[1, 0] - R[0, 1]) / S
   elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
       S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
       w = (R[2, 1] - R[1, 2]) / S
       x = 0.25 * S
       y = (R[0, 1] + R[1, 0]) / S
       z = (R[0, 2] + R[2, 0]) / S
   elif R[1, 1] > R[2, 2]:
       S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
       w = (R[0, 2] - R[2, 0]) / S
       x = (R[0, 1] + R[1, 0]) / S
       y = 0.25 * S
       z = (R[1, 2] + R[2, 1]) / S
   else:
       S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
       w = (R[1, 0] - R[0, 1]) / S
       x = (R[0, 2] + R[2, 0]) / S
       y = (R[1, 2] + R[2, 1]) / S
       z = 0.25 * S


   return np.array([w, x, y, z])