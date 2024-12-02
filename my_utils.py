import numpy as np
def find_line_intersections_2d(bin_pos, initial_position, bin_dimensions):
    """
    Finds the intersection points of the line defined by bin_pos and initial_position
    with the boundaries of the bin in the x-y plane.

    Args:
        bin_pos (list): Position of the bin center.
        initial_position (list): Initial position of the line.
        bin_dimensions (dict): Dictionary defining the bin dimensions with 'bottom_left' and 'top_right'.

    Returns:
        list: Intersection points of the line with the bin boundaries in the x-y plane.
    """
    line_dir = np.array([bin_pos[0] - initial_position[0], bin_pos[1] - initial_position[1]])  # Line direction in x-y
    line_dir /= np.linalg.norm(line_dir)  # Normalize the direction vector
    line_point = np.array([initial_position[0], initial_position[1]])

    intersections = []

    # Extract bin boundaries
    x_min, y_min = bin_dimensions['bottom_left'][0], bin_dimensions['bottom_left'][1]
    x_max, y_max = bin_dimensions['top_right'][0], bin_dimensions['top_right'][1]

    # Check intersections with vertical boundaries (x = x_min and x = x_max)
    for x in [x_min, x_max]:
        t = (x - line_point[0]) / line_dir[0] if line_dir[0] != 0 else None
        if t is not None:
            y = line_point[1] + t * line_dir[1]
            if y_min <= y <= y_max:  # Check if the intersection is within the y-bounds
                intersections.append([x, y])

    # Check intersections with horizontal boundaries (y = y_min and y = y_max)
    for y in [y_min, y_max]:
        t = (y - line_point[1]) / line_dir[1] if line_dir[1] != 0 else None
        if t is not None:
            x = line_point[0] + t * line_dir[0]
            if x_min <= x <= x_max:  # Check if the intersection is within the x-bounds
                intersections.append([x, y])

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