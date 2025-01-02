from thrower_generic_copy_joint_state import throw_sample
import robotic as ry
from tqdm import tqdm
import numpy as np
import json
import matplotlib.pyplot as plt
import os
import time
def generate_homogeneous_points(
robo_base, carpet_center, carpet_len, range_limit, z_min, z_max, num_points, grid_resolution=10
):
    points = []
    x_min = robo_base[0] - range_limit
    x_max = robo_base[0] + range_limit
    y_min = robo_base[1] - range_limit
    y_max = robo_base[1] + range_limit

    # Generate grid boundaries
    x_bins = np.linspace(x_min, x_max, grid_resolution)
    y_bins = np.linspace(y_min, y_max, grid_resolution)

    # Calculate the number of points per grid cell
    total_cells = (grid_resolution - 1) ** 2
    points_per_cell = num_points // total_cells

    for i in range(len(x_bins) - 1):
        for j in range(len(y_bins) - 1):
            cell_points = 0
            max_attempts = 1000  # Limit attempts to prevent infinite loop
            attempts = 0

            while cell_points < points_per_cell and attempts < max_attempts:
                attempts += 1
                # Sample random x, y, and z within the current grid cell
                x = np.random.uniform(x_bins[i], x_bins[i + 1])
                y = np.random.uniform(y_bins[j], y_bins[j + 1])
                z = np.random.uniform(z_min, z_max)

                # Check if point is outside the blue carpet area
                if (
                    (x < carpet_center[0] - carpet_len / 2 or x > carpet_center[0] + carpet_len / 2)
                    or (y < carpet_center[1] - carpet_len / 2 or y > carpet_center[1] + carpet_len / 2)
                ):
                    points.append((x, y, z))
                    cell_points += 1

            if attempts == max_attempts:
                print(f"Warning: Could not generate enough points in cell ({i}, {j})")

    # Fill any remaining points due to rounding
    while len(points) < num_points:
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = np.random.uniform(z_min, z_max)
        if (
            (x < carpet_center[0] - carpet_len / 2 or x > carpet_center[0] + carpet_len / 2)
            or (y < carpet_center[1] - carpet_len / 2 or y > carpet_center[1] + carpet_len / 2)
        ):
            points.append((x, y, z))

    return points
def generate_homogeneous_points_cylindrical(
    robo_base, carpet_center, carpet_len, range_limit, z_min, z_max, num_points, grid_resolution=10
):
    points = []
    # Define the center of the cylindrical area
    center_x, center_y = robo_base[0], robo_base[1]
    
    # Loop to generate points
    while len(points) < num_points:
        # Sample random radial distance and angle
        radius = np.random.uniform(0, range_limit)  # Random distance within the range limit
        angle = np.random.uniform(0, 2 * np.pi)    # Random angle in radians
        
        # Convert polar coordinates to Cartesian
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        z = np.random.uniform(z_min, z_max)       # Random height within the range
        
        # Check if the point is outside the carpet area
        if (
            (x < carpet_center[0] - carpet_len / 2 or x > carpet_center[0] + carpet_len / 2)
            or (y < carpet_center[1] - carpet_len / 2 or y > carpet_center[1] + carpet_len / 2)
        ):
            points.append((x, y, z))
    
    return points

#for testing this module
if __name__=="__main__":
    bin_side_len = "50cm(cylindiricaimproved)"
    failure_point_name = "optimization_failed.json"
    carpet_center = [0.2,2,0.03]
    carpet_length = 1
    robot_base = [0,2,0.05]
    range_limit = 2
    output_file = f"test_res_{bin_side_len}.json"
    num_points = 100
    test_points = generate_homogeneous_points_cylindrical(robot_base,carpet_center,carpet_length,range_limit, num_points=num_points,z_min=0.08,z_max=0.75,grid_resolution=4)
    # C = ry.Config()
    # C.addFile("throwing_bare.g")
    # for it,point in enumerate(test_points):
    #     C.addFrame(f"point-{it}").setShape(ry.ST.marker,[.1]).setPosition(point)
    # C.view()
    # time.sleep(30)
    print("Test points are generated")
    if os.path.exists(output_file):
        with open(output_file,"r") as f:
            results = json.load(f)
    else:
        results = []
    if os.path.exists(failure_point_name):
        with open(failure_point_name,"r") as f:
            failures = json.load(f)
    else:
        failures = []
    for it, point in tqdm(enumerate(test_points),desc="Processing",dynamic_ncols=True,position=0):
        try:
            result_data, deviation, trajectory_deviation = throw_sample(point,False,sleep_time=0.01)
            print(f"Test_p:{it}")
            results.append({"point":point,"result":result_data, "deviation":deviation, "trajectory_deviation":trajectory_deviation})
            with open(output_file,"w") as f:
                json.dump(results,f,indent=4)    
        except:
            print("Failed To optimize!")
            failures.append(point)
            with open(failure_point_name,"w") as f:
                json.dump(failures,f,indent=4)    
    # time.sleep(20)
    # throw_sample([-1,0.5,0.3],True,sleep_time=10)