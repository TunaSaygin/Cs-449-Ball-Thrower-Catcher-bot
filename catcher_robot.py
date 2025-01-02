import numpy as np
import robotic as ry
from sklearn.cluster import DBSCAN
from kalman_filter import KalmanFilter  # Custom Kalman filter implementation

class VisionCatcherRobot:
    def __init__(self, C, bot, dt=0.001):
        """
        Initialize the catcher robot.

        Args:
            C (ry.Config): Robot configuration.
            dt (float): Time step for updates.
        """
        self.C = C
        self.bot = bot
        self.dt = dt
        self.kalman_filter = KalmanFilter(dim=3)  # Assuming 3D state
        self.ball_positions = []
        self.predicted_landing = None

    def extract_ball_position(self, point_cloud):
        """
        Detect the ball position from the point cloud.

        Args:
            point_cloud (np.array): Input point cloud as a numpy array of shape (N, 3).

        Returns:
            np.array: Ball position as [x, y, z] or None if no ball detected.
        """
        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=0.05, min_samples=10).fit(point_cloud)
        labels = clustering.labels_

        # Find the largest cluster (excluding noise points, label = -1)
        unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
        if len(unique_labels) == 0:
            print("No clusters found in point cloud.")
            return None

        # Assume the ball is the largest cluster
        largest_cluster_label = unique_labels[np.argmax(counts)]
        ball_points = point_cloud[labels == largest_cluster_label]

        # Return the centroid of the ball cluster
        ball_position = np.mean(ball_points, axis=0)
        print(f'Ball position: {ball_position}')
        return ball_position

    def estimate_trajectory(self, ball_position):
        """
        Update the trajectory using Kalman filter.

        Args:
            ball_position (np.array): Observed ball position.
        """
        self.kalman_filter.update(ball_position)
        self.predicted_landing = self.kalman_filter.predict()

    def start(self):
        """
        Start the catcher robot.
        """
        ball = self.C.getFrame("cargo")
        time_step = self.dt
        x = 3
        while x > 0:
            # Simulate camera capturing a point cloud
            point_cloud = self.capture_point_cloud(ball)

            # Extract ball position
            ball_position = self.extract_ball_position(point_cloud)
            if ball_position is not None:
                self.ball_positions.append(ball_position)
                self.estimate_trajectory(ball_position)

            # Move the gripper to the predicted landing position
            if self.predicted_landing is not None:
                self.move_gripper_to_position(self.bot, self.predicted_landing)

            self.bot.sync(self.C, time_step)
            x = x - 1

    def capture_point_cloud(self, obj: ry.Frame):
        """
        Simulate capturing a point cloud from a camera.

        Args:
            camera_frame (ry.Frame): Camera frame.

        Returns:
            o3d.geometry.PointCloud: Simulated point cloud.
        """
        obj_xsize, obj_ysize, obj_zsize, obj_dim = obj.getSize()
        obj_xcenter, obj_ycenter, obj_zcenter = obj.getPosition()

        obj_xmin, obj_xmax = obj_xcenter - obj_xsize / 2, obj_xcenter + obj_xsize / 2
        obj_ymin, obj_ymax = obj_ycenter - obj_ysize / 2, obj_ycenter + obj_ysize / 2
        obj_zmin, obj_zmax = obj_zcenter - obj_zsize / 2, obj_zcenter + obj_zsize / 2

        # Camera list
        camera_names = ['camera1', 'camera2', 'camera3', 'camera4']
        all_pcl_world = []  # To store point clouds from all cameras

        for cam_name in camera_names:
            # Set up the camera
            cam = ry.CameraView(self.C)
            cam.setCamera(cam_name)
            rgb, depth = cam.computeImageAndDepth(self.C)
            pcl_camera = ry.depthImage2PointCloud(depth, cam.getFxycxy())  # (height, width, 3)

            # Flatten the point cloud
            pcl_flat = pcl_camera.reshape(-1, 3)

            # Get camera transformation (position and rotation)
            camera_frame = self.C.getFrame(cam_name)
            cam_pos = camera_frame.getPosition()  # Translation (3,)
            cam_rot = camera_frame.getRotationMatrix()  # Rotation (3x3)

            # Construct the transformation matrix (4x4)
            camera_transform = np.eye(4)
            camera_transform[:3, :3] = cam_rot  # Rotation part
            camera_transform[:3, 3] = cam_pos  # Translation part

            # Transform the point cloud to the world frame
            ones = np.ones((pcl_flat.shape[0], 1))  # Add a fourth homogeneous coordinate
            pcl_homogeneous = np.hstack([pcl_flat, ones])  # (N, 4)
            pcl_world_homogeneous = pcl_homogeneous @ camera_transform.T  # Transform (N, 4)
            pcl_world = pcl_world_homogeneous[:, :3]  # Drop the homogeneous coordinate

            # Append transformed point cloud to the global list
            all_pcl_world.append(pcl_world)

        # Merge all point clouds into one
        merged_pcl_world = np.vstack(all_pcl_world)

        # Mask points to include only those inside the obj
        obj_mask = (merged_pcl_world[:, 0] >= obj_xmin ) & (merged_pcl_world[:, 0] <= obj_xmax ) & \
                (merged_pcl_world[:, 1] >= obj_ymin ) & (merged_pcl_world[:, 1] <= obj_ymax ) & \
                (merged_pcl_world[:, 2] >= obj_zmin ) & (merged_pcl_world[:, 2] <= obj_zmax )

        # Apply the mask to get object points
        obj_pcl = merged_pcl_world[obj_mask]

        # Create a frame for the object point cloud and display it
        f = self.C.addFrame('obj_pcl', 'base')  # Can use any camera here for visualization
        f.setPointCloud(obj_pcl, [0, 255, 0])  # Display the point cloud in red

        self.C.view()  # View the scene with the object point cloud
        return obj_pcl


    def move_gripper_to_position(self, bot, position):
        """
        Move the robot gripper to the predicted landing position.

        Args:
            bot (ry.BotOp): Robot operation object.
            position (np.array): Target position [x, y, z].
        """
        komo = ry.KOMO(self.C, 1, 1, 0, True)
        komo.addObjective([], ry.FS.position, ["bin"], ry.OT.eq, [1e1], position)
        komo.addObjective([], ry.FS.scalarProductXY, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductXZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYX, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        ret = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        print(ret) 
        bot.move(komo.getPath(), [1.])
