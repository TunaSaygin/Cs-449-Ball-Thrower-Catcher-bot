import threading
import time
import robotic as ry
import numpy as np

class CatcherRobot:
    def __init__(self, C:ry.Config, bot, dt=0.01, g=9.81):
        self.C = C
        self.bot = bot
        self.dt = dt
        self.g = g
        self.running = threading.Event()  # Use threading.Event for better control
        self.predicted_landing = None
        self.ball_positions = []
        self.time_stamps = []
        self.max_reach = 1.0  # Maximum reach of the gripper
        self.lock = threading.Lock()  # Protect shared data
        self.thread = None

    def start(self):
        """
        Start tracking the ball's trajectory in a separate thread.
        """
        if not self.running.is_set():
            print("Catcher robot started.")
            self.running.set()
            self.thread = threading.Thread(target=self.track_ball, daemon=True)
            self.thread.start()
        else:
            print("Catcher robot is already running.")

    def stop(self):
        """
        Stop the catcher robot and wait for the thread to finish.
        """
        if self.running.is_set():
            print("Stopping catcher robot...")
            self.running.clear()
            if self.thread:
                self.thread.join()  # Wait for the thread to finish
                self.thread = None
            print("Catcher robot stopped.")
        else:
            print("Catcher robot is not running.")
    def fit_pre_release_trajectory(self, degree=2):
        """
        Fit a polynomial model to the pre-release motion.

        Returns:
            tuple: Polynomial coefficients for x, y, z.
        """
        times = np.array(self.time_stamps) - self.time_stamps[0]  # Normalize time to start from 0
        positions = np.array(self.ball_positions)

        # Fit polynomial models for x, y, z
        coeffs_x = np.polyfit(times, positions[:, 0], degree)
        coeffs_y = np.polyfit(times, positions[:, 1], degree)
        coeffs_z = np.polyfit(times, positions[:, 2], degree)

        return coeffs_x, coeffs_y, coeffs_z
    def predict_release_position(self, coeffs_x, coeffs_y, coeffs_z, t_release):
        """
        Predict the position and velocity of the ball at the release point.

        Args:
            coeffs_x, coeffs_y, coeffs_z: Polynomial coefficients for x, y, z.
            t_release (float): Time of release.

        Returns:
            tuple: Position [x, y, z] and velocity [vx, vy, vz] at the release point.
        """
        x_release = np.polyval(coeffs_x, t_release)
        y_release = np.polyval(coeffs_y, t_release)
        z_release = np.polyval(coeffs_z, t_release)

        # Velocity is the derivative of the polynomial
        vx_release = np.polyval(np.polyder(coeffs_x), t_release)
        vy_release = np.polyval(np.polyder(coeffs_y), t_release)
        vz_release = np.polyval(np.polyder(coeffs_z), t_release)

        return np.array([x_release, y_release, z_release]), np.array([vx_release, vy_release, vz_release])
    def predict_landing_post_release(self, position, velocity, g=9.81):
        """
        Predict the landing position after release using projectile motion.

        Args:
            position (np.array): Initial position [x0, y0, z0].
            velocity (np.array): Initial velocity [vx, vy, vz].
            g (float): Acceleration due to gravity.

        Returns:
            np.array: Landing position [x, y, 0].
        """
        x0, y0, z0 = position
        vx, vy, vz = velocity

        # Solve for time when z = 0
        t_landing = (vz + np.sqrt(vz**2 + 2 * g * z0)) / g

        # Calculate landing position
        x_landing = x0 + vx * t_landing
        y_landing = y0 + vy * t_landing

        return np.array([x_landing, y_landing, 0])

    def track_ball(self):
        """
        Track the ball's trajectory and predict its landing position.
        """
        robot_pos = self.C.getFrame("l2_gripper").getPosition()
        print("Tracking ball...")
        while self.running.is_set():
            with self.lock:
                ball_frame = self.C.getFrame("cargo")
                ball_pos = ball_frame.getPosition()
                if ball_pos is None:
                    continue

                # Append the new ball position
                self.ball_positions.append(ball_pos)
                self.time_stamps.append(time.time())
                # Estimate velocity and predict landing position
                if len(self.ball_positions) >= 2:
                    coeffs_x, coeffs_y, coeffs_z = self.fit_pre_release_trajectory()
                    # Predict release position and velocity
                    t_release = self.time_stamps[-1]  # Use the latest timestamp as release time
                    release_position, release_velocity = self.predict_release_position(coeffs_x, coeffs_y, coeffs_z, t_release)
                    # Predict the post-release landing position
                    if release_position is not None and release_velocity is not None:
                        self.predicted_landing = self.predict_landing_post_release(release_position, release_velocity)

                # Move to the predicted landing position
                if self.predicted_landing is not None:
                    self.move_gripper_to_position(self.bot)

                self.bot.sync(self.C, self.dt)

            time.sleep(self.dt)
        print("Tracking thread exited.")

    def predict_catch_position(self, position, velocity, robot_pos):
        """
        Predict the position where the ball will be catchable.

        Args:
            position (np.array): Current position of the ball [x, y, z].
            velocity (np.array): Current velocity of the ball [vx, vy, vz].
            robot_pos (np.array): Position of the robot's gripper [x, y, z].

        Returns:
            np.array: Predicted catch position [x, y, z].
        """
        x0, y0, z0 = position
        vx, vy, vz = velocity
        t = 0

        while True:
            # Ball's position at time t
            x_t = x0 + vx * t
            y_t = y0 + vy * t
            z_t = z0 + vz * t - 0.5 * self.g * t**2

            # Stop if the ball hits the ground
            if z_t <= 0:
                break

            # Check if the ball is within the robot's reachable zone
            ball_pos = np.array([x_t, y_t, z_t])
            distance_to_robot = np.linalg.norm(ball_pos - robot_pos)
            if distance_to_robot <= self.max_reach:
                print(f"Predicted Catch Position: {ball_pos}, Time: {t:.2f}s")
                return ball_pos

            t += self.dt

        # If no catchable position is found
        print(f"No suitable catch position found.Vel={velocity}")
        return None
    
    def estimate_velocity(self):
        """
        Estimate the ball's velocity based on its observed positions.

        Returns:
            np.array: Estimated velocity [vx, vy, vz].
        """
        if len(self.ball_positions) < 2:
            return np.zeros(3)

        # Calculate velocity using the difference between the last two positions
        pos1 = np.array(self.ball_positions[-2])
        pos2 = np.array(self.ball_positions[-1])
        t1 = self.time_stamps[-2]
        t2 = self.time_stamps[-1]
        velocity = (pos2 - pos1) / (t2-t1)
        return velocity
    
    def move_gripper_to_position(self, bot):
        if self.predicted_landing is None:
            raise ValueError("Predicted landing position not calculated.")

        komo = ry.KOMO(self.C, 1, 1, 0, True)
        komo.addObjective([], ry.FS.position, ["bin"], ry.OT.eq, [1e1], self.predicted_landing)
        komo.addObjective([], ry.FS.scalarProductXY, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductXZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYX, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        ret = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        print(ret) 
        bot.move(komo.getPath(), [1.])
    
    def render_sample_points(self):
        print(f"Entered visualization of sample points (catcher robot) length of samples is {len(self.ball_positions)}")
        for it,position in enumerate(self.ball_positions):
            print(f"it:{it} position:{position}")
            self.C.addFrame(f"catch_sample_{it}").setShape(ry.ST.marker, [.3]).setPosition(position).setColor([0,1,1])
            self.C.view()
        print(f"Time between samples:{self.time_stamps[-1]-self.time_stamps[0]}")

