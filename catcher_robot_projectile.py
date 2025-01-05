import threading
import time
import robotic as ry
import numpy as np

class CatcherRobot:
    def __init__(self, C, bot, dt=0.01, g=9.81):
        self.C = C
        self.bot = bot
        self.dt = dt
        self.g = g
        self.running = threading.Event()  # Use threading.Event for better control
        self.predicted_landing = None
        self.ball_positions = []
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

    def track_ball(self):
        """
        Track the ball's trajectory and predict its landing position.
        """
        robot_pos = self.C.getFrame("l2_gripper").getPosition()
        print("Tracking ball...")

        # Start a thread for predictions
        prediction_thread = threading.Thread(target=self.update_predictions, args=(robot_pos,))
        prediction_thread.start()

        while self.running.is_set():
            # Move the robot toward the latest prediction
            if self.predicted_landing is not None:
                self.move_gripper_to_position(self.bot)

            # Sync the bot for smoother movements
            self.bot.sync(self.C, self.dt)

        # Wait for prediction thread to exit
        prediction_thread.join()
        print("Tracking thread exited.")

    def update_predictions(self, robot_pos):
        while self.running.is_set():
            with self.lock:
                ball_frame = self.C.getFrame("cargo")
                ball_pos = ball_frame.getPosition()
                if ball_pos is None:
                    continue

                # Append the new ball position
                self.ball_positions.append(ball_pos)

                # Estimate velocity and predict landing position
                if len(self.ball_positions) >= 2:
                    velocity = self.estimate_velocity()
                    self.predicted_landing = self.predict_catch_position(ball_pos, velocity, robot_pos)
            #time.sleep(self.dt)  # Allow other threads to run


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
        print("No suitable catch position found.")
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
        velocity = (pos2 - pos1) / self.dt
        return velocity
    
    def move_gripper_to_position(self, bot, incremental=False):
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
        """ if incremental:
            path = komo.getPath()
            if path.shape[0] > 2:
                path = path[:2]  # Move only partially toward the goal
            bot.move(path, [0.5])
            bot.sync(self.C, 0.001)
        else:
            bot.move(komo.getPath(), [1.0])
            bot.sync(self.C, 0.001) """
        bot.move(komo.getPath(), [1.])

