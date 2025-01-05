import threading
import time
import robotic as ry
import numpy as np


class CatcherRobot:
    """
    A class representing a robot capable of catching a ball in motion.
    It uses multi-threading for prediction and movement.

    Attributes:
        C (ry.Config): The configuration of the robotic setup.
        bot (ry.BotOp): The robotic operation interface.
        dt (float): Time step for updates in seconds.
        g (float): Gravitational acceleration (default is 9.81 m/s^2).
        running (threading.Event): A flag to control thread execution.
        predicted_landing (np.array): The predicted position where the ball can be caught.
        ball_positions (list): A list of observed ball positions.
        max_reach (float): Maximum reachable distance for the robot's gripper.
        lock (threading.Lock): A lock to protect shared data across threads.
        prediction_thread (threading.Thread): Thread for trajectory prediction.
        movement_thread (threading.Thread): Thread for executing robot movements.
        timestamp (list): Timestamps of ball observations.
    """

    def __init__(self, C, bot, dt=0.001, g=9.81):
        """
        Initialize the CatcherRobot.

        Args:
            C (ry.Config): The robotic configuration object.
            bot (ry.BotOp): The robot operation object.
            dt (float): Time step for updates (default: 0.001).
            g (float): Gravitational acceleration (default: 9.81 m/s^2).
        """
        self.C = C
        self.bot = bot
        self.dt = dt
        self.g = g
        self.running = threading.Event()  # Use threading.Event for better control
        self.predicted_landing = None
        self.ball_positions = []
        self.max_reach = 1.5  # Maximum reach of the gripper
        self.lock = threading.Lock()  # Protect shared data
        self.prediction_thread = None
        self.movement_thread = None
        self.timestamp = []

    def start(self):
        """
        Start tracking the ball's trajectory in separate threads.
        """
        if not self.running.is_set():
            print("Catcher robot started.")
            self.running.set()
            self.prediction_thread = threading.Thread(target=self.update_predictions, daemon=True)
            self.movement_thread = threading.Thread(target=self.execute_movement, daemon=True)
            self.prediction_thread.start()
            self.movement_thread.start()
        else:
            print("Catcher robot is already running.")

    def stop(self):
        """
        Stop the catcher robot and wait for threads to finish.
        """
        if self.running.is_set():
            print("Stopping catcher robot...")
            self.running.clear()
            if self.prediction_thread:
                self.prediction_thread.join()
            if self.movement_thread:
                self.movement_thread.join()
            print("Catcher robot stopped.")
        else:
            print("Catcher robot is not running.")

    def update_predictions(self):
        """
        Continuously update the predicted landing position based on ball positions.
        """
        print("Updating predictions...")
        robot_pos = self.C.getFrame("l2_gripper").getPosition()

        while self.running.is_set():
            with self.lock:
                ball_frame = self.C.getFrame("cargo")
                ball_pos = ball_frame.getPosition()
                if ball_pos is None:
                    continue

                self.ball_positions.append(ball_pos)
                self.timestamp.append(time.time())

                # Estimate velocity and predict landing position
                if len(self.ball_positions) >= 2:
                    velocity = self.estimate_velocity()
                    self.predicted_landing = self.predict_catch_position(ball_pos, velocity, robot_pos)

            time.sleep(self.dt)  # Allow other threads to run

        print("Prediction thread exited.")

    def execute_movement(self):
        """
        Continuously move the gripper toward the latest predicted landing position.
        """
        while self.running.is_set():
            with self.lock:
                self.move_gripper_to_position(self.bot, incremental=True)

            self.bot.sync(self.C, 0.001)

        print("Movement thread exited.")

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
        t = 0.1

        while True:
            # Ball's position at time t
            x_t = x0 + vx * t
            y_t = y0 + vy * t
            z_t = z0 + vz * t - 0.5 * self.g * t**2

            if z_t <= 0:  # Stop if the ball hits the ground
                break

            # Check if the ball is within the robot's reachable zone
            ball_pos = np.array([x_t, y_t, z_t])
            distance_to_robot = np.linalg.norm(ball_pos - robot_pos)
            if distance_to_robot <= self.max_reach:
                print(f"Predicted Catch Position: {ball_pos}, Time: {t:.2f}s")
                return ball_pos

            t += self.dt
        print(f"position: {position} and velocity: {velocity}")
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

        pos1 = np.array(self.ball_positions[-2])
        pos2 = np.array(self.ball_positions[-1])
        self.dt = self.timestamp[-1] - self.timestamp[-2]
        velocity = (pos2 - pos1) / self.dt
        return velocity

    def move_gripper_to_position(self, bot, incremental=False):
        """
        Move the gripper toward the predicted landing position.

        Args:
            bot (ry.BotOp): Robot operation object.
            incremental (bool): Whether to move incrementally toward the goal.
        """
        if self.predicted_landing is None:
            return

        komo = ry.KOMO(self.C, 1, 1, 0, True)
        komo.addObjective([], ry.FS.position, ["bin"], ry.OT.eq, [1e1], self.predicted_landing)
        komo.addObjective([], ry.FS.scalarProductXY, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductXZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYX, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        ret = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        print(ret)

        if incremental:
            path = komo.getPath()
            if path.shape[0] > 2:
                path = path[:2]
            bot.move(path, [0.1])
        else:
            bot.move(komo.getPath(), [1.0])
