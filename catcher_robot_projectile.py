import numpy as np
import robotic as ry
import time

class CatcherRobot:
    def __init__(self, C, bot, dt=0.01, g=9.81):
        self.C = C
        self.bot = bot
        self.dt = dt
        self.g = g
        self.ball_initial_position = None
        self.ball_initial_velocity = None
        self.predicted_landing = None
        """ komo = ry.KOMO(self.C, 1, 1, 0, True)
        komo.addObjective([], ry.FS.scalarProductXY, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductXZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYX, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        komo.addObjective([], ry.FS.scalarProductYZ, ["l2_gripper", "l2_panda_base"], ry.OT.eq, [1e1], [0])
        ret = ry.NLP_Solver(komo.nlp()).setOptions(stopTolerance=1e-2, verbose=0).solve()
        print(ret) 
        bot.move(komo.getPath(), [1.])
        time.sleep(1)
        q0 = C.getJointState()
        C.setJointState(q0) """

    def start(self, position, velocity):
        self.ball_initial_position = np.array(position)
        self.ball_initial_velocity = np.array(velocity)
        self.predict_catch_position()
        self.move_gripper_to_position(self.bot)
        while self.bot.getTimeToEnd() > 0:
            self.bot.sync(self.C, self.dt)
        print("Gripper moved to the predicted landing position.")
        time.sleep(3)

    def predict_catch_position(self):
        if self.ball_initial_position is None or self.ball_initial_velocity is None:
            raise ValueError("Initial position and velocity must be set before predicting the catch position.")

        x0, y0, z0 = self.ball_initial_position
        vx, vy, vz = self.ball_initial_velocity
        robot_pos = self.C.getFrame("l2_gripper").getPosition()
        max_reach = 1.0  # Define the catcher's maximum reach (example: 1 meter)

        # Time resolution for checking positions
        dt = 0.01
        t = 0
        while True:
            # Ball's position at time t
            x_t = x0 + vx * t
            y_t = y0 + vy * t
            z_t = z0 + vz * t - 0.5 * self.g * t**2

            # Stop if the ball hits the ground
            if z_t <= 0:
                break

            # Check if the ball is within the catcher's reachable zone
            ball_pos = np.array([x_t, y_t, z_t])
            distance_to_robot = np.linalg.norm(ball_pos - robot_pos)
            if distance_to_robot <= max_reach:
                self.predicted_landing = ball_pos
                self.predicted_time = t
                print(f"Predicted Catch Position: {self.predicted_landing}, Time: {t:.2f} s")
                return ball_pos, t

            # Increment time
            t += dt

        # If no catch position is found
        print("No suitable catch position found.")
        return None, None

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

