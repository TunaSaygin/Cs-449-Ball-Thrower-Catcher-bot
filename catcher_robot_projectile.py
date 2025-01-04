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

    def start(self, position, velocity):
        self.ball_initial_position = np.array(position)
        self.ball_initial_velocity = np.array(velocity)
        self.predict_landing_position()
        self.move_gripper_to_position(self.bot)
        while self.bot.getTimeToEnd() > 0:
            self.bot.sync(self.C, self.dt)
        print("Gripper moved to the predicted landing position.")
        time.sleep(3)

    def predict_landing_position(self):
        if self.ball_initial_position is None or self.ball_initial_velocity is None:
            raise ValueError("Initial position and velocity must be set before predicting the landing position.")

        x0, y0, z0 = self.ball_initial_position
        vx, vy, vz = self.ball_initial_velocity

        # Solve for time to hit the ground using projectile motion formula
        # z = z0 + vz * t - 0.5 * g * t^2
        # When the ball hits the ground, z = 0:
        # 0 = z0 + vz * t - 0.5 * g * t^2
        # 0.5 * g * t^2 - vz * t - z0 = 0
        a = 0.5 * self.g
        b = -vz
        c = -z0

        # Quadratic formula: t = (-b ± sqrt(b^2 - 4ac)) / 2a
        discriminant = b**2 - 4 * a * c
        if discriminant < 0:
            raise ValueError("The ball does not appear to reach the ground.")

        t_ground = (-b + np.sqrt(discriminant)) / (2 * a)  # Use positive root for time

        # Calculate landing position
        x_land = x0 + vx * t_ground
        y_land = y0 + vy * t_ground
        self.predicted_landing = np.array([x_land, y_land, 0])
        print(f"Predicted Landing Position: {self.predicted_landing}")


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

