import numpy as np
import matplotlib.pyplot as plt


class RobotArm:
    def __init__(self, n_joints, total_length=6.0, base_position=(0, 0)):
        assert 2 <= n_joints <= 12, "Joints must be in [2, 12]"
        self.n_joints = n_joints
        self.base_position = np.array(base_position)
        self.total_length = total_length
        self.link_length = total_length / n_joints

        self._canonical_angles = np.array([0, 0, np.pi / 6, 0, 0, -np.pi / 6, 0, 0, 0, 0, 0, 0])
        self._canonical_points = self._forward_kinematics(self._canonical_angles, total_length / 12)

        self.target_points = self._resample_path(self._canonical_points, n_joints + 1)

        self.start_config = self._inverse_kinematics_from_points(self.target_points)
        self.goal_config = [-theta for theta in self.start_config]

    def _forward_kinematics(self, joint_angles, link_length):
        points = [self.base_position.copy()]
        x, y = self.base_position
        angle = 0
        for theta in joint_angles:
            angle += theta
            x += link_length * np.cos(angle)
            y += link_length * np.sin(angle)
            points.append((x, y))
        return np.array(points)

    def _resample_path(self, points, n_samples):
        distances = np.cumsum([0] + [np.linalg.norm(points[i + 1] - points[i]) for i in range(len(points) - 1)])
        total_dist = distances[-1]
        target_distances = np.linspace(0, total_dist, n_samples)

        x_interp = np.interp(target_distances, distances, points[:, 0])
        y_interp = np.interp(target_distances, distances, points[:, 1])
        return np.vstack((x_interp, y_interp)).T

    def _inverse_kinematics_from_points(self, points):
        joint_angles = []
        angle = 0
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            target_angle = np.arctan2(dy, dx)
            joint_angle = target_angle - angle
            joint_angles.append(joint_angle)
            angle = target_angle
        return joint_angles

    def forward_kinematics(self, joint_angles):
        return self._forward_kinematics(joint_angles, self.link_length)

    def plot(self, joint_angles=None, title="Robot Arm"):
        if joint_angles is None:
            joint_angles = self.start_config
        points = self.forward_kinematics(joint_angles)
        plt.figure(figsize=(6, 6))
        plt.plot(points[:, 0], points[:, 1], '-o', linewidth=2)
        plt.xlim(-self.total_length, self.total_length)
        plt.ylim(-self.total_length, self.total_length)
        plt.gca().set_aspect('equal')
        plt.title(title)
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    for dof in [2, 4, 6, 8, 10, 12]:
        arm = RobotArm(dof)
        print(f"{dof}-DOF Arm")
        arm.plot(arm.start_config, title=f"{dof}-DOF Arm - Start Config")
        arm.plot(arm.goal_config, title=f"{dof}-DOF Arm - Goal Config")
