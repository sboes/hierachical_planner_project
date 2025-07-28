import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from shapely.geometry import Point, LineString
from collections import deque
from scipy.spatial import KDTree
from IPython.display import display, HTML
from scipy.optimize import minimize


class RobotArm:
    def __init__(self, n_joints, total_length=6.0, base_position=(0, 0), obstacles=None):
        assert 2 <= n_joints <= 12, "Joints must be in [2, 12]"
        self.n_joints = n_joints
        self.base_position = np.array(base_position)
        self.total_length = total_length
        self.link_length = total_length / n_joints
        self.obstacles = obstacles if obstacles is not None else []

        self._canonical_angles = np.array([0, 0, np.pi / 6, 0, 0, -np.pi / 6, 0, 0, 0, 0, 0, 0])
        self._canonical_points = self._forward_kinematics(self._canonical_angles, total_length / 12)

        self.target_points = self._resample_path(self._canonical_points, n_joints + 1)

        self.start_config = self._inverse_kinematics_from_points(self.target_points)
        self.goal_config = [-theta for theta in self.start_config]

    def set_goal_position(self, goal_xy, max_iter=500, tol=1e-3):

        goal_xy = np.array(goal_xy)

        if np.linalg.norm(goal_xy - self.base_position) > self.total_length:
            print("Goal unreachable: beyond arm's length.")
            return

        def objective(joint_angles):
            ee_pos = self.forward_kinematics(joint_angles)[-1]
            return np.linalg.norm(ee_pos - goal_xy) ** 2

        def constraint_collision(joint_angles):
            return 0.0 if not self.check_collision(joint_angles) else 1.0

        result = minimize(
            objective,
            x0=np.array(self.start_config),
            method='L-BFGS-B',
            bounds=[(-np.pi, np.pi)] * self.n_joints,
            options={'maxiter': max_iter, 'ftol': tol}
        )

        if result.success:
            final_ee = self.forward_kinematics(result.x)[-1]
            error = np.linalg.norm(final_ee - goal_xy)
            if error < 1e-1 and not self.check_collision(result.x):
                self.goal_config = result.x.tolist()
                print(f"Goal reached at {final_ee.round(2)} (error: {error:.4f})")
            else:
                print(f"Optimization succeeded but goal not reached or in collision. Error: {error:.4f}")
                self.goal_config = result.x.tolist()
        else:
            print("Optimization failed.")

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

    def check_collision(self, joint_angles):
        points = self.forward_kinematics(joint_angles)
        for i in range(1, len(points)):
            link_line = LineString([points[i - 1], points[i]])
            for obs in self.obstacles:
                obstacle = Point(obs['center']).buffer(obs['radius'])
                if link_line.intersects(obstacle):
                    return True
        return False


class BasicPRM:
    def __init__(self, robot, num_samples=200, k=10):
        self.robot = robot
        self.num_samples = num_samples
        self.k = k
        self.samples = []
        self.roadmap = {}
        self.start_index = None
        self.goal_index = None

    def sample_config(self):
        while True:
            config = np.random.uniform(-np.pi, np.pi, self.robot.n_joints)
            if not self.robot.check_collision(config):
                return config

    def build_roadmap(self):
        self.samples = [self.robot.start_config, self.robot.goal_config]

        while len(self.samples) < self.num_samples:
            self.samples.append(self.sample_config())

        tree = KDTree(self.samples)
        self.roadmap = {i: [] for i in range(len(self.samples))}

        for i, config in enumerate(self.samples):
            _, neighbors = tree.query(config, k=self.k + 1)
            for j in neighbors[1:]:
                if j not in self.roadmap[i] and self.edge_is_valid(self.samples[i], self.samples[j]):
                    self.roadmap[i].append(j)
                    self.roadmap[j].append(i)

        self.start_index = 0
        self.goal_index = 1

    def edge_is_valid(self, config1, config2, steps=10):
        for alpha in np.linspace(0, 1, steps):
            interp = (1 - alpha) * np.array(config1) + alpha * np.array(config2)
            if self.robot.check_collision(interp):
                return False
        return True

    def find_path(self):
        queue = deque()
        queue.append(self.start_index)
        visited = [False] * len(self.samples)
        parent = [-1] * len(self.samples)
        visited[self.start_index] = True

        while queue:
            current = queue.popleft()
            if current == self.goal_index:
                break
            for neighbor in self.roadmap[current]:
                if not visited[neighbor]:
                    visited[neighbor] = True
                    parent[neighbor] = current
                    queue.append(neighbor)

        if not visited[self.goal_index]:
            return []

        path = []
        current = self.goal_index
        while current != -1:
            path.append(current)
            current = parent[current]
        path.reverse()
        return path

    def get_path_configs(self, path_indices):
        return [self.samples[i] for i in path_indices]


def plot_prm(robot, planner, path_indices=None):
    fig, ax = plt.subplots(figsize=(6, 6))

    for obs in robot.obstacles:
        circle = plt.Circle(obs['center'], obs['radius'], color='red', alpha=0.5)
        ax.add_patch(circle)

    for i, neighbors in planner.roadmap.items():
        point_i = robot.forward_kinematics(planner.samples[i])[-1]
        for j in neighbors:
            point_j = robot.forward_kinematics(planner.samples[j])[-1]
            ax.plot([point_i[0], point_j[0]], [point_i[1], point_j[1]], color='gray', linewidth=0.5)

    node_positions = np.array([robot.forward_kinematics(cfg)[-1] for cfg in planner.samples])
    ax.plot(node_positions[:, 0], node_positions[:, 1], 'ko', markersize=2)

    start = robot.forward_kinematics(robot.start_config)[-1]
    goal = robot.forward_kinematics(robot.goal_config)[-1]
    ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
    ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')

    if path_indices:
        path_points = [robot.forward_kinematics(planner.samples[i])[-1] for i in path_indices]
        path_points = np.array(path_points)
        ax.plot(path_points[:, 0], path_points[:, 1], 'b-', linewidth=2, label='Planned Path')

    ax.set_xlim(-robot.total_length, robot.total_length)
    ax.set_ylim(-robot.total_length, robot.total_length)
    ax.set_aspect('equal')
    ax.set_title("PRM Roadmap with Path")
    ax.legend()
    plt.show()


def interpolate_configs(configs, steps_per_segment=20):
    smooth_configs = []
    for i in range(len(configs) - 1):
        for alpha in np.linspace(0, 1, steps_per_segment, endpoint=False):
            interp = (1 - alpha) * np.array(configs[i]) + alpha * np.array(configs[i + 1])
            smooth_configs.append(interp)
    smooth_configs.append(configs[-1])
    return smooth_configs


def animate_path(robot, configs, interval=1000 / 60):
    smooth_configs = interpolate_configs(configs, steps_per_segment=20)

    fig, ax = plt.subplots(figsize=(6, 6))
    line, = ax.plot([], [], '-o', linewidth=2, color='blue')

    for obs in robot.obstacles:
        circle = plt.Circle(obs['center'], obs['radius'], color='red', alpha=0.5)
        ax.add_patch(circle)

    ax.set_xlim(-robot.total_length, robot.total_length)
    ax.set_ylim(-robot.total_length, robot.total_length)
    ax.set_aspect('equal')
    ax.set_title("Robot Arm Path Animation")

    def update(frame):
        angles = smooth_configs[frame]
        points = robot.forward_kinematics(angles)
        line.set_data(points[:, 0], points[:, 1])
        return line,

    ani = FuncAnimation(fig, update, frames=len(smooth_configs), interval=interval, blit=True)
    plt.close(fig)
    return HTML(ani.to_jshtml())


if __name__ == "__main__":
    env_obstacles = [
        {'center': (2, 2), 'radius': 0.5},
        {'center': (-1, 1), 'radius': 0.8},
        {'center': (0, -2), 'radius': 0.6},
    ]

    arm = RobotArm(3, obstacles=env_obstacles)
    arm.set_goal_position((2, 4))  # change to any (x, y) you want
    planner = BasicPRM(arm, num_samples=300, k=10)
    planner.build_roadmap()
    path_indices = planner.find_path()
    plot_prm(arm, planner, path_indices)

    if path_indices:
        path_configs = planner.get_path_configs(path_indices)
        print(f"Path found with {len(path_configs)} configurations:")
        for i, cfg in enumerate(path_configs):
            print(f"  Step {i}: {np.round(cfg, 2)}")
        display(animate_path(arm, path_configs))

    else:
        print("No valid path found.")
