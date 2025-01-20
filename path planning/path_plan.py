import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import heapq

class Node:
    def __init__(self, x, y, g, h, parent=None):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def reconstruct_path(node):
    path = [(node.x, node.y)]
    while node.parent is not None:
        node = node.parent
        path.append((node.x, node.y))
    return path[::-1]

def a_star_grid(start, goal, grid):
    open_list = []
    closed_list = set()

    start_node = Node(start[0], start[1], 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add((current_node.x, current_node.y))

        if (current_node.x, current_node.y) == (goal[0], goal[1]):
            return reconstruct_path(current_node)

        neighbors = [
            (current_node.x + 1, current_node.y),
            (current_node.x - 1, current_node.y),
            (current_node.x, current_node.y + 1),
            (current_node.x, current_node.y - 1)
        ]

        for neighbor in neighbors:
            if (neighbor[0] < 0 or neighbor[0] >= grid.shape[0] or
                neighbor[1] < 0 or neighbor[1] >= grid.shape[1]):
                continue

            if (neighbor[0], neighbor[1]) in closed_list or grid[neighbor[0], neighbor[1]] == 1:
                continue

            g = current_node.g + 1
            h = heuristic(neighbor, goal)
            neighbor_node = Node(neighbor[0], neighbor[1], g, h, current_node)

            in_open_list = False
            for open_node in open_list:
                if open_node.x == neighbor_node.x and open_node.y == neighbor_node.y:
                    if neighbor_node.g < open_node.g:
                        open_list.remove(open_node)
                        heapq.heapify(open_list)
                        heapq.heappush(open_list, neighbor_node)
                    in_open_list = True
                    break

            if not in_open_list:
                heapq.heappush(open_list, neighbor_node)

    return []

def point_cloud_to_occupancy_grid(pt_cloud, resolution):
    # Extract points
    points = np.asarray(pt_cloud.points)

    # Define grid limits based on the point cloud limits
    x_limits = [np.min(points[:, 0]), np.max(points[:, 0])]
    y_limits = [np.min(points[:, 1]), np.max(points[:, 1])]

    # Define grid size
    x_range = x_limits[1] - x_limits[0]
    y_range = y_limits[1] - y_limits[0]

    grid_size_x = int(np.ceil(x_range / resolution))
    grid_size_y = int(np.ceil(y_range / resolution))

    # Initialize occupancy grid
    occupancy_grid = np.zeros((grid_size_x, grid_size_y), dtype=int)

    # Populate occupancy grid
    for point in points:
        x_idx = int(np.floor((point[0] - x_limits[0]) / resolution))
        y_idx = int(np.floor((point[1] - y_limits[0]) / resolution))

        # Ensure indices are within bounds
        if 0 <= x_idx < grid_size_x and 0 <= y_idx < grid_size_y:
            occupancy_grid[x_idx, y_idx] = 1

    return occupancy_grid

def create_voxel_configuration_space(occupancy_grid, robot_size):
    rows, cols = occupancy_grid.shape
    cspace = np.zeros((rows, cols), dtype=bool)

    # Define robot footprint as a set of voxels
    footprint_x, footprint_y = np.meshgrid(
        np.arange(-np.floor(robot_size[0]/2), np.floor(robot_size[0]/2) + 1),
        np.arange(-np.floor(robot_size[1]/2), np.floor(robot_size[1]/2) + 1)
    )
    footprint = np.vstack((footprint_x.ravel(), footprint_y.ravel())).T

    for i in range(rows):
        for j in range(cols):
            if occupancy_grid[i, j] == 1:
                for k in range(footprint.shape[0]):
                    xi = int(i + footprint[k, 0])
                    yj = int(j + footprint[k, 1])
                    if 0 <= xi < rows and 0 <= yj < cols:
                        cspace[xi, yj] = True

    return cspace

def enforce_hv_path(grid, path):
    if len(path) == 0:
        return []

    smoothed_path = [path[0]]  # Start with the first point
    i = 1

    while i <= (len(path) - 3):
        current = path[i]
        prev = smoothed_path[-1]
        next_point = path[i + 1]
        after_next = path[i + 2]

        # Check for redundant small movements
        if (abs(current[0] - prev[0]) == 0 and abs(current[1] - next_point[1]) == 0) and (abs(current[1] - prev[1]) <= 15 and abs(current[0] - next_point[0]) <= 15):
            new_point = [current[0], after_next[1]]
            if is_line_collision_free(grid, prev, new_point) and is_line_collision_free(grid, new_point, after_next):
                smoothed_path.append(new_point)
                i += 2  # Skip the next point
            else:
                smoothed_path.append(current)
                i += 1
        elif abs(current[1] - prev[1]) == 0 and abs(current[0] - next_point[0]) == 0 and (abs(current[0] - prev[0]) <= 15 and abs(current[1] - next_point[1]) <= 15):
            new_point = [after_next[0], current[1]]
            if is_line_collision_free(grid, prev, new_point) and is_line_collision_free(grid, new_point, after_next):
                smoothed_path.append(new_point)
                i += 2  # Skip the next point
            else:
                smoothed_path.append(current)
                i += 1
        else:
            smoothed_path.append(current)
            i += 1

    # Ensure the last two points are included
    smoothed_path.extend(path[-2:])
    return smoothed_path

def smooth_path(grid, path):
    if len(path) == 0:
        return []

    smoothed_path = [path[0]]  # Start with the first point
    i = 1

    while i < len(path) - 1:
        current = path[i]
        prev = smoothed_path[-1]
        next_point = path[i + 1]

        # Check for direction changes (horizontal to vertical or vertical to horizontal)
        if (prev[0] == current[0] and current[0] != next_point[0]) or (prev[1] == current[1] and current[1] != next_point[1]):
            smoothed_path.append(current)

        i += 1

    # Ensure the last point is included
    smoothed_path.append(path[-1])
    return smoothed_path

def is_line_collision_free(grid, p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    collision_free = True

    # Check points along the line for collision
    for t in np.linspace(0, 1, 101):
        x = int(round(x1 * (1 - t) + x2 * t))
        y = int(round(y1 * (1 - t) + y2 * t))
        if x < 0 or x >= grid.shape[0] or y < 0 or y >= grid.shape[1] or grid[x, y] == 1:
            collision_free = False
            break

    return collision_free


def extract_turning_points(path):
    if len(path) == 0:
        return []

    turning_points = [path[0]]  # Start with the first point

    for i in range(1, len(path) - 1):
        prev = path[i - 1]
        current = path[i]
        next_point = path[i + 1]

        # Check if the direction changes
        if ((prev[0] == current[0] or prev[1] != current[1]) and
            (current[0] != next_point[0] or current[1] == next_point[1])) or \
           ((prev[0] != current[0] or prev[1] == current[1]) and
            (current[0] == next_point[0] or current[1] != next_point[1])):
            turning_points.append(current)

    # Ensure the last point is included
    turning_points.append(path[-1])

    return turning_points

# Load point cloud
pt_cloud_seg = o3d.io.read_point_cloud("lab.pcd")

# Set the resolution of the occupancy grid
resolution = 0.1  # Adjust as needed

# Convert point cloud to 2D occupancy grid
occupancy_grid_2d = point_cloud_to_occupancy_grid(pt_cloud_seg, resolution)

# Define robot size (length and width in meters)
robot_size = np.array([686, 517]) / 1000

# Create configuration space using Voxel-based approach
cspace_voxel = create_voxel_configuration_space(occupancy_grid_2d, robot_size / resolution)

# Define start and goal points in the grid
start_point = [115, 45]  # Adjust according to your grid
goal_point = [160, 20]  # Adjust according to your grid

# Generate initial paths using A* algorithm
paths = a_star_grid(start_point, goal_point, cspace_voxel)

previous_path = []
best_path = paths

while not np.array_equal(previous_path, best_path):
    previous_path = best_path
    best_path = enforce_hv_path(cspace_voxel, best_path)
best_path = smooth_path(cspace_voxel, best_path)
turning_points = extract_turning_points(best_path)

# Display the selected trajectory
plt.imshow(cspace_voxel.T, origin='lower', cmap='Greys')
plt.title('2D Occupancy Grid with Path')
plt.axis('on')
plt.gca().invert_yaxis()
plt.plot(*zip(*turning_points), 'g', linewidth=2)
plt.legend(['Selected Trajectory'])
plt.show()

# Transfer the path back to point cloud
point_path = []
for point in turning_points:
    xpoint = (point[0] - 0.001) * resolution + pt_cloud_seg.get_min_bound()[0]
    ypoint = (point[1] - 0.001) * resolution + pt_cloud_seg.get_min_bound()[1]
    point_path.append([xpoint, ypoint])

point_path = np.array(point_path) * 1000  # Convert to mm
direction_x = point_path[1, 0] - point_path[0, 0]

if direction_x > 0:
    direction_y = 1
else:
    direction_y = -1

robot_path = []
for i in range(len(turning_points) - 1):
    xpoint = abs(point_path[i + 1, 0] - point_path[i, 0])
    ypoint = (point_path[i + 1, 1] - point_path[i, 1]) * direction_y
    robot_path.append([xpoint, ypoint])

robot_path = np.array(robot_path)

# Save the path to a text file
np.savetxt('Calibration/path.txt', robot_path, delimiter=' ', fmt='%.2f')
