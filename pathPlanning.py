# Adding a simple A* algorithm for global path planning and basic local planning algorithm.
# Library used: Networkx for graph representation 

import networkx as nx
import heapq
import numpy as np

class PathPlanner:
    def __init__(self, map_data):
        self.map_data = map_data
        self.graph = self.build_graph(map_data)

    def build_graph(self, map_data):
        # Create a directed graph using networkx
        graph = nx.DiGraph()

        # Assuming map_data is a 2D array where each element represents a cell in the grid
        rows, cols = len(map_data), len(map_data[0])

        # Add nodes to the graph
        for row in range(rows):
            for col in range(cols):
                if map_data[row][col] == 0:  # 0 indicates a traversable cell
                    graph.add_node((row, col))

        # Add edges between adjacent traversable cells
        for row in range(rows):
            for col in range(cols):
                if map_data[row][col] == 0:  # Consider only traversable cells
                    neighbors = self.get_neighbors((row, col), rows, cols)
                    for neighbor in neighbors:
                        if map_data[neighbor[0]][neighbor[1]] == 0:  # Only connect to traversable neighbors
                            graph.add_edge((row, col), neighbor, weight=self.calculate_edge_weight((row, col), neighbor))

        return graph

    def get_neighbors(self, node, rows, cols):
        # Get neighboring nodes for a given node
        row, col = node
        neighbors = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if 0 <= row + i < rows and 0 <= col + j < cols and (i, j) != (0, 0):
                    neighbors.append((row + i, col + j))
        return neighbors

        def calculate_edge_weight(self, node1, node2):
        # Calculate the weight (cost) of the edge between two nodes
        # This could be based on the Euclidean distance or other factors
        return 1  # For simplicity, using a constant weight

# Create an instance of PathPlanner outside the class definition
map_data = [
    [0, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
]
path_planner = PathPlanner(map_data)

    def heuristic(self, a, b):
        # Calculate the Euclidean distance between two points (a and b)
        return np.linalg.norm(np.array(a) - np.array(b))

    def plan_path(self, start, destination):
        # A* algorithm for global path planning

        # Priority queue for open nodes
        open_nodes = [(0, start)]
        heapq.heapify(open_nodes)

        # Dict to store the cost to reach each node
        cost_so_far = {start: 0}

        while open_nodes:
            current_cost, current_node = heapq.heappop(open_nodes)

            if current_node == destination:
                break

            for next_node in self.graph.neighbors(current_node):
                new_cost = cost_so_far[current_node] + self.graph[current_node][next_node]["weight"]

                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(destination, next_node)
                    heapq.heappush(open_nodes, (priority, next_node))

        # Reconstruct the path
        path = []
        while current_node != start:

    def local_plan(self, current_pose, obstacles):
        # Simple local planning: adjust path based on dynamic obstacles
        # For simplicity, we'll just avoid obstacles by choosing a new random path segment

        # In a real implementation, you'd use a more sophisticated local planning algorithm
        # such as Velocity Obstacle (VO) or Dynamic Window Approach

        # For now, we'll just move a certain distance forward and adjust the path

        # Define a distance threshold to check for obstacles
        distance_threshold = 2.0  # Adjust as needed based on your simulation environment

        # Get the current position and orientation of the vehicle
        x, y, _ = current_pose

        # Check if there are obstacles within the distance threshold
        if any(np.linalg.norm(np.array([x, y]) - np.array(obstacle)) < distance_threshold for obstacle in obstacles):
            # If an obstacle is detected, generate a new random path segment
            new_path_segment = self.generate_random_path_segment(current_pose)
        else:
            # If no obstacle is detected, continue with the existing path
            new_path_segment = []

        return new_path_segment

    def generate_random_path_segment(self, current_pose):
        # Generate a new random path segment
        # This can be more sophisticated in a real implementation

        # For simplicity, let's just generate a straight path segment of a fixed length
        new_path_length = 5.0  # Adjust as needed
        new_path_segment = [
            current_pose,
            (current_pose[0] + new_path_length, current_pose[1], current_pose[2])
        ]

        return new_path_segment
