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
        # Build a graph representation of the map (nodes represent locations, edges represent connections)
        # This can be adapted based on the format of your map data
        pass

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
