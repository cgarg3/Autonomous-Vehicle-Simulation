``` Autonomous-Vehicle-Simulation ```

+ Involves working with the chosen simulation platform
+ Defining algorithms for path planning
+ Object detection and decision making

### I will implement high level Python based code 
+ Used: Carla and Basic Algorithms

Based on the specific requirements and the capabilities of the chosen simulator, the tasks have been divided.

``` PathPlanner Class: ```
i) __init__(self, map_data)
Initializes the PathPlanner class with map data.

ii) build_graph(self, map_data)
Builds a directed graph using NetworkX based on the provided map data.

iii) get_neighbors(self, node, rows, cols)
Returns neighboring nodes for a given node.

iv) calculate_edge_weight(self, node1, node2)
Calculates the weight (cost) of the edge between two nodes.

v) heuristic(self, a, b)
Calculates the Euclidean distance between two points a and b.

vi) plan_path(self, start, destination)
Plans a path from the start to the destination using the A* algorithm.

vii) local_plan(self, current_pose, obstacles)
Implements a simple local planning algorithm, adjusting the path based on dynamic obstacles.

viii) generate_random_path_segment(self, current_pose)
Generates a new random path segment for local planning.

``` SimulationController Class: ```
i) __init__(self, vehicle, path_planner, object_detector, decision_maker)
Initializes the SimulationController with instances of the Vehicle, PathPlanner, ObjectDetector, and DecisionMaker classes.

ii) simulate(self, start, destination)
Simulates the vehicle's movement from the start to the destination.
Plans a path using the path_planner.
In a loop over waypoints:
Retrieves sensor data from the vehicle.
Processes sensor data for object detection using the object_detector.
Makes decisions based on detected objects and the current state using the decision_maker.
Updates the vehicle's state based on the decision.
Optionally includes visualization and metric evaluation steps (currently commented out).
Prints a message indicating that the simulation is completed.

``` VehicleSimulation Class: ```
i) __init__(self, map_data, sensors)
Initializes the VehicleSimulation class with map data and sensors.
Creates instances of PathPlanner and ObjectDetector.
Initializes attributes for the current pose, obstacle positions, and planned path.

ii) update_obstacles(self)
Updates obstacle positions using the object detector.

iii) simulate(self, destination)
Plans the global path using the path planner.
In a simulation loop:
Updates obstacle positions.
Gets the current waypoint from the planned path.
Performs local planning to adjust the path based on obstacles.
Updates the current pose based on local adjustment.
Removes the first waypoint from the path.
Simulates the vehicle movement.
Prints the vehicle's position at each step.

``` ObjectDetector Class: ```
i) __init__(self, sensors)
Initializes the ObjectDetector class with a list of sensors.

ii) process_sensor_data(self)
Placeholder for retrieving sensor data and performing object detection.
Uses a dummy image as an example for sensor data.
Returns a dummy detection result (list of detected objects).

