``` Autonomous-Vehicle-Simulation ```

+ Involves working with the chosen simulation platform
+ Defining algorithms for path planning
+ Object detection and decision making

### I will implement high level Python based code 
+ Used: Carla and Basic Algorithms

Based on the specific requirements and the capabilities of the chosen simulator, the tasks have been divided.

``` PathPlanner Class: ```
+ __init__(self, map_data):
Initializes the PathPlanner class with map data.

+ build_graph(self, map_data):
Builds a directed graph using NetworkX based on the provided map data.

+ get_neighbors(self, node, rows, cols):
Returns neighboring nodes for a given node.

+ calculate_edge_weight(self, node1, node2):
Calculates the weight (cost) of the edge between two nodes.

+ heuristic(self, a, b):
Calculates the Euclidean distance between two points a and b.

+ plan_path(self, start, destination):
Plans a path from the start to the destination using the A* algorithm.

+ local_plan(self, current_pose, obstacles):
Implements a simple local planning algorithm, adjusting the path based on dynamic obstacles.

+ generate_random_path_segment(self, current_pose):
Generates a new random path segment for local planning.

``` SimulationController Class: ```
+ __init__(self, vehicle, path_planner, object_detector, decision_maker):
Initializes the SimulationController with instances of the Vehicle, PathPlanner, ObjectDetector, and DecisionMaker classes.

+ simulate(self, start, destination):
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
+ __init__(self, map_data, sensors):
Initializes the VehicleSimulation class with map data and sensors.
Creates instances of PathPlanner and ObjectDetector.
Initializes attributes for the current pose, obstacle positions, and planned path.

+ update_obstacles(self):
Updates obstacle positions using the object detector.

+ simulate(self, destination):
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
+ __init__(self, sensors):
Initializes the ObjectDetector class with a list of sensors.

+ process_sensor_data(self):
Placeholder for retrieving sensor data and performing object detection.
Uses a dummy image as an example for sensor data.
Returns a dummy detection result (list of detected objects).

``` DecisionMaker Class: ```
+ __init__(self, rule_based=True):
Initializes the DecisionMaker class with a boolean parameter rule_based (default is True), determining whether decision-making should be rule-based or machine learning-based.

+ make_decision(self, detected_objects, current_state):
Checks if decision-making should be rule-based or machine learning-based and calls the corresponding method.
Returns the decision.

+ rule_based_decision(self, detected_objects, current_state):
Placeholder for rule-based decision-making logic.
Example: If pedestrians are detected, prioritize stopping; otherwise, continue.

+ machine_learning_decision(self, detected_objects, current_state):
Placeholder for machine learning-based decision-making logic.
Example: Uses a trained model to determine the optimal action.
Returns a placeholder decision ("ML Decision").

``` How to integrate the code with the simulator API ```
+ Carla involves interacting with the Carla Python API
+ Make sure you have the Carla Python API installed. This can be done using: pip install carla
+ To refer to the Carla Python API: https://carla.readthedocs.io/en/latest/
