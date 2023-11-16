# outlines a good structure for orchestrating the simulation
# Adding a real time visualisation component and incorporating metrics evaluation
# Including appropriate methods for obtaining sensor data, updating the state based on decisions and accessing the current data

class SimulationController:
    def __init__(self, vehicle, path_planner, object_detector, decision_maker, visualizer=None, metrics_evaluator=None):
        self.vehicle = vehicle
        self.path_planner = path_planner
        self.object_detector = object_detector
        self.decision_maker = decision_maker
        self.visualizer = visualizer  # Optional: Real-time visualization component
        self.metrics_evaluator = metrics_evaluator  # Optional: Metrics evaluator

    def simulate(self, start, destination):
        # Plan path
        waypoints = self.path_planner.plan_path(start, destination)

        for waypoint in waypoints:
            # Get sensor data
            sensor_data = self.vehicle.get_sensor_data()

            # Process sensor data for object detection
            detected_objects = self.object_detector.process_sensor_data(sensor_data)

            # Make decisions based on detected objects and current state
            decision = self.decision_maker.make_decision(detected_objects, self.vehicle.get_state())

            # Update vehicle state based on decision
            self.vehicle.update_state(decision)

            # Real-time visualization
            if self.visualizer:
                self.visualizer.update_visualization(sensor_data, detected_objects, decision, self.vehicle.get_state())

            # Evaluate metrics (if needed)
            if self.metrics_evaluator:
                self.metrics_evaluator.evaluate_metrics(sensor_data, detected_objects, decision, self.vehicle.get_state())

        # Simulation completed
        print("Simulation completed.")

# Example usage:
# Assume you have appropriate classes for Visualizer and MetricsEvaluator
visualizer = Visualizer()
metrics_evaluator = MetricsEvaluator()

# Create instances of Vehicle, PathPlanner, ObjectDetector, DecisionMaker
vehicle = Vehicle()
path_planner = PathPlanner(map_data)
object_detector = ObjectDetector(sensors)
decision_maker = DecisionMaker(rule_based=True)

# Create an instance of SimulationController
simulation_controller = SimulationController(vehicle, path_planner, object_detector, decision_maker, visualizer, metrics_evaluator)

# Start simulation
simulation_controller.simulate(start=(0, 0), destination=(3, 3))
