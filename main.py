# main.py
from path_planner import PathPlanner
from simulation_controller import SimulationController
from vehicle_simulation import VehicleSimulation
from object_detector import ObjectDetector
from decision_maker import DecisionMaker

# Main script
if __name__ == "__main__":
    # Connect to Carla server
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    # Load Carla world
    world = client.get_world()

    # Define your map_data and sensors
    map_data = ...  # Define your map data
    sensors = [...]  # Define your sensors

    # Initialize your simulation
    vehicle_simulation = VehicleSimulation(map_data, sensors, world)

    # Start the simulation
    destination = ...  # Define your destination
    vehicle_simulation.simulate(destination)
