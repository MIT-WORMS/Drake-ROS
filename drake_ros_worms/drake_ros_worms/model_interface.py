#!/usr/bin/env python3

import drake_ros.core
from drake_ros.core import RosInterfaceSystem
from drake_ros.core import ClockSystem
from drake_ros.viz import RvizVisualizer

from pydrake.multibody.plant import AddMultibodyPlantSceneGraph

from pydrake.all import (
    Parser,
    DiagramBuilder,
    Simulator,
)

import sys

def main():
    # Retrieve the model file path parameter
    scene_file_path = None
    for i, param in enumerate(sys.argv):
        if param == '--scene_file_path' and i + 1 < len(sys.argv):
            scene_file_path = sys.argv[i + 1]

    if scene_file_path is None: raise ValueError("--scene_file_path was not passed")

    # Initialize drake_ros and add our node
    drake_ros.core.init()
    builder = DiagramBuilder()
    ros_interface_system = builder.AddSystem(
        RosInterfaceSystem("drake_node"))
    ClockSystem.AddToBuilder(builder, ros_interface_system.get_ros_interface())

    # Adds both MultibodyPlant and the SceneGraph, and wires them together.
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.1)

    # Add the model to the scene
    parser = Parser(plant)
    parser.AddModels(scene_file_path)
    plant.Finalize()

    # Visualization
    rviz_visualizer = builder.AddSystem(
        RvizVisualizer(ros_interface_system.get_ros_interface()))

    rviz_visualizer.RegisterMultibodyPlant(plant)
    rviz_visualizer.ComputeFrameHierarchy()

    builder.Connect(
        scene_graph.GetOutputPort('query'),
        rviz_visualizer.get_graph_query_input_port()
    )

    # Set up simulator
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator_context = simulator.get_mutable_context()

    # Step the simulator in 0.1s intervals
    length = float('inf')
    step = 0.1
    while simulator_context.get_time() < length:
        next_time = min(
            simulator_context.get_time() + step, length,
        )
        simulator.AdvanceTo(next_time)


if __name__ == '__main__':
    main()
