#!/usr/bin/env python3

# Drake ROS imports
import drake_ros.core
from drake_ros.core import RosInterfaceSystem, RosPublisherSystem, RosSubscriberSystem, ClockSystem
from drake_ros.viz import RvizVisualizer

# Pydrake imports
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, LeafSystem, Context, BasicVector
from pydrake.math import RigidTransform
from pydrake.geometry import HalfSpace
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    MultibodyPlant, 
    CoulombFriction, 
    DiscreteContactApproximation, 
    AddMultibodyPlantSceneGraph
)
from pydrake.common.value import AbstractValue

# ROS imports
import rclpy
import rclpy.logging
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.qos import QoSProfile
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

# Extra imports
from collections import defaultdict
import argparse
import re
import numpy as np

class DrakeSimNode(Node):
    """
    A node dedicated to progressing and resetting the Drake simulation. Sending a
    `/reset_simulation` empty service will reset the simulation to its initial state.
    """

    def __init__(
            self,
            sim_period: float,
            simulator: Simulator | None = None
        ) -> None:
        """
        Initializes the util node and stores the simulator reference for resetting later. If the
        simulator is not passed at initialization, it must be set at a later time with a call 
        to `store_simulator`.

        Args:
            sim_period (float): The timestep to advance the simulator by
            simulator (Simulator): The simulator reference to reset the context on
        """
        super().__init__('drake_util_node')
        self._logger = self.get_logger()
        self._sim_period = sim_period

        # Simulator setup if it was passed during initialization
        self._simulator = None
        self._sim_context = None
        self.store_simulator(simulator)

        # Simulator timer callback
        self.timer = self.create_timer(self._sim_period, self.sim_callback)
        
        # Create the reset service
        self.srv = self.create_service(Empty, 'reset_simulation', self.reset_callback)

    @property
    def logger(self) -> RcutilsLogger:
        return self._logger

    def store_simulator(
            self,
            simulator: Simulator
        ) -> None:
        """
        Stores the simulator referece if it was not set during initialization.

        Args:
            simulator (Simulator): The simulator reference to reset the context on
        """
        if simulator:
            assert isinstance(simulator, Simulator)
            self.logger.info("Starting Drake simulation...")
            self._simulator = simulator
            simulator.Initialize()
            simulator.set_target_realtime_rate(1.0)
            self._sim_context = simulator.get_mutable_context()

    def sim_callback(self) -> None:
        """Progresses the simulation by a fixed timestep."""
        if self._simulator:
            next_time = self._sim_context.get_time() + self._sim_period
            self._simulator.AdvanceTo(next_time)

    def reset_callback(
            self, 
            request: Empty.Request, 
            response: Empty.Response
        ) -> Empty.Response:
        """
        Callback function for resetting the Drake simulation.
        
        Args:
            request (Empty.Request): The empty request for this service call
            response (Empty.Response): The empty response for this service
        """

        if self._simulator:
            # Reset the simulation
            self.logger.info("Resetting the simulation...")
            context = self._simulator.get_mutable_context()
            context.SetTime(0)
            self._simulator.Initialize()
            self._simulator.get_system().SetDefaultContext(context)
            self._sim_context = context

        else:
            # Log an error, node was not fully initialized yet
            self.logger.error(
                "`/reset_simulation` was called before the simulator "
                "was initialized in the Drake util node."
            )

        return response

class DrakeInterface(LeafSystem):
    """
    A DrakeInterface system is responsible for taking all actuator inputs from ROS and combining them
    into a single output for Drake. In addition, it will publish all actuator states to ROS topics.
    
    For an actuation interface to be established, the .urdf or .sdf for the model must name every
    actuator field as `<actuator name="{namespace}_actuator_{n}">`, where `{namespace}` is the namespace
    published to by the WORM/Species, and `{n}` is the zero-indexed position in the `actuation_cmd` message.
    Additionally, the corresponding joint fields must be named as `<joint name="{namespace}_joint_{n}" type="...">`.

    .. warning::
        Only actuated joints will produce state outputs. There is currently no support for unactuated joint states.
    """

    def __init__(
            self,
            plant: MultibodyPlant,
            util_node: Node
        ) -> None:
        """
        Responsible for initializing the interface system and setting up all inputs and outputs.

        Args:
            plant (MultibodyPlant): The plant to set up the ROS interface for
            util_node (Node): A utility node, used to get ROS time for publishing messages
        """
        super().__init__()
        self.util_node = util_node

        # Set up the actuation map -> {'namespace': (input_port, actuator_indices)}
        self.actuation_map = None
        self.actuation_output = None
        self._actuation_setup(plant)

        # Set up the state map -> {'namespace': (output_port, joint_indices)}
        self.state_map = None
        self.state_input = None
        self._state_setup(plant)

    @property
    def namespaces(self) -> list[str]:
        return list(self.actuation_map.keys())

    def calc_actuation_output(
            self,
            context: Context,
            data: BasicVector
        ) -> None:
        """
        Full actuation output callback. Combines subscribed ROS commands into a single Drake command.

        Args:
            context (Context): The context at the time of callback, used to evaluate the input ports
            data (BasicVector): The data vector to be filled with the full command
        """

        # Evaluate all actuation input ports on the context and store their values
        output = np.zeros(data.value().size)
        for (input_port, actuator_indices) in self.actuation_map.values():
            cur_cmd = np.array(input_port.Eval(context).effort)
            if cur_cmd.size == 0: continue  # Skip unpublished messages
            output[actuator_indices] = cur_cmd

        # Set the output to the full actuation command
        data.SetFromVector(output)

    def calc_state_output(
            self,
            context: Context,
            data: AbstractValue,
            ns: str
        ) -> None:
        """
        Individual state output callback. Divides Drake outputs into multiple ROS outputs.

        Args:
            context (Context): The context at the time of callback, used to evaluate the input ports
            data (AbstractValue): The ROS JointState message to populate
            ns (str): Namespace of the joints this output corresponds to
        """
        
        # Get the joint configuration for this namespace
        output_port, joint_indices = self.state_map[ns]
        state = self.state_input.Eval(context)
        q = state[joint_indices]
        qdot = state[joint_indices + len(state) // 2]

        # Create the ROS message
        msg = JointState()
        msg.header.stamp = self.util_node.get_clock().now().to_msg()
        msg.position = q.tolist()
        msg.velocity = qdot.tolist()
        
        # Set the output
        data.set_value(msg)

    def _actuation_setup(
            self,
            plant: MultibodyPlant,
        ) -> None:
        """
        Sets up the actuation subscriber ports and the actuation output port.

        Args:
            plant (MultibodyPlant): The plant to initialize actuation subscribers for
        """

        # Extract namespaces from the actuator names (ex: pony_actuator_0)
        actuator_names = plant.GetActuatorNames(add_model_instance_prefix=False)
        pattern = r"^(.*)_actuator_(\d+)$"
        namespace_dict = defaultdict(list)
        for name in actuator_names:
            match = re.match(pattern, name)

            # Invalid actuator name
            if not match:
                raise ValueError(
                    f"Actuator name, {name}, did not match the required format "
                    "{module}_actuator_{n}. Update the .urdf/.sdf"
                )

            namespace = match.group(1)
            actuator_num = int(match.group(2))
            namespace_dict[namespace].append(actuator_num)

        # Assert format to detect errors early
        for namespace, actuator_indices in namespace_dict.items():

            # Namespaces must be all lowercase
            if not namespace.islower():
                raise RuntimeError(
                    f"An auto-extracted namespace ({namespace}) in the DrakeInterface node "
                    "had capital letters. Either the .urdf/.sdf was incorrect, or your namespace "
                    "is named improperly."
                )
            
            # Actuators must be zero-indexed
            actuator_indices.sort()
            if actuator_indices[0] != 0:
                raise RuntimeError(
                    f"Module, {namespace}, with actuator indices, {actuator_indices}, is not zero indexed."
                )
            
            # Cannot be a gap in actuator indices
            for i in range(1, len(actuator_indices)):
                if actuator_indices[i] != actuator_indices[i - 1] + 1:
                    raise RuntimeError(
                        f"Module {namespace} had a gap in actuator indices: {actuator_indices}"
                    )
                
        # Create an input port for each namespace and store in the map
        self.actuation_map = {}
        for namespace, actuator_indices in namespace_dict.items():

            # Extract an actuator index map for each namespace
            actuators = [plant.GetJointActuatorByName(f'{namespace}_actuator_{n}') for n in actuator_indices]
            actuator_indices = [actuator.input_start() for actuator in actuators]

            # Store a map of each namespace to the corresponding input port and index map
            self.actuation_map[namespace] = (
                self.DeclareAbstractInputPort(
                    f'/{namespace}/actuation_cmd', 
                    AbstractValue.Make(JointState())
                ),
                np.array(actuator_indices)
            )

        # Establish the combined output port for all ROS actuation messages
        self.actuation_output = self.DeclareVectorOutputPort(
            'actuation',
            plant.num_actuated_dofs(),
            self.calc_actuation_output
        )

    def _state_setup(
            self,
            plant: MultibodyPlant
        ) -> None:
        """
        Sets up the state publisher output ports and the state input port. Must be called after
        actuation setup.

        Args:
            plant (MultibodyPlant): The plant to initialize state publishers for            
        """
        
        # Relies on error checking done in actuation setup
        assert self.actuation_map is not None

        # Extract joint map from joint names (ex: pony_joint_0)
        joint_names = plant.GetPositionNames(add_model_instance_prefix=False, always_add_suffix=False)

        # Assert format to detect errors early
        self.state_map = {}
        for namespace, (_, actuator_indices) in self.actuation_map.items():

            # Must be one joint for every actuator 
            joint_indices = []
            for n, _ in enumerate(actuator_indices):
                
                expected_name = f"{namespace}_joint_{n}"
                if expected_name not in joint_names:
                    raise ValueError(
                        f"Joint name, {expected_name}, must exist due to the existence "
                        f"of {namespace}_actuator_{n}. All joint names: {joint_names}"
                    )

                # Get the index into the global q vector for this joint
                joint = plant.GetJointByName(expected_name)
                joint_indices.append(joint.position_start())

            # Store a map of each namespace to the corresponding output port and index map
            self.state_map[namespace] = (
                self.DeclareAbstractOutputPort(
                    f'/{namespace}/state',
                    lambda: AbstractValue.Make(JointState()),
                    lambda context, data, ns=namespace : self.calc_state_output(context, data, ns)
                ),
                np.array(joint_indices)
            )

        # Establish the combined input port for all Drake state messages
        self.state_input = self.DeclareVectorInputPort(
            'state',
            plant.get_state_output_port().size()
        )


def log_debug_topics(
        logger: RcutilsLogger,
        actuation_topics: list[str],
        state_topics: list[str]
    ) -> None:
    """
    Logs a message to the logger indicating what topics are being subscribed
    and published to.

    Args:
        logger (RcutilsLogger): A ROS2 logging interface
        actuation_topics ([str]): A list of actuation topics subscribed to
        state_topics ([str]): A list of state topics published to
    """
    actuation_msg = "Subscribing to the following topics:"
    for topic in actuation_topics: actuation_msg += f"\n\t{topic}"
    logger.info(actuation_msg)

    state_msg = "Publishing to the following topics:"
    for topic in state_topics: state_msg += f"\n\t{topic}"
    logger.info(state_msg)


def parse_args() -> tuple[str, float]:
    """
    Parses and returns the arguments required for setup.

    Returns:
        A tuple containing the path to a .urdf or .sdf and the
        target period of the simulator.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene-file-path",
        help="Path to the .urdf or .sdf to load.",
        type=str,
        required=True
    )
    parser.add_argument(
        "--simulator-step",
        help="Simulator timestep.",
        type=float,
        default=0.01
    )
    args = parser.parse_known_args()[0]

    return args.scene_file_path, args.simulator_step


def main():
    model_path, sim_period = parse_args()

    # Initialize drake_ros and add our node
    drake_ros.core.init()
    builder = DiagramBuilder()
    ros_interface_system = builder.AddSystem(RosInterfaceSystem('drake_node'))
    ClockSystem.AddToBuilder(builder, ros_interface_system.get_ros_interface())

    # Create the util node for resetting the simulation, we will pass the sim reference later
    rclpy.init()
    sim_node = DrakeSimNode(sim_period)

    # Add the MultibodyPlant and SceneGraph
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_period)
    plant.set_discrete_contact_approximation(DiscreteContactApproximation.kSap)

    # Add the model to the scene
    parser = Parser(plant)
    parser.AddModels(model_path)

    # Add a ground plane to the scene
    # NOTE (trejohst): We can uncomment this to see the ground if we wish
    # plant.RegisterVisualGeometry(
    #     plant.world_body(), 
    #     RigidTransform(), 
    #     HalfSpace(), 
    #     "GroundVisual", 
    #     np.array([1, 1, 1, 1])
    # )
    plant.RegisterCollisionGeometry(
        plant.world_body(), 
        RigidTransform(), 
        HalfSpace(), 
        "GroundCollision",
        CoulombFriction(1.0, 1.0)
    )
    plant.Finalize()

    # Visualization
    rviz_visualizer = builder.AddSystem(RvizVisualizer(ros_interface_system.get_ros_interface()))
    rviz_visualizer.RegisterMultibodyPlant(plant)
    rviz_visualizer.ComputeFrameHierarchy()
    builder.Connect(scene_graph.GetOutputPort('query'), rviz_visualizer.get_graph_query_input_port())

    # Add our ROS interface system and wire the unique ports
    interface = builder.AddSystem(DrakeInterface(plant, sim_node))
    builder.Connect(plant.get_state_output_port(), interface.GetInputPort('state'))
    builder.Connect(interface.GetOutputPort('actuation'), plant.get_actuation_input_port())

    qos = QoSProfile(depth=10)

    # Subscribe and publish to all required topics for this model
    actuation_topics = []
    state_topics = []
    for namespace in interface.namespaces:

        # Subscribe to the ROS actuation command, passing it to the interface
        actuation_topic = f"/{namespace}/actuation_cmd"
        actuation_sub = builder.AddSystem(
            RosSubscriberSystem.Make(
                JointState, 
                actuation_topic, 
                qos, 
                ros_interface_system.get_ros_interface()
            )
        )
        builder.Connect(actuation_sub.get_output_port(0), interface.GetInputPort(actuation_topic))
        actuation_topics.append(actuation_topic)

        # Publish a ROS state command, getting it from the interface
        state_topic = f"/{namespace}/state"
        state_pub = builder.AddSystem(
            RosPublisherSystem.Make(
                JointState, 
                state_topic, 
                qos, 
                ros_interface_system.get_ros_interface()
            )
        )
        builder.Connect(interface.GetOutputPort(state_topic), state_pub.get_input_port(0))
        state_topics.append(state_topic)

    # Log topics for debug
    log_debug_topics(sim_node.logger, actuation_topics, state_topics)

    # Set up simulator
    diagram = builder.Build()
    simulator = Simulator(diagram)
    sim_node.store_simulator(simulator)
    rclpy.spin(sim_node)

    # Cleanup when exiting
    sim_node.logger.info("Stopping Drake simulation...")
    sim_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()