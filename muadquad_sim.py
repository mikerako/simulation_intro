##
#
# Simple simulation of muadquad quadruped with various options
#
#
# The muadquad is a robot with the following inputs
# and outputs:
#
#                              ---------------------------------
#                              |                               |
#                              |                               |
#                              |                               |
#                              |                               | --> measured_hip_angle
#                              |                               | --> measured_hip_angular_vel
#     torso_target ----------> |         MuadQuad Leg          | --> measured_hip_torque
#     torso_target ----------> |                               |
#                              |                               |
#                              |                               | --> measured_knee_angle
#                              |                               | --> measured_knee_angular_vel
#                              |                               | --> measured_knee_torque
#                              |                               |
#                              |                               |
#                              |                               | --> measured_torso_position
#                              |                               | --> measured_torso_velocity
#                              |                               |
#                              |                               |
#                              |                               |
#                              |                               |
#                              |                               |
#                              |                               |
#                              ---------------------------------
#
#
#
# See the "Parameters" section below for different ways of using and visualizing
# this system. 
#
##

import ipdb
from pydrake.all import *
import numpy as np
import matplotlib.pyplot as plt
import time

from stations.muadquad_station import MuadQuadStation

########################### Parameters #################################

# Make a plot of the inner workings of the station
show_station_diagram = True

# Make a plot of the diagram for this example, where only the inputs
# and outputs of the station are shown
show_toplevel_diagram = False

# Run a quick simulation
simulate = True

########################################################################

# Set up the kinova station
station = MuadQuadStation(time_step=0.002)
station.SetupSingleLegScenario()
station.ConnectToMeshcatVisualizer()
station.Finalize()

if show_station_diagram:
    # Show the station's system diagram
    plt.figure()
    plot_system_graphviz(station,max_depth=1)
    plt.show()

# Connect input ports to the kinova station
builder = DiagramBuilder()
builder.AddSystem(station)


# pose_des = np.array([0.0,0.0,0.0,
#                      0.1,0.0,0.15])
pose_des = np.array([0.0,0.0,0.0,
                     -0.283,0.0,0.100])
target_source = builder.AddSystem(ConstantVectorSource(pose_des))


# Send foot command
builder.Connect(
        target_source.get_output_port(),
        station.GetInputPort("foot_target"))

target_source.set_name("foot_command_source")

# Loggers force certain outputs to be computed
# wrench_logger = LogVectorOutput(station.GetOutputPort("measured_foot_wrench"),builder)
# wrench_logger.set_name("wrench_logger")

pose_logger = LogVectorOutput(station.GetOutputPort("measured_foot_pose"), builder)
pose_logger.set_name("pose_logger")

leg_pos_logger = LogVectorOutput(station.GetOutputPort("measured_leg_position"), builder)
leg_pos_logger.set_name("leg_pos_logger")

# Build the system diagram
diagram = builder.Build()
diagram.set_name("toplevel_system_diagram")
diagram_context = diagram.CreateDefaultContext()

if show_toplevel_diagram:
    # Show the overall system diagram
    plt.figure()
    plot_system_graphviz(diagram,max_depth=1)
    plt.show()

print("About to simulate the system!")
if simulate:
    # Set default leg position
    station.go_home(diagram, diagram_context, name="Home")

    # # Set starting position for any objects in the scene
    # station.SetManipulandStartPositions(diagram, diagram_context)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(5.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    start = time.time()
    simulator.AdvanceTo(40.0)
    print("Simulation took:", time.time()-start)

    # Plot the pose data of the end effector over time
    plt.figure()
    ax = plt.axes(projection='3d')
    pose_log = pose_logger.FindLog(simulator.get_context())
    x = pose_log.data()[3,:]
    y = pose_log.data()[4,:]
    z = pose_log.data()[5,:]
    # ipdb.set_trace()
    ax.plot3D(x,y,z)
    ax.set_xlim(0, 1)
    ax.set_ylim(-.5, 0.5)
    ax.set_zlim(0, 1)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()

    print("last xyz pose of foot:", x[-1], y[-1], z[-1])

    leg_pos_log = leg_pos_logger.FindLog(simulator.get_context())
    print("Last leg position was: ", leg_pos_log.data()[:,-1])