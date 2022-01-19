from pydrake.all import *
# from kinova_station.common import (EndEffectorTarget, 
#                                    GripperTarget, 
#                                    EndEffectorWrenchCalculator,
#                                    CameraPosePublisher)
import os
package_dir = os.path.dirname(os.path.abspath(__file__))

class MuadQuadStation(Diagram):
    """
    A template system diagram for controlling a simple double pendulum leg, modeled 
    after Drake's ManipulationStation, but with a leg instead of a kuka arm.
   
                               ---------------------------------
                               |                               |
                               |                               |
                               |                               |
                               |                               | --> measured_leg_position
                               |                               | --> measured_leg_velocity
    foot_target -------------> |        MuadQuadStation        | --> measured_leg_torque
                               |                               |
                               |                               |
                               |                               | --> measured_foot_pose
                               |                               | --> measured_foot_twist
                               |                               | --> measured_foot_wrench
                               |                               | 
                               |                               | 
                               |                               | 
                               |                               | 
                               |                               |
                               |                               | 
                               |                               | 
                               |                               | 
                               |                               |
                               |                               |
                               ---------------------------------
    """
    def __init__(self, time_step=0.002):
        Diagram.__init__(self)
        self.set_name("muadquad_station")

        self.builder = DiagramBuilder()

        self.scene_graph = self.builder.AddSystem(SceneGraph())
        self.scene_graph.set_name("scene_graph")

        self.plant = self.builder.AddSystem(MultibodyPlant(time_step=time_step))
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.plant.set_name("plant")

        # A separate plant which only has access to the robot arm + gripper mass,
        # and not any other objects in the scene
        self.controller_plant = MultibodyPlant(time_step=time_step)

        # Body id's and poses for any extra objects in the scene
        self.object_ids = []
        self.object_poses = []


    def Finalize(self):
        """
        Do some final setup stuff. Must be called after making modifications
        to the station (e.g. adding arm, gripper, manipulands) and before using
        this diagram as a system. 
        """

        self.plant.Finalize()
        self.controller_plant.Finalize()
        
        # Set up the scene graph
        self.builder.Connect(
                self.scene_graph.get_query_output_port(),
                self.plant.get_geometry_query_input_port())
        self.builder.Connect(
                self.plant.get_geometry_poses_output_port(),
                self.scene_graph.get_source_pose_port(self.plant.get_source_id()))

        # Create controller
        leg_controller = self.builder.AddSystem(LegController(
                                        self.controller_plant,
                                        self.controller_leg))
        leg_controller.set_name("leg_controller")

        # End effector target and target type go to the controller
        self.builder.ExportInput(leg_controller.foot_target_port,
                                 "foot_target")

        # Output measured arm position and velocity
        demux = self.builder.AddSystem(Demultiplexer(
                                        self.plant.num_multibody_states(self.leg),
                                        self.plant.num_positions(self.leg)))
        demux.set_name("demux")
        self.builder.Connect(
                self.plant.get_state_output_port(self.leg),
                demux.get_input_port(0))
        self.builder.ExportOutput(
                demux.get_output_port(0),
                "measured_leg_position")
        self.builder.ExportOutput(
                demux.get_output_port(1),
                "measured_leg_velocity")
        
        # Measured arm position and velocity are sent to the controller
        self.builder.Connect(
                demux.get_output_port(0),
                leg_controller.leg_position_port)
        self.builder.Connect(
                demux.get_output_port(1),
                leg_controller.leg_velocity_port)

        # Torques from controller go to the simulated plant
        self.builder.Connect(
                leg_controller.GetOutputPort("applied_leg_torque"),
                self.plant.get_actuation_input_port(self.leg))

        # Controller outputs measured arm torques, end-effector pose, end-effector twist
        self.builder.ExportOutput(
                leg_controller.GetOutputPort("applied_leg_torque"),
                "measured_leg_torque")
        self.builder.ExportOutput(
                leg_controller.GetOutputPort("measured_foot_pose"),
                "measured_foot_pose")
        self.builder.ExportOutput(
                leg_controller.GetOutputPort("measured_foot_twist"),
                "measured_foot_twist")
        
        # Compute and output end-effector wrenches based on measured joint torques
        # wrench_calculator = self.builder.AddSystem(FootWrenchCalculator(
        #         self.controller_plant,
        #         self.controller_plant.GetFrameByName("foot")))
        # wrench_calculator.set_name("wrench_calculator")

        # self.builder.Connect(
        #         demux.get_output_port(0),
        #         wrench_calculator.GetInputPort("joint_positions"))
        # self.builder.Connect(
        #         demux.get_output_port(1),
        #         wrench_calculator.GetInputPort("joint_velocities"))
        # self.builder.Connect(
        #         leg_controller.GetOutputPort("applied_leg_torque"),
        #         wrench_calculator.GetInputPort("joint_torques"))

        # self.builder.ExportOutput(
        #         wrench_calculator.get_output_port(),
        #         "measured_foot_wrench")

        # Build the diagram
        self.builder.BuildInto(self)

    def SetupSingleLegScenario(self, peg_position=[0.8,0,0.1], peg_rotation=[0,np.pi/2,0]):
        """
        Set up a scenario with the robot arm, a gripper, and a single peg. 
        And connect to the Drake visualizer while we're at it.
        """
        self.AddGround()
        self.AddSimpleLeg()

        self.ConnectToDrakeVisualizer()

    def AddGround(self):
        """
        Add a flat ground with friction
        """
        X_BG = RigidTransform()
        hfspce = HalfSpace()
        surface_friction = CoulombFriction(
                static_friction = 0.7,
                dynamic_friction = 0.5)
        self.plant.RegisterCollisionGeometry(
                self.plant.world_body(),
                X_BG,
                hfspce,
                "ground_collision",
                surface_friction)
        self.plant.RegisterVisualGeometry(
                self.plant.world_body(),
                X_BG,
                hfspce,
                "ground_visual",
                np.array([0.5,0.5,0.5,0]))  # transparent

    def AddSimpleLeg(self, include_damping=False):
        """
        Add simple leg to the system.
        """
        leg_urdf = package_dir + "/../models/urdf/leg.urdf"

        self.leg = Parser(plant=self.plant).AddModelFromFile(leg_urdf, "leg")
        self.controller_leg = Parser(plant=self.controller_plant).AddModelFromFile(leg_urdf, "leg")

        # Fix the base of the arm to the world
        self.plant.WeldFrames(self.plant.world_frame(),
                              self.plant.GetFrameByName("base_link",self.leg))

        self.controller_plant.WeldFrames(self.controller_plant.world_frame(),
                                         self.controller_plant.GetFrameByName("base_link", self.controller_leg))

        # Create a new frame with the actual end-effector position.
        self.X_foot = RigidTransform()
        self.X_foot.set_translation([0,0,0])
        self.plant.AddFrame(FixedOffsetFrame(
                        "foot",
                        self.plant.GetFrameByName("foot_link"),
                        self.X_foot, self.leg))
        self.controller_plant.AddFrame(FixedOffsetFrame(
                        "foot",
                        self.controller_plant.GetFrameByName("foot_link"),
                        self.X_foot, self.controller_leg))


    def SetArmPositions(self, diagram, diagram_context, q):
        """
        Set arm positions to the given values. Must be called after the overall
        system diagram is built, and the associated diagram_context set. 
        """
        plant_context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        self.plant.SetPositions(plant_context, self.leg, q)

    def SetArmVelocities(self, diagram, diagram_context, qd):
        """
        Set arm velocities to the given values. Must be called after the overall
        system diagram is built, and the associated diagram_context set. 
        """
        plant_context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        self.plant.SetVelocities(plant_context, self.leg, qd)

    # def SetManipulandStartPositions(self, diagram, diagram_context):
    #     """
    #     Set positions of any manipulands to their starting values. Must be called
    #     after the overall system diagram is built, and the associated diagram_context set.
    #     """
    #     assert len(self.object_ids) == len(self.object_poses), "Manipuland poses and ids don't match"

    #     plant_context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        
    #     for i in range(len(self.object_ids)):
    #         self.plant.SetFreeBodyPose(plant_context, 
    #                                    self.plant.get_body(self.object_ids[i]),
    #                                    self.object_poses[i])


    def ConnectToDrakeVisualizer(self):
        visualizer_params = DrakeVisualizerParams(role=Role.kIllustration)
        DrakeVisualizer().AddToBuilder(builder=self.builder,
                                       scene_graph=self.scene_graph,
                                       params=visualizer_params)

    def ConnectToMeshcatVisualizer(self, zmq_url=None):
        if zmq_url is None:
            # Start meshcat server. This saves the step of opening meshcat separately,
            # but does mean you need to refresh the page each time you re-run something.
            from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
            proc, zmq_url, web_url = start_zmq_server_as_subprocess()

        # Defining self.meshcat in this way allows us to connect to 
        # things like a point-cloud visualizer later
        self.meshcat = ConnectMeshcatVisualizer(builder=self.builder,
                                 zmq_url = zmq_url,
                                 scene_graph=self.scene_graph,
                                 output_port=self.scene_graph.get_query_output_port())

    def go_home(self, diagram, diagram_context, name="Home"):
        """
        Move the arm to the specified home position. Must be one of
        'Home', 'Retract', or 'Zero'.
        """
        q0 = np.array([0, 0])
        # qd0 = np.array([-np.pi/8,-np.pi/8])
        self.SetArmPositions(diagram, diagram_context, q0)
        # self.SetArmVelocities(diagram, diagram_context, qd0)


class LegController(LeafSystem):
    """
    A controller which controls the position of a double pendulum.
 
                         -------------------------
                         |                       |
                         |                       |
    foot_target -------> |     LegController     | ----> applied_leg_torque
                         |                       | 
                         |                       | 
                         |                       | ----> measured_foot_pose
    leg_position ------> |                       | ----> measured_foot_twist
    leg_velocity ------> |                       | 
                         |                       | 
                         |                       | 
                         |                       |
                         -------------------------

    The foot target is a positional target until further notice
    """
    def __init__(self, plant, leg_model):
        LeafSystem.__init__(self)

        self.plant = plant
        self.leg = leg_model
        self.context = self.plant.CreateDefaultContext()

        # Define input ports
        self.foot_target_port = self.DeclareVectorInputPort(
                                        "foot_target",
                                        BasicVector(6))

        self.leg_position_port = self.DeclareVectorInputPort(
                                        "leg_position",
                                        BasicVector(self.plant.num_positions(self.leg)))

        self.leg_velocity_port = self.DeclareVectorInputPort(
                                        "leg_velocity",
                                        BasicVector(self.plant.num_velocities(self.leg)))

        # Define output ports
        self.DeclareVectorOutputPort(
                "applied_leg_torque",
                BasicVector(self.plant.num_actuators()),
                self.CalcLegTorques)

        self.DeclareVectorOutputPort(
                "measured_foot_pose",
                BasicVector(6),
                self.CalcFootPose,
                {self.time_ticket()}   # indicate that this doesn't depend on any inputs,
                )                      # but should still be updated each timestep
        self.DeclareVectorOutputPort(
                "measured_foot_twist",
                BasicVector(6),
                self.CalcFootTwist,
                {self.time_ticket()})

        # Define some relevant frames
        self.world_frame = self.plant.world_frame()
        self.foot_frame = self.plant.GetFrameByName("foot")

        # Set joint limits (set self.{q,qd}_{min,max})
        self.GetJointLimits()
           
        # Store desired foot pose and corresponding joint
        # angles so we only run full IK when we need to
        self.last_foot_pose_target = None
        self.last_q_target = None
    
    def GetJointLimits(self):
        """
        Iterate through self.plant to establish joint angle
        and velocity limits. 

        Sets:

            self.q_min
            self.q_max
            self.qd_min
            self.qd_max

        """
        # @MICHAEL TODO: This whole function might need hardcoded limits for now. Or figure out where limits are set.
        q_min = []
        q_max = []
        qd_min = []
        qd_max = []

        joint_indices = self.plant.GetJointIndices(self.leg)

        for idx in joint_indices:
            joint = self.plant.get_joint(idx)
            
            if joint.type_name() == "revolute":  # ignore the joint welded to the world
                q_min.append(joint.position_lower_limit())
                q_max.append(joint.position_upper_limit())
                qd_min.append(joint.velocity_lower_limit())  # note that higher limits
                qd_max.append(joint.velocity_upper_limit())  # are availible in cartesian mode

        self.q_min = np.array(q_min)
        self.q_max = np.array(q_max)
        self.qd_min = np.array(qd_min)
        self.qd_max = np.array(qd_max)

    def CalcFootPose(self, context, output):
        """
        This method is called each timestep to determine the foot pose
        """
        q = self.leg_position_port.Eval(context)
        qd = self.leg_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Compute the rigid transform between the world and end-effector frames
        X_foot = self.plant.CalcRelativeTransform(self.context,
                                                  self.world_frame,
                                                  self.foot_frame)

        foot_pose = np.hstack([RollPitchYaw(X_foot.rotation()).vector(), X_foot.translation()])
        # print(X_foot.translation())

        output.SetFromVector(foot_pose)
    
    def CalcFootTwist(self, context, output):
        """
        This method is called each timestep to determine the foot twist
        """
        q = self.leg_position_port.Eval(context)
        qd = self.leg_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Compute end-effector Jacobian
        J = self.plant.CalcJacobianSpatialVelocity(self.context,
                                                   JacobianWrtVariable.kV,
                                                   self.foot_frame,
                                                   np.zeros(3),
                                                   self.world_frame,
                                                   self.world_frame)

        foot_twist = J@qd
        output.SetFromVector(foot_twist)
    
    def CalcLegTorques(self, context, output):
        """
        This method is called each timestep to determine output torques
        """
        q = self.leg_position_port.Eval(context)
        qd = self.leg_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Some dynamics computations
        tau_g = -self.plant.CalcGravityGeneralizedForces(self.context)

        # # Indicate what type of command we're recieving
        # FOR NOW, WE ARE ALWAYS RECEIVING A POSITION COMMAND
        # target_type = self.ee_target_type_port.Eval(context)






        
        # Compute joint torques which move the end effector to the desired pose
        rpy_xyz_des = self.foot_target_port.Eval(context)

        # Only do a full inverse kinematics solve if the target pose has changed
        if (rpy_xyz_des != self.last_foot_pose_target).any():
            
            # @MICHAEL TODO: This looks sus for our small example. May have to make this smaller
            X_WE_des = RigidTransform(RollPitchYaw(rpy_xyz_des[:3]),
                                        rpy_xyz_des[-3:])         

            # print("desired translation:",X_WE_des.translation())
            # print("desired rotation:",X_WE_des.rotation())
            # First compute joint angles consistent with the desired pose using Drake's IK.
            # This sets up a nonconvex optimization problem to find joint angles consistent
            # with the given constraints
            ik = InverseKinematics(self.plant,self.context)
            ik.AddPositionConstraint(self.foot_frame,
                                        [0,0,0],
                                        self.world_frame,
                                        [i - 0.01 for i in X_WE_des.translation()], 
                                        [i + 0.01 for i in X_WE_des.translation()])
        #     @MICHAEL TODO: add back in later
        #     ik.AddOrientationConstraint(self.foot_frame,
        #                                 RotationMatrix(),
        #                                 self.world_frame,
        #                                 X_WE_des.rotation(),
        #                                 0.001)

            prog = ik.get_mutable_prog()
            q_var = ik.q()
            prog.AddQuadraticErrorCost(np.eye(len(q_var)), q, q_var)
            prog.SetInitialGuess(q_var, q)
            result = Solve(ik.prog())

            if not result.is_success():
                print("Inverse Kinematics Failed!")
                q_nom = np.zeros(self.plant.num_positions())
            else:
                q_nom = result.GetSolution(q_var)
                print(q_nom)
                # q_nom = np.zeros(self.plant.num_positions()) #DELETE TODO
                # Save the results of this solve for later
                self.last_foot_pose_target = rpy_xyz_des
                self.last_q_target = q_nom

        else:
            q_nom = self.last_q_target

        qd_nom = np.zeros(self.plant.num_velocities())

        # Use PD controller to map desired q, qd to desired qdd
        Kp = 100*np.eye(self.plant.num_positions())
        Kd = 2*np.sqrt(Kp)  # critical damping
        qdd_nom = Kp@(q_nom - q) + Kd@(qd_nom - qd)

        # Compute joint torques consistent with these desired qdd
        f_ext = MultibodyForces(self.plant)
        tau = tau_g + self.plant.CalcInverseDynamics(self.context, qdd_nom, f_ext)

        # else:
        #     raise RuntimeError("Invalid target type %s" % target_type)



        # target_actuator_angles = self.foot_target_port.Eval(context)
        # Kp = 10
        # error = target_actuator_angles - q
        # f_ext = MultibodyForces(self.plant)



        # # Compute joint torques consistent with the desired twist
        # twist_des = self.foot_target_port.Eval(context)

        # # Use DoDifferentialInverseKinematics to determine desired qd
        # params = DifferentialInverseKinematicsParameters(self.plant.num_positions(),
        #                                                 self.plant.num_velocities())
        # params.set_timestep(0.005)
        # params.set_joint_velocity_limits((self.qd_min, self.qd_max))
        # params.set_joint_position_limits((self.q_min, self.q_max))

        # result = DoDifferentialInverseKinematics(self.plant,
        #                                         self.context,
        #                                         twist_des,
        #                                         self.foot_frame,
        #                                         params)

        # if result.status == DifferentialInverseKinematicsStatus.kSolutionFound:
        #         qd_nom = result.joint_velocities
        # else:
        #         print("Differential inverse kinematics failed!")
        #         qd_nom = np.zeros(self.plant.num_velocities())

        # # Select desired accelerations using a proportional controller
        # Kp = 10*np.eye(self.plant.num_velocities())
        # qdd_nom = Kp@(qd_nom - qd)
        # print("qdd_nom is:", qdd_nom)

        # # Compute joint torques consistent with these desired accelerations
        # f_ext = MultibodyForces(self.plant)
        # print("f_ext are:", f_ext.generalized_forces())
        # tau = tau_g + self.plant.CalcInverseDynamics(self.context, qdd_nom, f_ext)

        # print(tau)
        # tau = np.zeros(2)
        # tau[0] = -1
        # tau[1] = -1




        output.SetFromVector(tau)


class FootWrenchCalculator(LeafSystem):
    """
    A simple system which takes as input joint torques and outputs the corresponding
    wrench applied to the end-effector. 

                       ---------------------------------
                       |                               |
                       |                               |
                       |                               |
    joint_positions -> |     FootWrenchCalculator      | ---> foot_wrench
    joint_angles ----> |                               | 
    joint_torques ---> |                               |
                       |                               |
                       |                               |
                       ---------------------------------
    """
    def __init__(self, plant, foot_frame):
        LeafSystem.__init__(self)

        self.plant = plant
        self.context = self.plant.CreateDefaultContext()
        self.foot_frame = foot_frame

        # Inputs are joint positions, angles and torques
        self.q_port = self.DeclareVectorInputPort(
                                "joint_positions",
                                BasicVector(plant.num_positions()))
        self.v_port = self.DeclareVectorInputPort(
                                "joint_velocities",
                                BasicVector(plant.num_velocities()))
        self.tau_port = self.DeclareVectorInputPort(
                                "joint_torques",
                                BasicVector(plant.num_actuators()))

        # Output is applied wrench at the end-effector
        self.DeclareVectorOutputPort(
                "foot_wrench",
                BasicVector(6),
                self.CalcFootWrench)

    def CalcFootWrench(self, context, output):
        # Gather inputs
        q = self.q_port.Eval(context)
        v = self.v_port.Eval(context)
        tau = self.tau_port.Eval(context)

        # Set internal model state
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, v)

        # Some dynamics computations
        M = self.plant.CalcMassMatrixViaInverseDynamics(self.context)
        tau_g = -self.plant.CalcGravityGeneralizedForces(self.context)

        # Compute foot jacobian
        J = self.plant.CalcJacobianSpatialVelocity(self.context,
                                                   JacobianWrtVariable.kV,
                                                   self.foot_frame,
                                                   np.zeros(3),
                                                   self.plant.world_frame(),
                                                   self.plant.world_frame())

        print("Mass Matrix:", M)
        print("tau_g:", tau_g)
        print("Foot Jacobian",J)

        # Compute jacobian pseudoinverse
        Minv = np.linalg.inv(M)
        Lambda = np.linalg.inv(J@Minv@J.T)
        Jbar = Lambda@J@Minv

        # Compute wrench (spatial force) applied at foot
        w = Jbar@(tau-tau_g)
        # print(w)
        # w = np.zeros(6)

        output.SetFromVector(w)



