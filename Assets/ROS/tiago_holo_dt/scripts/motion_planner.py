#!/usr/bin/env python

import rospy
import argparse
import sys
import math
import numpy as np
import copy
import moveit_commander
import tf2_ros

from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3, PoseStamped, Quaternion, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene

from tiago_holo_dt.msg import PlannedAction
from tiago_holo_dt.srv import ActionService, ExecutionService, ActionServiceResponse, ExecutionServiceResponse


class PlanningSceneHandler:

    def __init__(self, limb, ee_link, touch_links):
        self.limb = limb
        self.end_effector_link = ee_link
        self.touch_links = touch_links
        # Get reference to planning_scene interface
        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        self.obstacles = []

        self.attach_object_pub = rospy.Publisher('/attached_collision_object', AttachedCollisionObject, queue_size=10)
        self.get_planning_scene_srv = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)

    # Spawn static obstacles for robot planning scene
    def spawn_static_obstacles(self, obstacles_list):
        for obst in obstacles_list:
            p1 = PoseStamped()
            p1.header.frame_id = "base_footprint"
            p1.pose.position.x = obst[2].x
            p1.pose.position.y = obst[2].y
            p1.pose.position.z = obst[2].z
            self.scene.add_box(obst[0], p1, obst[1])
            self.obstacles.append(obst[0])

    def remove_static_obstacles(self):
        for obstacle in self.obstacles:
            self.scene.remove_world_object(obstacle)
        del self.obstacles[:]

    # Instantiate collision object msg to notify planning scene about the attached object
    def form_attach_collision_object_msg(self, link_name, object_name, size, pose):
        obj = AttachedCollisionObject()
        # The CollisionObject will be attached with a fixed joint to this link
        obj.link_name = link_name
        obj.touch_links = self.touch_links
        #This contains the actual shapes and poses for the CollisionObject
        col_obj = CollisionObject()
        col_obj.id = object_name
        col_obj.header.frame_id = link_name
        col_obj.header.stamp = rospy.Time.now()
        sp = SolidPrimitive()
        sp.type = sp.BOX
        sp.dimensions = [0.0]*3
        sp.dimensions[0] = size.x
        sp.dimensions[1] = size.y
        sp.dimensions[2] = size.z
        col_obj.primitives = [sp]
        col_obj.primitive_poses = [pose]
        # Adds the object to the planning scene. If the object previously existed, it is replaced.
        col_obj.operation = col_obj.ADD
        obj.object = col_obj
        # The weight of the attached object, if known
        obj.weight = 0.0
        return obj

    # Attach object manipulated by the robot
    def attach_tool_to_end_effector(self):
        tool_pose = Pose()
        tool_pose.position.x = 0.17
        tool_size = Vector3(0.05, 0.15, 0.05)
        attach_msg = self.form_attach_collision_object_msg(link_name=self.end_effector_link, object_name='tool', size=tool_size, pose=tool_pose)
        self.attach_object_pub.publish(attach_msg)

    # Instantiate collision object msg to notify planning scene about the detached object
    def form_remove_all_attached_msg(self, link_name):
        obj = AttachedCollisionObject()
        obj.link_name = link_name
        col_obj = CollisionObject()
        col_obj.id = "tool"
        col_obj.operation = col_obj.REMOVE
        obj.object = col_obj
        return obj
    
    # Detach object manipulated by the robot
    def detach_tool_from_end_effector(self):
        detach_msg = self.form_remove_all_attached_msg(link_name=self.end_effector_link)
        self.attach_object_pub.publish(detach_msg)
        self.scene.remove_world_object("tool")


class MotionPlanner:

    def __init__(self, limb):
        self.limb = limb
        self.current_action = []

        # Get reference to move_group object
        group_name = "arm_" + self.limb + "_torso"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        # Set some parameters for the planner
        self.move_group.set_max_velocity_scaling_factor(0.9)
        self.move_group.set_max_acceleration_scaling_factor(0.7)
        # Get reference to some useful poses
        self.home_joint_config =  self.move_group.get_current_joint_values()
        self.transport_joint_config = [0.09, -1.07, 1.3, 2.7, 2.0, -1.8, -0.12, 0.38]
        # Utility variable for handover round and return trips
        self.round_trip = True

        # Get reference to robot object
        self.robot = moveit_commander.RobotCommander()

        # Instantiate planning Scene Interface object
        ee_link = self.move_group.get_end_effector_link()
        touch_links = self.robot.get_link_names("gripper_{0}".format(self.limb))
        self.planning_scene = PlanningSceneHandler(self.limb, ee_link, touch_links)
        
        # Define orientations for grasping objects
        self.vertical_or = Quaternion(-0.5, -0.5, 0.5, -0.5)
        self.horizontal_or = Quaternion(math.sqrt(2)/2, 0, 0, math.sqrt(2)/2)

        # Define publisher for controlling gripper
        self.gripper_command_pub = rospy.Publisher("/gripper_{0}_controller/command".format(self.limb), JointTrajectory, queue_size=10)

        # Offsets for approaching grasping position and retreating
        self.gripper_offset = 0.25 # Measured distance from actual grasping frame to move_group's end-effector frame
        self.grasp_offset = 0.1

    # Utility functions to perform operation between ROS points
    def to_np(self, point):
        return np.array([point.x, point.y, point.z])

    def to_point(self, array):
        p = Point()
        p.x = array[0]
        p.y = array[1]
        p.z = array[2]
        return p
	
    # Functions to open and close Tiago's gripper
    def close_gripper(self):
        close_gripper_srv = rospy.ServiceProxy("/parallel_gripper_{0}_controller/grasp".format(self.limb), Empty)
        close_gripper_srv()

    def open_gripper(self):
        jt_cmd = JointTrajectory()
        jt_cmd.header.stamp = rospy.Time.now()
        jt_cmd.joint_names = ["gripper_{0}_left_finger_joint".format(self.limb), "gripper_{0}_right_finger_joint".format(self.limb)]

        cmd = JointTrajectoryPoint()
        cmd.positions = [0.045, 0.045]
        cmd.time_from_start = rospy.Time(0.25)

        jt_cmd.points = [cmd]
        self.gripper_command_pub.publish(jt_cmd)

    # -------------------
    # PLANNING PRIMITIVES

    # Plan straight line trajectory
    def plan_cartesian_trajectory(self, waypoints, start_joint_angles, robot_state):
        current_joint_state = JointState()
        joint_names = ["torso_lift_joint"]
        for i in range(1,8):
            joint_names.append("arm_" + self.limb + "_" + str(i) + "_joint")
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        if robot_state is None:
            robot_state = RobotState()        
        robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(robot_state)
        self.move_group.set_goal_tolerance(10e-2)

        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        return plan, fraction

    # Plan trajectory to given pose
    def plan_to_pose(self, destination_pose, start_joint_angles, robot_state):
        current_joint_state = JointState()
        joint_names = ["torso_lift_joint"]
        for i in range(1,8):
            joint_names.append("arm_" + self.limb + "_" + str(i) + "_joint")
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        if robot_state is None:
            robot_state = RobotState()        
        robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(robot_state)

        self.move_group.set_pose_target(destination_pose)
        self.move_group.set_goal_tolerance(10e-3)
        self.move_group.set_path_constraints(None)
        plan = self.move_group.plan()

        if not plan:
            print("Valid plan not found from starting config {0} to pose {1}!".format(start_joint_angles, destination_pose))
            return None
        return plan
    
    # Plan joint space trajectory
    def plan_joint_space(self, final_joint_config, start_joint_config, robot_state):
        current_joint_state = JointState()
        joint_names = ["torso_lift_joint"]
        for i in range(1,8):
            joint_names.append("arm_" + self.limb + "_" + str(i) + "_joint")
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_config

        if robot_state is None:
            robot_state = RobotState()        
        robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(robot_state)

        self.move_group.set_joint_value_target(final_joint_config)
        self.move_group.set_goal_tolerance(10e-3)
        self.move_group.set_path_constraints(None)
        plan = self.move_group.plan()

        if not plan:
            print("Valid plan not found from starting config {0} to final config {1}!".format(start_joint_config, final_joint_config))
            return None
        return plan

    # ------------------
    # PLAN COMPLEX ACTIONS USING PRIMITIVES AND ROUTINES (PICK-PLACE, HANDOVER, ...)

    def dispatcher(self, req):
        rospy.loginfo("Received request to plan {0} action for {1} group...".format(req.action_type, self.limb))
        self.current_action = req.action_type
        
        if req.action_type == "pick":
            return self.plan_pick(req)
        elif req.action_type == "place":
            return self.plan_place(req)
        elif req.action_type == "handover":
            return self.plan_handover_action(req)
        else:
            self.round_trip = True
            return None

    # PICK ACTION (GRASP OBJECT AND GO TO TRANSPORT CONFIG)
    def plan_pick(self, req):
        # Add static obstacles in front of the robot
        shelf_1 = Point()
        shelf_1.x = 0.725
        shelf_1.y = 0.0
        shelf_1.z = 0.76

        shelf_2 = Point()
        shelf_2.x = 0.725
        shelf_2.y = 0.0
        shelf_2.z = 1.06

        shelf_side_left = Point()
        shelf_side_left.x = 0.725
        shelf_side_left.y = 0.3
        shelf_side_left.z = 0.85

        shelf_side_right = Point()
        shelf_side_right.x = 0.725
        shelf_side_right.y = -0.3
        shelf_side_right.z = 0.85

        side_wall_center = Point()
        side_wall_center.x = 0
        side_wall_center.y = 1.0
        side_wall_center.z = 1.0
        self.planning_scene.spawn_static_obstacles(
            [
                    ("shelf_1", (0.27, 0.6, 0.03), shelf_1),
                    ("shelf_2", (0.27, 0.6, 0.03), shelf_2),
                    ("shelf_side_left", (0.27, 0.03, 1.7), shelf_side_left),
                    ("shelf_side_right", (0.27, 0.03, 1.7), shelf_side_right), 
                    ("Side_wall", (0.6, 0.2, 2.0), side_wall_center)         
            ]
        )
        rospy.sleep(0.1)

        # Instantiate response msg
        response = ActionServiceResponse()

        # Determine grasping direction from request(vertical or horizontal)
        grasp_dir = Point()
        grasp_dir.x = req.grasp_direction.x
        grasp_dir.y = req.grasp_direction.y
        grasp_dir.z = req.grasp_direction.z
        if grasp_dir.x == 1:
            orientation = self.horizontal_or
        else:
            orientation = self.vertical_or

        # Initial joint configuration
        previous_ending_joint_angles = []
        current_robot_joint_configuration = []
        # If empty (robot is connected), retrieve directly from move_group
        if not req.joint_angles:
            current_robot_joint_configuration = self.move_group.get_current_joint_values()
        # Else, if values are specified (robot not connected), use those from request
        else:
            current_robot_joint_configuration = req.joint_angles

        # Store copy to use for return position
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - move gripper to approach position
        pre_grasp_pose = Pose()
        pre_grasp_pose.position = self.to_point(self.to_np(req.pick_pos) - self.gripper_offset * self.to_np(grasp_dir))
        pre_grasp_pose.orientation = orientation
        pre_grasp_traj = self.plan_to_pose(pre_grasp_pose, current_robot_joint_configuration, None)
        if pre_grasp_traj is not None:
            response.planned_action.pre_grasp_trajectory = pre_grasp_traj
            previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        else:
            return

        # Attach tool to end-effector for keeping track of it in further planning
        #self.planning_scene.attach_tool_to_end_effector()
        #scene_srv_response = self.planning_scene.get_planning_scene_srv(components = PlanningSceneComponents(components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        #robot_state = scene_srv_response.scene.robot_state

        # Grasp - lower gripper so that fingers are on either side of object
        grasp_pose = Pose()
        grasp_pose.position = self.to_point(self.to_np(req.pick_pos) - self.grasp_offset * self.to_np(grasp_dir))
        grasp_pose.orientation = orientation
        (grasp_traj, fraction) = self.plan_cartesian_trajectory([grasp_pose], previous_ending_joint_angles, None)
        if fraction <= 0.99:
            grasp_traj = self.plan_to_pose(grasp_pose, previous_ending_joint_angles, None)

        if grasp_traj is not None:
            response.planned_action.grasp_trajectory = grasp_traj
            previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response

        # Robot has released object
        #self.planning_scene.detach_tool_from_end_effector()

        # Move away from released object
        (return_traj_1, fraction) = self.plan_cartesian_trajectory([pre_grasp_pose], previous_ending_joint_angles, None)
        if return_traj_1 is not None:
            previous_ending_joint_angles = return_traj_1.joint_trajectory.points[-1].positions
            previous_ending_time = return_traj_1.joint_trajectory.points[-1].time_from_start
            previous_ending_duration = rospy.Duration()
            previous_ending_duration.secs = previous_ending_time.secs
            previous_ending_duration.nsecs = previous_ending_time.nsecs
            # Return to home pose
            return_traj_2 = self.plan_joint_space(self.transport_joint_config, previous_ending_joint_angles, None)
            for point in return_traj_2.joint_trajectory.points:
                point.time_from_start += previous_ending_duration
            return_traj_1.joint_trajectory.points.extend(return_traj_2.joint_trajectory.points[1:])
            return_traj_1.multi_dof_joint_trajectory.points.extend(return_traj_2.multi_dof_joint_trajectory.points)
            response.planned_action.return_trajectory = return_traj_1
            
            self.move_group.clear_pose_targets()
            self.planning_scene.remove_static_obstacles()

            # Return successful planning message
            response.planning_result = True
            return response
        else:
            response.planning_result = False
            return response


    # PLACE ACTION (RELEASE OBJECT ON DESTINATION AND RETURN HOME)
    def plan_place(self, req):
        # Add static table obstacle in front of the robot
        table_center = Point()
        table_center.x = 0.9
        table_center.y = 0.0
        table_center.z = 0.375

        box_center = Point()
        box_center.x = 0.7
        box_center.y = 0.0
        box_center.z = 0.82

        self.planning_scene.spawn_static_obstacles(
            [
                ("table", (0.7, 1.0, 0.75), table_center),
                ("box", (0.22, 0.5, 0.15), box_center)
            ]
        )
        rospy.sleep(0.1)

        # Instantiate response msg
        response = ActionServiceResponse()

        # Determine grasping direction from request(vertical or horizontal)
        grasp_dir = Point()
        grasp_dir.x = req.grasp_direction.x
        grasp_dir.y = req.grasp_direction.y
        grasp_dir.z = req.grasp_direction.z
        if grasp_dir.x == 1:
            orientation = self.horizontal_or
        else:
            orientation = self.vertical_or

        # Initial joint configuration
        previous_ending_joint_angles = []
        current_robot_joint_configuration = []
        # If empty (robot is connected), retrieve directly from move_group
        if not req.joint_angles:
            current_robot_joint_configuration = self.move_group.get_current_joint_values()
        # Else, if values are specified (robot not connected), use those from request
        else:
            current_robot_joint_configuration = req.joint_angles

        # Attach tool to end-effector for keeping track of it in further planning
        self.planning_scene.attach_tool_to_end_effector()
        scene_srv_response = self.planning_scene.get_planning_scene_srv(components = PlanningSceneComponents(components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        robot_state = scene_srv_response.scene.robot_state

        # Move towards place pose
        move_pose = Pose()
        move_pose.position = self.to_point(self.to_np(req.place_pos) - self.gripper_offset * self.to_np(grasp_dir))
        move_pose.orientation = orientation
        move_trajectory = self.plan_to_pose(move_pose, current_robot_joint_configuration, robot_state)
        if move_trajectory is not None:
            response.planned_action.move_trajectory = move_trajectory
            previous_ending_joint_angles = move_trajectory.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response
        
        # Place - Descend and leave object in the desired position
        place_pose = Pose()
        place_pose.position = self.to_point(self.to_np(req.place_pos) - self.grasp_offset * self.to_np(grasp_dir)) 
        place_pose.orientation = orientation
        (place_traj, fraction) = self.plan_cartesian_trajectory([place_pose], previous_ending_joint_angles, robot_state)
        if fraction <= 0.99:
            place_traj = self.plan_to_pose(place_pose, previous_ending_joint_angles, robot_state)

        if place_traj is not None:
            response.planned_action.place_trajectory = place_traj
            previous_ending_joint_angles = place_traj.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response


        # return - Go back to home position
        return_traj = self.plan_joint_space(self.home_joint_config, previous_ending_joint_angles, robot_state)

        self.move_group.clear_pose_targets()
        self.planning_scene.remove_static_obstacles()
        self.planning_scene.detach_tool_from_end_effector()

        if return_traj is not None:
            response.planned_action.return_trajectory = return_traj
            # Return successful planning message
            response.planning_result = True
            return response
        else:
            response.planning_result = False
            return response


    # HANDOVER ACTION
    def plan_handover_action(self, req):
        # Instantiate response msg
        response = ActionServiceResponse()

        # Determine grasping direction from request(vertical or horizontal)
        grasp_dir = Point()
        grasp_dir.x = req.grasp_direction.x
        grasp_dir.y = req.grasp_direction.y
        grasp_dir.z = req.grasp_direction.z
        if grasp_dir.x == 1:
            orientation = self.horizontal_or
        else:
            orientation = self.vertical_or

        # Initial joint configuration
        previous_ending_joint_angles = []
        current_robot_joint_configuration = []
        # If empty (robot is connected), retrieve directly from move_group
        if not req.joint_angles:
            current_robot_joint_configuration = self.move_group.get_current_joint_values()
        # Else, if values are specified (robot not connected), use those from request
        else:
            current_robot_joint_configuration = req.joint_angles

        # Pre grasp - move gripper to approach position
        pre_grasp_pose = Pose()
        pre_grasp_pose.position = req.pick_pos
        pre_grasp_pose.orientation = orientation
        (pre_grasp_traj, fraction) = self.plan_cartesian_trajectory([pre_grasp_pose], current_robot_joint_configuration, None)
        if pre_grasp_traj is not None:
            response.planned_action.pre_grasp_trajectory = pre_grasp_traj
            previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response

        # Return to home position with object in hand (plan for future execution)
        return_traj = self.plan_joint_space(self.transport_joint_config, previous_ending_joint_angles, None)
        if return_traj is not None:
            # Return successful planning message
            response.planned_action.return_trajectory = return_traj
            response.planning_result = True
            return response
        else:
            response.planning_result = False
            return response

    '''
    # PICK-PLACE ACTION
    def plan_pick_and_place_action(self, req):

        # Add static obstacle in front of the robot
        obst_center = Point()
        obst_center.x = 0.8
        obst_center.y = 0
        obst_center.z = 0.375
        self.planning_scene.spawn_static_obstacles([("table", (0.7, 1.0, 0.75), obst_center)])
        rospy.sleep(0.1)

        # Instantiate response msg
        response = ActionServiceResponse()

        # Determine grasping direction from request(vertical or horizontal)
        grasp_dir = Point()
        grasp_dir.x = req.grasp_direction.x
        grasp_dir.y = req.grasp_direction.y
        grasp_dir.z = req.grasp_direction.z
        if grasp_dir.x == 1:
            orientation = self.horizontal_or
        else:
            orientation = self.vertical_or

        # Initial joint configuration
        previous_ending_joint_angles = []
        current_robot_joint_configuration = []
        # If empty (robot is connected), retrieve directly from move_group
        if not req.joint_angles:
            current_robot_joint_configuration = self.move_group.get_current_joint_values()
        # Else, if values are specified (robot not connected), use those from request
        else:
            current_robot_joint_configuration = req.joint_angles

        # Store copy to use for return position
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - move gripper to approach position
        pre_grasp_pose = Pose()
        pre_grasp_pose.position = self.to_point(self.to_np(req.pick_pos) - self.gripper_offset * self.to_np(grasp_dir))
        pre_grasp_pose.orientation = orientation
        pre_grasp_traj = self.plan_to_pose(pre_grasp_pose, current_robot_joint_configuration, None)
        if pre_grasp_traj is not None:
            response.planned_action.pre_grasp_trajectory = pre_grasp_traj
            previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response

        # Attach tool to end-effector for keeping track of it in further planning
        self.planning_scene.attach_tool_to_end_effector()
        #scene_srv_response = self.planning_scene.get_planning_scene_srv(components = PlanningSceneComponents(components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        #robot_state = scene_srv_response.scene.robot_state

        # Grasp - lower gripper so that fingers are on either side of object
        grasp_pose = Pose()
        grasp_pose.position = self.to_point(self.to_np(req.pick_pos) - self.grasp_offset * self.to_np(grasp_dir))
        grasp_pose.orientation = orientation
        (grasp_traj, fraction) = self.plan_cartesian_trajectory([grasp_pose], previous_ending_joint_angles, None)
        if fraction <= 0.99:
            grasp_traj = self.plan_to_pose(grasp_pose, previous_ending_joint_angles, None)

        if grasp_traj is not None:
            response.planned_action.grasp_trajectory = grasp_traj
            previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response
        
        # Move towards place pose
        move_pose = Pose()
        move_pose.position = self.to_point(self.to_np(req.place_pos) - self.gripper_offset * self.to_np(grasp_dir))
        move_pose.orientation = orientation
        move_trajectory = self.plan_to_pose(move_pose, previous_ending_joint_angles, None)
        if move_trajectory is not None:
            response.planned_action.move_trajectory = move_trajectory
            previous_ending_joint_angles = move_trajectory.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response
        
        # Place - Descend and leave object in the desired position
        place_pose = Pose()
        place_pose.position = self.to_point(self.to_np(req.place_pos) - self.grasp_offset * self.to_np(grasp_dir)) 
        place_pose.orientation = orientation
        (place_traj, fraction) = self.plan_cartesian_trajectory([place_pose], previous_ending_joint_angles, None)
        if fraction <= 0.99:
            place_traj = self.plan_to_pose(place_pose, previous_ending_joint_angles, None)

        if place_traj is not None:
            response.planned_action.place_trajectory = place_traj
            previous_ending_joint_angles = place_traj.joint_trajectory.points[-1].positions
        else:
            response.planning_result = False
            return response
        
        # Robot has released object
        self.planning_scene.detach_tool_from_end_effector()

        # Move away from released object
        (return_traj_1, fraction) = self.plan_cartesian_trajectory([move_pose], previous_ending_joint_angles, None)
        if return_traj_1 is not None:
            previous_ending_joint_angles = return_traj_1.joint_trajectory.points[-1].positions
            previous_ending_time = return_traj_1.joint_trajectory.points[-1].time_from_start
            previous_ending_duration = rospy.Duration()
            previous_ending_duration.secs = previous_ending_time.secs
            previous_ending_duration.nsecs = previous_ending_time.nsecs
            # Return to home pose
            return_traj_2 = self.plan_joint_space(initial_joint_configuration, previous_ending_joint_angles, None)
            for point in return_traj_2.joint_trajectory.points:
                point.time_from_start += previous_ending_duration
            return_traj_1.joint_trajectory.points.extend(return_traj_2.joint_trajectory.points[1:])
            return_traj_1.multi_dof_joint_trajectory.points.extend(return_traj_2.multi_dof_joint_trajectory.points)
            response.planned_action.return_trajectory = return_traj_1
            
            self.move_group.clear_pose_targets()
            self.planning_scene.remove_static_obstacles()

            # Return successful planning message
            response.planning_result = True
            return response
        else:
            response.planning_result = False
            return response
    '''

    # ------------------------
    # METHOD TO EXECUTE PLANNED ACTIONS

    def execute_action(self, response):
        # If planning successful, execute according to action type
        if response.planning_result:
            rospy.loginfo("Executing {0} action...".format(self.current_action))

            # Perform pre-grasp and grasp motions only if not place action
            if (self.current_action == "pick") or (self.current_action == "handover" and self.round_trip):
                self.open_gripper()
                self.move_group.execute(response.planned_action.pre_grasp_trajectory, wait=True)
                self.move_group.execute(response.planned_action.grasp_trajectory, wait=True)

            # Grasp object only if pick action is required
            if self.current_action == "pick":
                self.close_gripper()

            # Move to other destination and release object if place action required
            if self.current_action == "place":
                self.move_group.execute(response.planned_action.move_trajectory, wait=True)
                self.move_group.execute(response.planned_action.place_trajectory, wait=True)
                self.open_gripper()
                rospy.sleep(1.0)

            # Always get back to home pose (or to transport pose if object is grasped), unless handover, which needs to be triggered
            if self.current_action != "handover":
                self.move_group.execute(response.planned_action.return_trajectory, wait=True)
            else:
                if self.round_trip:
                    self.round_trip = False
                else:
                    self.close_gripper()
                    self.move_group.execute(response.planned_action.return_trajectory, wait=True)
                    self.round_trip = True

            result = ExecutionServiceResponse()
            result.execution_result = True
            rospy.loginfo("Action complete!")
            return result
  

# ------------------------------
# MAIN

def main():
    # Initialize node
    rospy.init_node('motion_planner_service_node', log_level=rospy.INFO, anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv) 

    # Parse argument from launch file
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, type=str,
        help='limb parameter [left, right]'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Parsed arguments
    limb = args.limb
    
    # Define motion planner object with limb argument parsed
    motion_planner = MotionPlanner(limb)
    rospy.Service('/' + limb + '_group/plan_action', ActionService, motion_planner.dispatcher)
    rospy.Service("/" + limb + '_group/execute_action', ExecutionService, motion_planner.execute_action)
    
    rospy.loginfo("Ready to plan for {0} group...".format(limb))
    rospy.spin()



if __name__ == "__main__":
    main()

