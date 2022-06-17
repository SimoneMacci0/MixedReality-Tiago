#!/usr/bin/env python

from __future__ import print_function

import rospy
import argparse
import sys
import copy
import math
import moveit_commander

from std_msgs.msg import String
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Pose, PoseStamped

from tiago_unity.srv import PickPlaceService, PickPlaceServiceRequest, PickPlaceServiceResponse


class MotionPlanner:

    def __init__(self, limb, offset):
        self.limb = limb
        self.height_offset = offset

        group_name = "arm_" + self.limb
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        
    # Plan straight line trajectory
    def plan_cartesian_trajectory(self, destination_pose, start_joint_angles):
        current_joint_state = JointState()
	joint_names = ["arm_" + self.limb + "_" + i + "_joint" for i in range(1,8)]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_goal_tolerance(10e-3)
        (plan, fraction) = self.move_group.compute_cartesian_path([destination_pose], 0.05, 0.0)

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            exit(1)

        return plan

    # Plan trajectory to given pose
    def plan_to_pose(self, destination_pose, start_joint_angles):
        current_joint_state = JointState()
        joint_names = ["arm_" + self.limb + "_" + i + "_joint" for i in range(1,8)]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_angles

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_pose_target(destination_pose)
        self.move_group.set_goal_tolerance(10e-3)
        plan = self.move_group.plan()

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            exit(1)

        return plan[1]
    
    # Plan joint space trajectory
    def plan_return_to_home(self, final_joint_config, start_joint_config):
        current_joint_state = JointState()
        joint_names = ["arm_" + self.limb + "_" + i + "_joint" for i in range(1,8)]
        current_joint_state.name = joint_names
        current_joint_state.position = start_joint_config

        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = current_joint_state
        self.move_group.set_start_state(moveit_robot_state)
        self.move_group.set_joint_value_target(final_joint_config)
        self.move_group.set_goal_tolerance(10e-3)
        
        plan = self.move_group.plan()

        if not plan:
            exception_str = """
                Trajectory could not be planned for a destination of {} with starting joint angles {}.
                Please make sure target and destination are reachable by the robot.
            """.format(destination_pose, destination_pose)
            exit(1)

        return plan[1]

    # Plan pick and place action
    def pick_and_place(self, req):
        
        response = PickPlaceServiceResponse()
        response.arm_trajectory.arm = self.limb
        
        # Initial joint configuration
        current_robot_joint_configuration = [math.radians(req.joints_angles[i]) for i in range(7)]
        initial_joint_configuration = copy.deepcopy(current_robot_joint_configuration)

        # Pre grasp - position gripper directly above target object
        pre_grasp_traj = self.plan_cartesian_trajectory(req.pick_pose, current_robot_joint_configuration)

        previous_ending_joint_angles = pre_grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(pre_grasp_traj)

        # Grasp - lower gripper so that fingers are on either side of object
        pick_pose = copy.deepcopy(req.pick_pose)
        pick_pose.position.z -= self.height_offset
        grasp_traj = self.plan_cartesian_trajectory(pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = grasp_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(grasp_traj)

        # Pick Up - raise gripper back to the pre grasp position
        pick_up_traj = self.plan_cartesian_trajectory(req.pick_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = pick_up_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(pick_up_traj)

        # Move gripper to desired placement position
        move_traj = self.plan_cartesian_trajectory(req.place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = move_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(move_traj)

        # Place - Descend and leave object in the desired position
        place_pose = copy.deepcopy(req.place_pose)
        place_pose.position.z -= self.height_offset*0.8
        place_traj = self.plan_cartesian_trajectory(place_pose, previous_ending_joint_angles)

        previous_ending_joint_angles = place_traj.joint_trajectory.points[-1].positions
        response.arm_trajectory.trajectory.append(place_traj)
            
        # Return to home pose
        return_home_traj = self.plan_return_to_home(initial_joint_configuration, previous_ending_joint_angles)
        response.arm_trajectory.trajectory.append(return_home_traj)

        self.move_group.clear_pose_targets()
        
        return response
  
    
def main():
    # Initialize node
    rospy.init_node('motion_planner_service_node')
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
    required.add_argument(
        '-o', '--offset', required=True, type=float,
        help='height offset value'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Parsed arguments
    limb = args.limb
    offset = args.offset

    # Define motion planner object with argument parsed
    motion_planner = MotionPlanner(limb, offset)
    s = rospy.Service('/' + limb + '_group/tiago_unity_motion_planner', PickPlaceService, motion_planner.pick_and_place)
        
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    main()

