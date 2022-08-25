#!/usr/bin/env python

import math
import rospy
import sys
from random import Random

import tf2_ros
import tf2_geometry_msgs
import actionlib

from geometry_msgs.msg import Point, PointStamped, Vector3, PoseStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path

from tiago_holo_dt.srv import ActionService, ExecutionService, PoseService
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse


# Utility class for handling robot's navigation
class NavHandler:

    def __init__(self):
        # Pose in front of the table, to drop off items needed for the assembly 
        posT = Pose()
        posT.position.x = -0.11
        posT.position.y = -0.74
        posT.position.z = 0.0
        posT.orientation.x = 0.0
        posT.orientation.y = 0.0
        posT.orientation.z = -0.75
        posT.orientation.w = 0.65
        
        # Poses in front of the two shelves, from where the robot can grasp objects
        posS1 = Pose()
        posS1.position.x = 0.61
        posS1.position.y = 0.66
        posS1.position.z = 0.0
        posS1.orientation.x = 0.0
        posS1.orientation.y = 0.0
        posS1.orientation.z = 0.64
        posS1.orientation.w = 0.76

        posS2 = Pose()
        posS2.position.x = 0.82
        posS2.position.y = 0.48
        posS2.position.z = 0.0
        posS2.orientation.x = 0.0
        posS2.orientation.y = 0.0
        posS2.orientation.z = -0.08
        posS2.orientation.w = 0.98

        self.poses = [posT, posS1, posS2]
        self.idx_to_pose = {0: "Table", 1: "Shelf_1", 2: "Shelf_2"}

        # Get reference to services necessary to compute navigation plan
        rospy.wait_for_service("/move_base/make_plan")
        self.make_plan_srv = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        rospy.wait_for_service("/base_footprint_pose_service")
        self.base_pose_srv = rospy.ServiceProxy("/base_footprint_pose_service", PoseService)

        # Publisher for navigation plan
        self.path_pub = rospy.Publisher("/navigation_plan", Path, queue_size=10)

        # Action client for navigation service
        self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.loginfo("Navigation service active.")

    # Compute plan and publish it to Unity interface
    def make_plan_to_destination(self, target_idx):
        # Retrieve current robot's base_footprint pose
        base_footprint_transform = self.base_pose_srv(True)

        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.pose.position.x = base_footprint_transform.base_link_pose.position.x
        start_pose.pose.position.y = base_footprint_transform.base_link_pose.position.y
        start_pose.pose.position.z = base_footprint_transform.base_link_pose.position.z
        start_pose.pose.orientation.x = base_footprint_transform.base_link_pose.orientation.x
        start_pose.pose.orientation.y = base_footprint_transform.base_link_pose.orientation.y
        start_pose.pose.orientation.z = base_footprint_transform.base_link_pose.orientation.z
        start_pose.pose.orientation.w = base_footprint_transform.base_link_pose.orientation.w

        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.pose = self.poses[target_idx]
        
        # Compute plan
        response = self.make_plan_srv(start_pose, target_pose, 0.1)
        return response.plan
        
    # Routine to navigate real robot to target pose
    def navigate_to(self, idx):
        move_base_goal = MoveBaseGoal()

        nav_goal = PoseStamped()
        nav_goal.header.frame_id = "map"
        #nav_goal.header.stamp = rospy.Time.now()
        nav_goal.pose = self.poses[idx]
        
        move_base_goal.target_pose = nav_goal
        self.move_base_client.send_goal(move_base_goal)
        self.move_base_client.wait_for_result()


# Utility class to track the objects which are looked for by the robot
class ObjectsTracker:

    def __init__(self):

        self.object_39 = PointStamped()
        self.sub1 = rospy.Subscriber('/objects/39', PointStamped, self.update_object_39_pose)

    def update_object_39_pose(self, msg):
        self.object_39.header = msg.header
        self.object_39.point = msg.point


class ZedToBasetransformer:

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        
    def transform_to_base(self, point_stamped):
        try:
            zed_to_base_transf = self.tfBuffer.lookup_transform('base_footprint', 'zed_camera', rospy.Time())
            return tf2_geometry_msgs.do_transform_point(point_stamped, zed_to_base_transf)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None


class TiagoExperiment:

    def __init__(self, n_items, delay):
        self.n_items = n_items
        self.on_shelf_1 = n_items / 2
        self.on_shelf_2 = n_items / 2
        self.in_box = 0

        self.delay = delay

        self.nav_handler = NavHandler()
        self.tracker = ObjectsTracker()
        self.transformer = ZedToBasetransformer()
        
        self.motion_planners = []
        for limb in ["left", "right"]:
            rospy.wait_for_service("/{0}_group/plan_action".format(limb))
            planner = rospy.ServiceProxy("/{0}_group/plan_action".format(limb), ActionService)
            self.motion_planners.append(planner)
            rospy.loginfo("{0} planner service active!".format(limb))

        self.execution_srvs = []
        for limb in ["left", "right"]:
            rospy.wait_for_service("/{0}_group/execute_action".format(limb))
            exec_srv = rospy.ServiceProxy("/{0}_group/execute_action".format(limb), ExecutionService)
            self.execution_srvs.append(exec_srv)
            rospy.loginfo("{0} execution service active!".format(limb))

        # Fixed position for releasing items in the box
        self.above_table = Point()
        self.above_table.x = 0.75
        self.above_table.y = 0.05
        self.above_table.z = 1.05

        # Fixed horizontal grasping direction
        self.grasp_dir = Vector3()
        self.grasp_dir.x = 1

        rospy.loginfo("Ready for experiment!")

    # Routine for the experimental cycle
    def experiment_cycle(self):

        rospy.loginfo("\nPress ENTER to start:")

        raw_input()

        # Run as long as there are items to put in the box
        while self.in_box < self.n_items:

            # If there are elements in both shelves, choose randomly the next destination
            next_destination = 0
            if self.on_shelf_1 > 0 and self.on_shelf_2 > 0:
                next_destination = Random().randint(1,2)
            # Else...
            elif self.on_shelf_1 == 0 and self.on_shelf_2 > 0:
                next_destination = 2
            elif self.on_shelf_1 > 0 and self.on_shelf_2 == 0:
                next_destination = 1
            else:
                rospy.loginfo("No more items to pick and place, shutting down....")
                rospy.sleep(1.0)
                sys.exit(0)

            next_destination_str = self.nav_handler.idx_to_pose[next_destination]
            rospy.loginfo("Next destination is ---> {0}".format(next_destination_str))
            rospy.loginfo("Press ENTER to continue:")

            raw_input()

            # Compute plan before navigating
            rospy.loginfo("Requesting navigation plan towards {0}..".format(next_destination_str))
            plan = self.nav_handler.make_plan_to_destination(next_destination)
            rospy.loginfo("Plan found!")
            self.nav_handler.path_pub.publish(plan)

            rospy.sleep(self.delay)

            # Navigate to destination
            rospy.loginfo("Navigating to {0}...".format(next_destination_str))
            self.nav_handler.navigate_to(next_destination)
            rospy.loginfo("{0} reached!".format(next_destination_str))

            # ----Detection pipeline ----
            #tracked_objects_handler = ObjectsTracker()
            #rospy.sleep(1.0)
            #zed_to_base_transformer = ZedToBasetransformer()

            #rospy.wait_for_service("/right_group/plan_action")
            #rospy.wait_for_service("/right_group/execute_action")

            #transformed_point = None
            #while transformed_point == None:
            #    transformed_point = zed_to_base_transformer.transform_to_base(tracked_objects_handler.object_39)
            #print(transformed_point.point)

            pick_pos = Point()
            pick_pos.x = 0.62
            pick_pos.y = 0.0
            pick_pos.z = 0.87

            # Decide which move_group to plan for from perception..
            N = 0
            try:
                # Plan action
                motion_plan = self.motion_planners[N]("pick", [], pick_pos, None, self.grasp_dir)

                # Publish planned action to Unity interface

                # Execute it
                rospy.loginfo("Picking item...")
                _ = self.execution_srvs[N](motion_plan.planning_result, motion_plan.planned_action)
                rospy.loginfo("Item picked!")

            except rospy.ServiceException as e:
                pass

            rospy.sleep(1.0)
            rospy.loginfo("Next destination is ---> Table")
            rospy.loginfo("Press ENTER to continue:")

            raw_input()

            # Compute plan before navigating
            rospy.loginfo("Requesting navigation plan towards Table..")
            plan = self.nav_handler.make_plan_to_destination(0)
            rospy.loginfo("Plan found!")
            self.nav_handler.path_pub.publish(plan)

            rospy.sleep(self.delay)

            # Navigate to destination
            rospy.loginfo("Navigating to Table...")
            self.nav_handler.navigate_to(0)
            rospy.loginfo("Table reached!")

            try:
                # Plan action
                motion_plan = self.motion_planners[N]("place", [], None, self.above_table, self.grasp_dir)

                # Publish planned action to Unity interface

                # Execute it
                rospy.loginfo("Placing item...")
                _ = self.execution_srvs[N](motion_plan.planning_result, motion_plan.planned_action)
                rospy.loginfo("Item placed!")

            except rospy.ServiceException as e:
                pass

            # Increase counter for items picked
            self.in_box += 1
            # Reduce counter for items yet to be picked
            if next_destination == 1:
                self.on_shelf_1 -= 1
            elif next_destination == 2:
                self.on_shelf_2 -= 1
            else:
                rospy.loginfo("Not supposed to be here, shutting down...")
                sys.exit(1)
            
            
def main(args):

    rospy.init_node('tiago_experiment_node', log_level=rospy.INFO, disable_signals=True)

    n_items = int(args[1])
    delay = int(args[2])

    rospy.loginfo("Setting up everything...")

    exp = TiagoExperiment(n_items, delay)
    exp.experiment_cycle()


if __name__ == '__main__':
    main(sys.argv)
    

        
