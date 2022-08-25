#!/usr/bin/env python  
import rospy
import tf2_ros
from tiago_holo_dt.srv import PoseService, PoseServiceResponse


class BasePoseSrvClass:

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)

        self.srv = rospy.Service('base_footprint_pose_service', PoseService, self.get_base_pose)
        rospy.loginfo("Advertising base_footprint_pose_service...")

    def get_base_pose(self, req):
        
        rospy.loginfo("Received request for base_footprint pose.")
        response = PoseServiceResponse()
        try:
            base_transf = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            response.base_link_pose.position.x = base_transf.transform.translation.x
            response.base_link_pose.position.y = base_transf.transform.translation.y
            response.base_link_pose.position.z = base_transf.transform.translation.z
            response.base_link_pose.orientation.x = base_transf.transform.rotation.x
            response.base_link_pose.orientation.y = base_transf.transform.rotation.y
            response.base_link_pose.orientation.z = base_transf.transform.rotation.z
            response.base_link_pose.orientation.w = base_transf.transform.rotation.w
            return response

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None


def main():
    rospy.init_node('base_pose_service_node', log_level=rospy.INFO)

    BasePoseSrvClass()
    rospy.spin()


if __name__ == '__main__':
    main()
    

