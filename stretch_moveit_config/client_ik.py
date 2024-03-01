#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped

joint_states_sub = False
robot_state=RobotState()

def joint_state_callback(msg):
    joint_positions = [msg.position[i] for i, joint_name in enumerate(msg.name) if joint_name.startswith("joint")]

    robot_state.joint_state.name = msg.name
    robot_state.joint_state.position = joint_positions

    rospy.loginfo("Updated Robot State: %s", robot_state)

    joint_states_sub = True

def compute_ik_client():
    rospy.init_node('ik_client_node')
    pose_goal = PoseStamped()
    #Pose details
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.position.x = -0.01
    pose_goal.pose.position.y = -0.6
    pose_goal.pose.position.z = 0.5

    pose_goal.pose.orientation.x = 0.022
    pose_goal.pose.orientation.y = -0.666
    pose_goal.pose.orientation.z = 0.745
    pose_goal.pose.orientation.w = 0.016


 #   robot_state=RobotState()

    
    rospy.Subscriber('/stretch/joint_states', JointState, joint_state_callback, queue_size=1)
    
    ik_request = GetPositionIK._request_class()
    ik_request.ik_request.group_name = "stretch_arm"
    ik_request.ik_request.ik_link_name = "link_wrist_roll"
    ik_request.ik_request.pose_stamped = pose_goal
    ik_request.ik_request.robot_state = robot_state
    ik_request.ik_request.avoid_collisions = False
    ik_request.ik_request.timeout = rospy.Duration(10.0)


    ik_service = rospy.ServiceProxy('/compute_ik', GetPositionIK)
    try:
        ik_response = ik_service(ik_request)
        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            joint_angles = ik_response.solution.joint_state.position
            #Joint angles can be used in script
            rospy.loginfo("IK solution obtained successfully!")
        else:
            rospy.logerr("IK failed with error code %d", ik_response.error_code.val)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
    

    rospy.spin()

if __name__ == "__main__":
    compute_ik_client()

