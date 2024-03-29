#!/usr/bin/env python3


#Used https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py as template. See what they do for better explanations. There are parts here--such as setup--which are almost exactly as in that document.


#necessary imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import roslib
import numpy as np




#not actually sure why this is here or why its necessary, but I suppose its a good way to test tolerances
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False


  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)


  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)


  return True


#How the robot is understood and controlled
class MoveGroupPythonInterface(object):
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()
    
    #the moveit_commander is what is responsible for sending info the moveit controllers
    moveit_commander.roscpp_initialize(sys.argv)
    
    #Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()
    
    #Instantiate a `PlanningSceneInterface`_ object. This provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    
    #Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a planning group (group of joints), which in our moveit setup is named 'arm'
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    #Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
 
    #Get all the info which is carried with the interface object
    #We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("Planning frame: %s" % planning_frame)


    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("End effector link: %s" % eef_link)


    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("Available Planning Groups:", robot.get_group_names())


    # Misc variables
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    
  def plan_cartesian_path(self, scale=1):
    print('Planning')
    N = 50
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    for i in range(N):
    
      wpose.position.x = wpose.position.x + 0.001
      wpose.position.y = wpose.position.y
      wpose.position.z = wpose.position.z + 0.002
      
      waypoints.append(copy.deepcopy(wpose))
      
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.001,       # eef_step
                                       0.0)       # jump_threshold
    print("Planning for " + str(fraction * 100) + "% of trajectory acheived!")
    return plan


  def display_trajectory(self, plan):
    #ask rviz to display the trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):
    #execute given plan
    self.move_group.execute(plan, wait=True)


  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
    #either this times out and returns false or the object is found within the planning scene and returns true
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in self.scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False




def main():
  #initialize node
  rospy.init_node('stretch_planning', anonymous=True)
  try:
    print("Planning a demo")
    print("Press Ctrl-D to exit at any time")
    print("Press 'Enter' to begin")
    input()
    stretch_arm = MoveGroupPythonInterface()
    
    print("Press 'Enter' to plan trajectory")
    input()
    plan = stretch_arm.plan_cartesian_path()
    stretch_arm.display_trajectory(plan)
    
    print("Press 'Enter' to execute trajectory")
    input()
    stretch_arm.execute_plan(plan)
    
    print("Execution complete")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()
