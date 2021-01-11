#! /usr/bin/env python


import rospy

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import task_manager.msg




class FollowTrajectoryClient(object):

  def __init__(self, name, joint_names):
    self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                               FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for %s..." % name)
    self.client.wait_for_server()
    self.joint_names = joint_names
    rospy.loginfo("Controller Connected")

  def move_to(self, positions, duration=5.0):
    if len(self.joint_names) != len(positions):
        print("Invalid trajectory position")
        return False
    trajectory = JointTrajectory()
    trajectory.joint_names = self.joint_names
    trajectory.points.append(JointTrajectoryPoint())
    trajectory.points[0].positions = positions
    trajectory.points[0].velocities = [0.0 for _ in positions]
    trajectory.points[0].accelerations = [0.0 for _ in positions]
    trajectory.points[0].time_from_start = rospy.Duration(duration)
    follow_goal = FollowJointTrajectoryGoal()
    follow_goal.trajectory = trajectory

    self.client.send_goal(follow_goal)
    self.client.wait_for_result()
    return self.client.get_result()




class PTUAction(object):
  # create messages that are used to publish feedback/result
  _feedback = task_manager.msg.PTUFeedback()
  _result   = task_manager.msg.PTUResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, task_manager.msg.PTUAction, execute_cb=self.execute_cb, auto_start = False)
    self._as.start()
    self.ptu_action = FollowTrajectoryClient("ptu_controller", ["ptu_pan_joint", "ptu_tilt_joint"]) 
    rospy.loginfo('PTU Action Server Started')
    
    
  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    # success = True  

    
    # publish info to the console for the user
    rospy.loginfo('Starting Action')

    res = self.ptu_action.move_to([goal.joint_values[0], goal.joint_values[1],])

    

    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
    

    rospy.loginfo('Executing Action')

      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    r.sleep()
    
    if res:
        success = True
    else:
        success = False    
    
    if success:
        rospy.loginfo('Action Completed')
        self.set_status('SUCCESS')
    else:
        self.set_status('FAILURE')
      
#  update the status

  def set_status(self,status):
      if status == 'SUCCESS':
        self._feedback.status = 1
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)
      elif status == 'FAILURE':
        self._feedback.status = 2
        self._result.status = self._feedback.status
        rospy.loginfo('Action %s: Failed' % self._action_name)
        self._as.set_succeeded(self._result)
      else:
        rospy.logerr('Action %s: has a wrong return status' % self._action_name)



if __name__ == '__main__':
  rospy.init_node('ptu_server')
  PTUAction('ptu_action')
  rospy.spin()
