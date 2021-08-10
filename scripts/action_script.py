#! /usr/bin/env python
import rospy
import time
import actionlib
from actions_quiz.msg import CustomActionMsgAction, CustomActionMsgFeedback, CustomActionMsgResult
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

    
class drone_land_or_take_off_class(object):
    
  # create messages that are used to publish feedback/result
  _feedback = CustomActionMsgFeedback()
  _result   = CustomActionMsgResult()

  def __init__(self):
     # creates the action server
     self._as = actionlib.SimpleActionServer("/action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
     self._as.start()
     self.ctrl_c = False
     self.rate = rospy.Rate(1) # 1hz
    
  def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.drone_vel_publisher.get_num_connections()
            if connections > 0:
                self.drone_vel_publisher.publish(self.cmd)
                rospy.loginfo("Velocity Published")
                break
            else:
                self.rate.sleep()

  
  def stop_drone(self):
      self.cmd.linear.x = 0.0
      self.cmd.angular.z = 0.0
      self.publish_once_in_cmd_vel()
      rospy.loginfo("Stop the drone")


  def goal_callback(self, goal):
     # this callback is called when the action server is called.
     # helper variables
     r = rospy.Rate(1)
     success = True

     self.drone_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
     self.cmd = Twist()
     self.pub_takeoff = rospy.Publisher('/drone/takeoff',Empty, queue_size = 1)
     self.take_off = Empty()
     self.pub_land = rospy.Publisher('/drone/land', Empty, queue_size = 1)
     self.land = Empty()

     
     if (goal.goal == 'TAKEOFF'):
         i = 0
         while i < 3:
             if self._as.is_preempt_requested():
                 rospy.loginfo('The goal has been cancelled/preempted')
                 self._as.set_preempted()
                 success = False
                 break
             rospy.loginfo("Taking Off")
             self.pub_takeoff.publish(self.take_off)
             time.sleep(1)
             self._feedback.feedback = 'taking off'
             self._as.publish_feedback(self._feedback)
             i += 1
     elif (goal.goal == 'LAND'):
         self.stop_drone()
         i = 0
         while i<3:
             if self._as.is_preempt_requested():
                 rospy.loginfo('The goal has been cancelled/preempted')
                 self._as.set_preempted()
                 success = False
                 break
             self.pub_land.publish(self.land)
             rospy.loginfo("Landing...")
             time.sleep(1)
             self._feedback.feedback = 'landing'
             self._as.publish_feedback(self._feedback)
             i +=1
     else:
         rospy.loginfo("The current action supports the following requests: TAKEOFF, LAND. Please choose and enter one of the above!")
    
     # at this point, either the goal has been achieved (success==true)
     # or the client preempted the goal (success==false)
     # If success, then we publish the final result
     # If not success, we do not publish anything in the result
     if success:
         self._result = Empty()
         self._as.set_succeeded(self._result)
      
     self.stop_drone()
    

if __name__ == '__main__':
  rospy.init_node('move_drone_in_square')
  drone_land_or_take_off_class()
  rospy.spin()