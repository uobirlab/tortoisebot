#!/usr/bin/env python
import roslib; roslib.load_manifest('tortoise_follow')
import rospy
import tf
import math
from turtlebot_node.msg import TurtlebotSensorState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from turtle_talk.srv import *
from std_msgs.msg import String

FOLLOW_DISTANCE = 1.5
LINEAR_SPEED = 0.15


########################################################################
class TurtleFollow(object):
    """Main node class"""

    #----------------------------------------------------------------------
    def __init__(self):
        self.vel_pubisher = rospy.Publisher('/cmd_vel', Twist)
        self.tf_listener = tf.TransformListener()
        self.target_vel = Twist()
#        self.target_vel.linear.x = 0.15
        self.publishing_on = True
        self.following = False
        self.previous_positions=[]

        rospy.loginfo("Waiting for Speach node.")
        rospy.wait_for_service('/say')
        self.say_service = rospy.ServiceProxy('/say', Speach)

        # Block node until we are following
        rospy.loginfo("Waiting to see a person")
        self.say_service(String("Waiting to see you, please show yourself."))
        try:
            self.tf_listener.waitForTransform("/openni_depth_frame", "/head_1", rospy.Time(), rospy.Duration(100))
        except tf.Exception, e:
            rospy.loginfo("Error:%s"%e.message)
        rospy.loginfo("Found you!")
        self.say_service(String("I found you, now I will follow you."))
        self.following = True
        self.state_sub = rospy.Subscriber("/turtlebot_node/sensor_state", TurtlebotSensorState, self.incoming_data)
        
    
    #----------------------------------------------------------------------
    def incoming_data(self, msg):
        """Subscription to the sensor data"""
        isinstance(msg,TurtlebotSensorState)

        if msg.remote_opcode == msg.REMOTE_FORWARD:
            rospy.set_param("/turtlebot_node/bonus", True)
            print "Forwards"
        elif msg.remote_opcode == msg.REMOTE_NONE:
            rospy.set_param("/turtlebot_node/bonus", False)


        # Get the latest position of the person
        (trans,rot) = self.tf_listener.lookupTransform("/openni_depth_frame", "/head_1", rospy.Time(0))

        # If the exact same transform is for the last 5 tries then oh crap, we lost it.
        # This seriously needs improving.
        self.previous_positions.append((trans[0],trans[1]))
        if len(self.previous_positions)>15:
            self.previous_positions=self.previous_positions[-5:]
        same=0
        f=self.previous_positions[0]
        for i in self.previous_positions[1:]:
            if i[0]==f[0] and i[1]==f[1]:
                same=same+1
        if same==14:
            self.say_service(String("I lost you."))
            self.publishing_on=False
                                       
        # Got to try and keep the Y component 0 at all times => heading towards person
        angle_to_person=math.atan2(trans[1],trans[0])
        self.target_vel.angular.z=4.0*angle_to_person

        # X component should be kept at the following distance.
        d=trans[0] - FOLLOW_DISTANCE
        self.target_vel.linear.x=3.0*d

        if self.publishing_on:
            self.vel_pubisher.publish(self.target_vel)
        

    #----------------------------------------------------------------------
    def laser_data(self,msg):
        """"""
        isinstance(msg,LaserScan)
        
        self.dist= min(msg.ranges)
        print self.dist

    #----------------------------------------------------------------------    
    def __del__(self):
        self.target_vel.linear.x=0
        self.target_vel.angular.z=0
        self.vel_pubisher.publish(self.target_vel)
        
    
if __name__ == '__main__':
    rospy.init_node('wall_finder')
    
    try:
        node = TurtleFollow()

        while not rospy.is_shutdown():
            rospy.sleep(1)
            pass
        
    except rospy.ROSInterruptException: pass
