#!/usr/bin/env python

# imports
from math import sin, cos, atan2, pi

import rospy
from geometry_msgs.msg import Pose2D
from math import sqrt, sin, cos, atan2
from geometry_msgs.msg import Pose2D, Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range



# exercise class
class Ex2():
    # init func
    def __init__(self,name):
        # create node
        rospy.init_node('thymio_controller', anonymous=True)
        #set thymeio name
        self.name = name
        # log name
        rospy.loginfo('Controlling %s' % self.name)
        #publisher to send move commands
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % self.name, Twist, queue_size=10)
        #pose to be filled
        self.pose = Pose2D()
        #velocity message and frequency setup
        self.vel_msg = Twist()
        Hz = 10.0
        self.rate = rospy.Rate(Hz)
        self.step = rospy.Duration.from_sec(1/Hz)
        #set sensors distance before start turning
        self.thresh = 0.10
        #create dictionary to keep proximity sensor values
        self.proximity_sensors = ["left", "center_left", "center", "center_right", "right","rear_left","rear_right"]
        self.proximity_distances = dict()
        #create subscriber for proximity sensor
        self.proximity_subscribers = [rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor) for sensor in self.proximity_sensors]
    # update func for proximity listener
    def update_proximity(self, data, sensor):
        self.proximity_distances[sensor] = data.range
    #Function to reposition robot perpendicular to wall(on x axis)
    def perpendicular_position(self):
        while not rospy.is_shutdown():
            #error is difference between the 2 front center parameters
            error = self.proximity_distances["center_left"] - self.proximity_distances["center_right"]
            # prepare rotation messag by setting x to 0
            self.vel_msg.linear.x = 0.0
            #if error is on left or right sensor set a small rotation on that side
            if error>0.001:
                self.vel_msg.angular.z = -0.1
            elif error<-0.001:
                self.vel_msg.angular.z = 0.1
            # if error is small enough it will be considered to be perpendicular
            else:
                break
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
        # once reached perp rotation set stop movement
        self.stop_moving()
    # func to stop robot movement
    def stop_moving(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()
    # main logic
    def go(self):
        while not rospy.is_shutdown():
            # if at least 2 front sensor detect a wall stop movement and start turning
            if sum(v < self.thresh for k,v in list(self.proximity_distances.items())) >= 2:
                # stop straight move
                self.stop_moving()
                # position perpendicular to wall
                self.perpendicular_position()
                break
            else:
                #move straight for a small amount
                self.vel_msg.linear.x = 0.05
                self.vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(self.vel_msg)

if __name__ == '__main__':
    # create class
    controller = Ex2("Thymio1")
    try:
        #call logic
        controller.go()
    except rospy.ROSInterruptException as e:
        pass