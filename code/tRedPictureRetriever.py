#!/usr/bin/env python

#imports
from math import sin, cos, atan2, pi
import rospy
from geometry_msgs.msg import Pose2D
from math import sqrt, sin, cos, atan2
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Range
from ex3 import Ex3
from ex3 import Ex2
import numpy as np
from copy import copy
import random
import tf
from nav_msgs.msg import Odometry
import sys
import os as os
from statistics import mean
import time

import predict
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
#exercise class 
class Ex4(Ex3):
    #initialize by calling superclass
    def __init__(self,name):
        super(Ex4,self).__init__(name)

        self.state=0
        self.dir=0
        self.foundCrossroad=0
        self.foundGiveWay=0
        self.straightSpeed=0.1
        self.foundSpeedUp=0
        self.founSlowDown=0
        self.imagename=1
        self.crossroadCounter=0

        self.predictor = predict.PredictModel()


        rospy.Subscriber("%s/camera/image_raw" % (name), Image, self.callback)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("%s/image_topic" % (name), Image)
        # create node
        rospy.init_node('thymio_controller', anonymous=True)
        #set thymeio name
        self.name = name
        self.robot=1
        rospy.loginfo('Controlling %s' % self.name)
        #publisher to send move commands
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel' % name, Twist, queue_size=10)
        #pose to be filled
        self.pose = Pose2D()
        #velocity message and frequency setup
        self.vel_msg = Twist()
        Hz = 50.0
        self.rate = rospy.Rate(Hz)
        self.stop = rospy.Rate(Hz*3)
        self.crawl = rospy.Rate(Hz*2)
        self.step = rospy.Duration.from_sec(1/Hz)
        #set sensors distance before start turning
        self.thresh = 0.10
        self.distanceCenter=0.0
        #create dictionary to keep proximity sensor values
        self.proximity_sensors = ["left", "center_left", "center", "center_right", "right","rear_left","rear_right"]
        self.proximity_distances = dict()
        #create subscriber for proximity sensor
        self.proximity_subscribers = [rospy.Subscriber('/%s/proximity/%s' % (self.name, sensor), Range, self.update_proximity, sensor) for sensor in self.proximity_sensors]
    #function that turns robot by a certain angle degree
    def callback(self,msg):
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY, dstCn=0)
        frame = np.array(gray)
        width, height = frame.shape[1], frame.shape[0]
        raw = frame.tostring()
        ERECT, POSITIONE, far,center,low = self.erecter(cv2_img,self.robot)
        if POSITIONE==3:
            self.foundCrossroad=1
        else:
            self.distanceCenter=((low[0][1]+low[1][1])/2)-(640//2)
            self.foundCrossroad=0
        if ((((far[0][1]+far[1][1])/2)>((640//2)-100)) and (((far[0][1]+far[1][1])/2)<((640//2)+100))):
            pass
        else:
            if(len(center)>1):
                if ((((center[0][1]+center[1][1])/2)>((640//2)-100)) and (((center[0][1]+center[1][1])/2)<((640//2)+100))):
                    self.dir=0
                    self.state=0
                elif (((center[0][1]+center[1][1])/2)>((640//2)+100)):
                    self.state=1
                    self.dir=1
                elif (((center[0][1]+center[1][1])/2)<((640//2)+100)):
                    self.state=1
                    self.dir=-1
            else:
                print("found crossroad!")
            
            
    def erecter(self, img,robot):
        self.imagename=self.imagename+1
        if (self.imagename%3==0):
            crop_img = img[50:220, 440:605]
            crop_img[np.where(crop_img[:,:,0]==crop_img[:,:,1])]=[0,0,0]
            imagename="img_"+str(self.imagename)+".png"
            cv2.imwrite(imagename, crop_img)
            try:
                self.image_message = self.bridge.cv2_to_imgmsg(crop_img, encoding="bgr8")
            except CvBridgeError as e:
                print(e)
            self.image_pub.publish(self.image_message)
        else:
            try:
                self.image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            except CvBridgeError as e:
                print(e)
            self.image_pub.publish(self.image_message)
        
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_pink = np.array([0, 5, 15])
        upper_pink = np.array([15, 255, 255])
        lower_pink_2 = np.array([135, 10, 10])
        upper_pink_2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_pink, upper_pink)
        mask2 = cv2.inRange(hsv, lower_pink_2, upper_pink_2)
        mask = cv2.bitwise_or(mask1, mask2)
        out = cv2.bitwise_and(img, img, mask = mask)
        
        
        # cv2_imshow(out)
        colorLow=[0, 5, 15]
        colorHigh=[180, 255, 255]
        # linefar=480-70
        # lineCenter=480-50
        linefar=480-90
        lineCenter=480-14
        lineLow=480-10
        redPointsFar=[]
        redPointsCenter=[]
        redPointsLow=[]
        for x in range(0,640):
            if (out[(linefar,x)][0]>=colorLow[0] and out[(linefar,x)][0]<=colorHigh[0] and out[(linefar,x)][1]>=colorLow[1] and out[(linefar,x)][1]<=colorHigh[1] and out[(linefar,x)][2]>=colorLow[2] and out[(linefar,x)][2]<=colorHigh[2]):
                redPointsFar.append((linefar,x))
                img = cv2.circle(img, (x,linefar), radius=0, color=(0, 255, 0), thickness=-1)
            if (out[(lineCenter,x)][0]>=colorLow[0] and out[(lineCenter,x)][0]<=colorHigh[0] and out[(lineCenter,x)][1]>=colorLow[1] and out[(lineCenter,x)][1]<=colorHigh[1] and out[(lineCenter,x)][2]>=colorLow[2] and out[(lineCenter,x)][2]<=colorHigh[2]):
                redPointsCenter.append((lineCenter,x))
                img = cv2.circle(img, (x,lineCenter), radius=0, color=(0, 255, 0), thickness=-1)
            if (out[(lineLow,x)][0]>=colorLow[0] and out[(lineLow,x)][0]<=colorHigh[0] and out[(lineLow,x)][1]>=colorLow[1] and out[(lineLow,x)][1]<=colorHigh[1] and out[(lineLow,x)][2]>=colorLow[2] and out[(lineLow,x)][2]<=colorHigh[2]):
                redPointsLow.append((lineLow,x))
                img = cv2.circle(img, (x,lineLow), radius=4, color=(0, 255, 0), thickness=-1)
        position=0
        speedString="LINEAR SPEED:"+str(self.straightSpeed)
        img=cv2.putText(img,speedString, (10,20), cv2.FONT_HERSHEY_PLAIN, 2, 255)
        if(self.foundCrossroad==1 or self.crossroadCounter<300):
            self.crossroadCounter=0
            img=cv2.putText(img,"FOUND CROSSROAD", (10,40), cv2.FONT_HERSHEY_PLAIN, 2, 255)

        # try:
        #     self.image_message = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        # except CvBridgeError as e:
        #     print(e)
        # self.image_pub.publish(self.image_message)

        if len(redPointsLow) == 0:
                position = 3
                return out,position,((0,0),(0,0)),((0,0),(0,0)),((0,0),(0,0))
        elif ((len(redPointsLow)!=0) and (len(redPointsFar)!=0) and (len(redPointsCenter)!= 0)):
            firstFar=redPointsFar[0]
            lastFar=redPointsFar[len(redPointsFar)-1]
            firstCenter=redPointsCenter[0]
            lastCenter=redPointsCenter[len(redPointsCenter)-1]
            firslLow=redPointsLow[0]
            lastLow=redPointsLow[len(redPointsLow)-1]
            return out,position,(firstFar,lastFar),(firstCenter,lastCenter),(firslLow,lastLow)
        elif ((len(redPointsLow)!=0) and (len(redPointsCenter)!= 0)):
            firstCenter=redPointsCenter[0]
            lastCenter=redPointsCenter[len(redPointsCenter)-1]
            firslLow=redPointsLow[0]
            lastLow=redPointsLow[len(redPointsLow)-1]
            return out,position,((2000,2000),(2000,2000)),(firstCenter,lastCenter),(firslLow,lastLow)
        elif ((len(redPointsLow)!=0)):
            firslLow=redPointsLow[0]
            lastLow=redPointsLow[len(redPointsLow)-1]
            return out,position,((2000,2000),(2000,2000)),((2000,2000),(2000,2000)),(firslLow,lastLow)
        

    def stop_moving(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(self.vel_msg)
        self.rate.sleep()


    def go(self):
        straightAmount=0
        rightAmount=0
        leftAmount=0

        while not rospy.is_shutdown():
            
            linear=0.1
            if(straightAmount>10):
                straightAmount=0
                rightAmount=0
                leftAmount=0
                self.state=0
            if self.dir==1:
                self.vel_msg.linear.x = linear
                self.vel_msg.angular.z = -min(0.3,(self.distanceCenter/1650))
                self.velocity_publisher.publish(self.vel_msg)
                self.rate.sleep()
                rightAmount=rightAmount+1
            elif self.dir==-1:
                self.vel_msg.linear.x = linear
                self.vel_msg.angular.z = min(0.3,(self.distanceCenter/1650))
                self.velocity_publisher.publish(self.vel_msg)
                self.rate.sleep()
                leftAmount=leftAmount+1
            elif self.dir==0:
                self.vel_msg.linear.x = self.straightSpeed
                self.vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(self.vel_msg)
                straightAmount=straightAmount+1


if __name__ == '__main__':
    #create class
    controller = Ex4('Thymio1')
    try:
        #repeat logic
        while not rospy.is_shutdown():
            os.system("""rosservice call /Thymio1/camera_pitch_controller/set_parameters "config:
  bools:
  - {name: '', value: false}
  ints:
  - {name: '', value: 0}
  strs:
  - {name: '', value: ''}
  doubles:
  - {name: 'pitch', value: 0.0}
  groups:
  - {name: '', state: false, id: 0, parent: 0}" """)
            #call exercise 2 logic
            controller.go()
    except rospy.ROSInterruptException as e:
        pass




