#!/usr/bin/env python


import collections 
import rospy as ros
from std_srvs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
import subprocess
import socket
import time
#import struct
#import msvcrt

class Servo_input:

	#variable list
        servolist = ""	
        servocount = 1 #default ammount of servo's
                
        def __init__(self, n_saved_=10):        
                
                "Create handles and variables"
                
                # ROS parameters
                self.node_name = "servo_input"         
                self.pub_rate = 20
                self.pub_name = "ee_commands"		
                self.queue_size = 2


        def __ros_pub(self):
                "ROS publisher of the servo values"
		rate = ros.Rate(self.pub_rate)
		q = 0
                while not ros.is_shutdown():			
                        try:	
				
                                x=180				
				if q <3:
					q=q+1
					for y in range (0,90):
						x = 180-y
						print(x)
						self.servolist = str(x)
						self.pub.publish(self.servolist)
						time.sleep(0.01)
				else:
					self.servolist = str(x)
					self.pub.publish(self.servolist)
					time.sleep(0.01)
                                											
                        except:
                                pass
                              
                        
			rate.sleep()
			
		#return
	
		

	
		

        def start_ros_node(self):
                "This function allows to use the class as a ROS node with SS subscriber and SR publisher \
		instead of using it inside a python code"

                ros.init_node(self.node_name)
                self.pub = ros.Publisher(self.pub_name, String, queue_size=self.queue_size)
                
		#self.srv = ros.Service(self.srv_name, Trigger, self.__ros_srv)

                try:
                        self.__ros_pub()
                except ros.ROSInterruptException:
                        pass

                return
	
