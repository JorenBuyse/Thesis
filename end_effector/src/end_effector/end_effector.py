#!/usr/bin/env python


import collections 
import rospy as ros
from std_srvs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
import subprocess
import socket
import time

class End_effector:

	#variable list
        servolist = "180"
	localIP = '192.168.0.58' 
	localPort = 2390
	arduinoPort = 2391
	bufferSize = 1024 #
	
        address = ( '192.168.0.59', 2391) #arduino IP and port, hardgecodeerd op arduino
	 
	print("UDP client on %s:%s" % (localIP,localPort)) 
	client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #Socket setup      
        client_socket.settimeout(1) #only wait 1 second for a resonse
	
        servocount = 2 #default ammount of servo's, will be updated on initialisation
        sensorcount = 2 #default ammount of sensors, will be updated on initialisation
        
        def __init__(self, n_saved_=10):        
                
                "Create handles and variables for SR and SS"
                
                self.sensordata = ""
                
                                
		# ROS parameters
                self.node_name = "end_effector"         
                self.pub_rate = 20
                self.pub_name = "ee_data"		
                self.commands_sub_name = "ee_commands"		
                self.queue_size = 2
		
               
        def __ros_commands_sub(self, msg):
                "Forward commands send by baxter on the topic ee_commands to Arduino"
                self.servolist = msg.data #updata global variable servolist
                #self.client_socket.sendto(self.servolist.encode('utf-8'), self.address) #send command to arduino with UDP
                print("SERVO SET: " + self.servolist)
		return
                              
                       

        def __ros_pub(self):
                "ROS publisher of the last sensordata"
		rate = ros.Rate(self.pub_rate)
                self.client_socket.sendto(self.servolist.encode('utf-8'), self.address)  
		x = ""	
				                             
                while not ros.is_shutdown():			
                        
                        try:
					
				(rec_data,addr) = self.client_socket.recvfrom(2048) #Read data 
				self.client_socket.sendto(self.servolist.encode('utf-8'), self.address) 
				#print(rec_data)
				x = rec_data.split(':,')
				#print(x)
				self.sensordata = rec_data
				sendBuffer = self.sensordata + "," +self.servolist
				print(sendBuffer)
				if x[0] == "D":
					#self.pub.publish(self.sensordata)
					self.pub.publish(sendBuffer)
				
												
                        except:
                                pass
                              
                        #ros.loginfo("Last sensordata: " + self.sensordata)                     
						
                        #self.client_socket.sendto(self.servolist.encode('utf-8'), self.address)
			
			rate.sleep()
			
		#return


        def start_ros_node(self):
                "This function allows to use the class as a ROS node with SS subscriber and SR publisher \
		instead of using it inside a python code"

                ros.init_node(self.node_name)
                self.pub = ros.Publisher(self.pub_name, String, queue_size=self.queue_size)
                self.sub = ros.Subscriber(self.commands_sub_name, String, callback=self.__ros_commands_sub, queue_size=self.queue_size)
		#self.srv = ros.Service(self.srv_name, Trigger, self.__ros_srv)

                try:
                        self.__ros_pub()
                except ros.ROSInterruptException:
                        pass

                return
	
       	