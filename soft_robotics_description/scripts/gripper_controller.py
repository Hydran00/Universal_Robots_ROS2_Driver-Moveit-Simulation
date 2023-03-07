#!/usr/bin/env python 
# Echo client program
import socket
import sys
import os
import rospy
from std_msgs.msg import String
HOST = "192.168.0.100" # The UR IP address
PORT = 30002 # UR secondary client
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
#f = open ("Grip.script", "rb")   #Robotiq Gripper
#f = open ("setzero.script", "rb")  #Robotiq FT sensor
path = os.path.expanduser('~')+ '/catkin_ws/src/soft_robotics/soft_robotics_description/scripts/'
def callback(data):
    
    script=''
    if(data.data == 'open1'):
        script = path + 'open1.script'
    elif(data.data == 'open2'):
        script = path + 'open2.script'
    elif(data.data == 'close'):
        script = path + 'close.script'
    else:
        print("Invalid argument!")
    f = open (script, "rb")
    l = f.read(2024)
    while (l):
        s.send(l)
        l = f.read(2024)
    f.close()

def listener():
    rospy.init_node('gripper_controller_execution', anonymous=True)
   
    rospy.Subscriber("gripper_controller_cmd", String, callback,queue_size=10)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    listener()

