#!/usr/bin/env python  
import rospy  
import math
from geometry_msgs.msg import Twist #Importar tipo de mensaje que usa cmd_vel 

class GPSPubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)   
        ############################### SUBSCRIBERS #####################################  
        #Subscriber que toma los valores de velocidad de operator para posteriormente hacerlos mas grandes
        rospy.Subscriber('nav2d_vel', Twist, self.get_cmd) 
        ####  PUBLISHERS ###
        self.cmd_vel_pub=rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        ### CONSTANTS ###
        self.robot_vel=Twist()
        self.robot_vel.linear.x=0
        self.robot_vel.angular.z=0
        r = rospy.Rate(10) #10 Hz 
        #********** INIT NODE **********###  
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.robot_vel) #Publicar valores de velocidad ya incrementados 
            r.sleep()
      
    def get_cmd (self, msg):
        #Se leen valores de velocidad de nav2d_vel para multiplicarse y hacer que UMA se mueva mejor
        self.robot_vel.linear.x = 1.5 * msg.linear.x #1.5
        self.robot_vel.angular.z = 4.6 * msg.angular.z #4.5   
     
    def cleanup(self):  

        #Esta funcion es ejecutada antes de que el nodo se cierre, se usa para detener al UMA  
        self.robot_vel.angular.z = 0
        self.robot_vel.linear.x = 0 
        self.cmd_vel_pub.publish(self.robot_vel)

      

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("gps_subscriber", anonymous=True)  

    try:  

        GPSPubClass()  

    except:  

        rospy.logfatal("gps_subscriber died")  
