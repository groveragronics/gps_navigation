#!/usr/bin/env python  
import rospy  
import math     #Para poder usar la tangente inversa, abs()... 
from sensor_msgs.msg import NavSatFix #Importar tipo de mensaje que usa el GPS
from sensor_msgs.msg import Imu #Importar tipo de mensaje IMU
from tf.transformations import euler_from_quaternion #Libreria para tranformar los datos del IMU de cuaterniones a angulos de Euler
from nav2d_operator.msg import cmd #Importar tipo de mensaje a publicar a operator

class GPSPubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)   
        ############################### SUBSCRIBERS #####################################  
        # Subscriber para leer el valor del gps del UMA, gps del goal, valor del IMU  
        rospy.Subscriber("fix", NavSatFix, self.gps_cb)
        rospy.Subscriber("fixgoal", NavSatFix, self.gps_goal_cb)
        rospy.Subscriber('imu', Imu, self.get_rotation)
        ####  PUBLISHERS ###
        #Publica velocidad a cmd el cual recibe operator para la evasion de obstaculos
        self.cmd_pub=rospy.Publisher("cmd", cmd, queue_size = 1)
        ### CONSTANTS ###
        self.robot_vel=cmd()
        self.robot_vel.Velocity=0
        self.robot_vel.Turn=0
        r = rospy.Rate(10) #10 Hz 
        self.dif_latitude=0.0
        self.dif_longitude=0.0
        self.latitude_robot=0.0
        self.longitude_robot=0.0
        self.latitude_goal=0.0
        self.longitude_goal=0.0
        #********** INIT NODE **********###  
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.robot_vel) #Publicando datos a cmd_pub a una frecuencia de 10Hz
            r.sleep()

    def gps_cb(self, msg):  
        #Se obtienen valores de latitud y longitud del GPS del robot (topico fix)   
        self.latitude_robot= msg.latitude
        self.longitude_robot= msg.longitude + (-0.000009594 * 0.165) #Correccion de longitud por posicion de GPS en el UMA

    def gps_goal_cb(self, msg):  
        #Se obtienen valores de latitud y longitud del GPS goal (topico fixgoal) y se calcula la diferencia que hay respecto al goal
        self.latitude_goal= msg.latitude
        self.longitude_goal= msg.longitude
        self.dif_latitude= self.latitude_goal - self.latitude_robot
        self.dif_longitude= self.longitude_goal - self.longitude_robot
    
    def get_rotation (self, msg):
        #Transformacion de valores de IMU de cuaterniones a angulos de Euler
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        
        #Calculo del angulo del goal respecto al UMA
        #Angulos de cuadrantes 1 y 2 respecto a x. En radianes de 0 a Pi (0 -> 180)
        #Angulos de cuadrantes 2 y 4 respecto a x. En radianes de 0 a -Pi (0 -> -180)
        angle_goal = -1* math.atan2(self.dif_longitude, (self.dif_latitude + 0.0000000000000000001) )
      
        ## Movimiento de motores para llegar al objetivo
        kdist = 25000 #La constante por la que se multiplica la diferencia de distancia (en terminos de lat y long) entre UMA y goal        
        kang = -0.20  #Para dejar los valores de velocidad angular en rango de -1 a 1 que son los que lee Turn en cmd
        vmax = 1.0    #Para dejar los valores de velocidad lineal en terminos de -1 a 1 que son los que lee Velocity en cmd
        latlong = 0.00001317690754 #1m se representa como 0.00001317690754 en terminos de  latitude y longitude aproximadamente
        #Calculo de la diferencia de angulo del goal respecto al frente del UMA usando yaw valor generado por el IMU del UMA.  
        dif_angle = angle_goal - yaw
        ## Calculo de distancia al goal
        dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5  
       
        #Evaluacion de casos especiales en donde la diferencia de angulo no corresponde al valor necesario para girar hacia el goal. 
        if (angle_goal < -0.07 and angle_goal > -1.50 and yaw > 2.355): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.1
            self.robot_vel.Turn =  kang * (dif_angle + 6.28)
        elif (angle_goal < -1.64 and yaw > 0.785): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.1
            self.robot_vel.Turn =  kang * (dif_angle + 6.28)
        elif (angle_goal > 1.64 and yaw < -0.785): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.1
            self.robot_vel.Turn =  kang * (dif_angle - 6.28)
        elif (angle_goal > 0.07 and angle_goal < 1.50 and yaw < -2.355): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.1
            self.robot_vel.Turn =  kang * (dif_angle - 6.28)
        else:
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.1
            self.robot_vel.Turn =  self.robot_vel.Turn =  kang * dif_angle
        #Si la velocidad angular de UMA es pequeña puede incrementar su velocidad lineal     
        if( (abs(self.robot_vel.Turn)) < 0.1 ): 
            #El UMA se queda a 50cm de distancia de separacion del goal y no se mueva si el objetivo esta a mas de 500m de distancia
            if(dist_to_goal  > (latlong/2.0) and dist_to_goal  < (latlong*500)):
                self.robot_vel.Velocity = self.robot_vel.Velocity + 0.3 
                if (self.robot_vel.Velocity >= (kdist*dist_to_goal) ): 
                    self.robot_vel.Velocity = kdist*dist_to_goal
                if (self.robot_vel.Velocity >= vmax):
                    self.robot_vel.Velocity = vmax
            else:
                if ( (abs(self.robot_vel.Turn)) != 0.0 ): 
                    self.robot_vel.Velocity = 0.001 #Al publicar a cmd ocupa que velocity sea mayor a 0 para poder seguir girando
                else:
                    self.robot_vel.Velocity = 0.0    

  
    def cleanup(self):  

        #Esta funcion es ejecutada antes de que el nodo se cierre, se usa para detener al UMA   
        self.robot_vel.Turn = 0
        self.robot_vel.Velocity = 0 
        self.cmd_pub.publish(self.robot_vel)

      

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("gps_navigation_V5", anonymous=True)  

    try:  

        GPSPubClass()  

    except:  

        rospy.logfatal("gps_navigation_V5 died")  
