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
        latitude_goal=0.0
        longitude_goal=0.0
        self.option=1
        self.n_goal=0
        self.dist_to_goal=0
        
        #self.option = int(input("UMA empieza recorrido desde el pasillo: "))
        #if self.option is 1: 
           #print('UMA ira al pasillo 1...')
        #elif self.option is 2: 
           #print('UMA ira al pasillo 2...')
        #elif self.option is 3: 
           #print('UMA ira al pasillo 3...')
        #else:
           #print('Esa no es una opcion, por favor elige una que este en las opciones')
           #print(opciones)

        #********** INIT NODE **********###  
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.robot_vel) #Publicando datos a cmd_pub a una frecuencia de 10Hz
            r.sleep()

    def gps_cb(self, msg):  
        home_latitude= 20.614414    #Valor de latitud y longitud  de entrada al invernadero o donde quieres que este home
        home_longitude= -100.403293 
        latlong1 = 0.000013176908  ## 1m se representa como 0.00001317690754 en terminos de  latitude y longitude
        #Se obtienen valores de latitud y longitud del GPS del robot (topico fix)   
        self.latitude_robot= msg.latitude
        self.longitude_robot= msg.longitude + (-0.000009594 * 0.165) #Correccion de longitud por posicion de GPS en el UMA
        #Secuencia en que se recorren los pasillos, option: pasillo, n_goal:puntos de inicio y fin del pasollo
        #Es necesario hacer los calculos de dif_latitude, dif_longitude, dist_to_goal para que se actualicen con los valores del goal que corresponde 
        if self.option is 1: 
            if self.n_goal is 0:
                latitude_goal= 20.6143221529
                longitude_goal= -100.403172748
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.n_goal=1
                    print('goal 1')
            elif self.n_goal is 1:
                latitude_goal= 20.6143221529
                longitude_goal= -100.403288203
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.option = 2
                    self.n_goal=0
                    print('pasillo 1 listo!')
                    
         
        elif self.option is 2: 
            if self.n_goal is 0:
                latitude_goal= 20.6143038042
                longitude_goal= -100.403269016
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.n_goal=1
                    print('goal 2')
            elif self.n_goal is 1:
                latitude_goal= 20.6143038042
                longitude_goal= -100.403149095
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.option = 3
                    self.n_goal = 0
                    print('pasillo 2 listo!')
           
        elif self.option is 3: 
            if self.n_goal is 0:
                latitude_goal= 20.6142861034
                longitude_goal= -100.403172265
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.n_goal=1
                    print('goal 2')
            elif self.n_goal is 1:
                latitude_goal= 20.6142854835
                longitude_goal= -100.403293
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):                
                    self.option = 4
                    self.n_goal = 0
                    print('pasillo 3 listo!')

        elif self.option is 4: 
            if self.n_goal is 0:
                latitude_goal= 20.6142676745
                longitude_goal= -100.403273813
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.n_goal=1
                    print('goal 2')
            elif self.n_goal is 1:
                latitude_goal= 20.6142676745
                longitude_goal= -100.403149094
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.option = 5
                    self.n_goal = 0
                    print('pasillo 4 listo!')

        elif self.option is 5: 
            if self.n_goal is 0:
                latitude_goal= 20.6142496096
                longitude_goal= -100.403168282
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.n_goal=1
                    print('goal 2')
            elif self.n_goal is 1:
                latitude_goal= 20.6142496096
                longitude_goal= -100.403293
                self.dif_latitude= latitude_goal - self.latitude_robot
                self.dif_longitude= longitude_goal - self.longitude_robot
                self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
                if(self.dist_to_goal  <= (latlong1/2.0)):
                    self.option = 0
                    self.n_goal = 2

                    print('pasillo 5 listo!')
                    print('Ruta finalizada, consulta los datos que recopile en la pagina!')
        #Opcion de que se vaya a home, puede ser para cambiar pilas antes de que se le acaben
        elif self.option is 6: 
            latitude_goal=  home_latitude
            longitude_goal= home_longitude 
            self.dif_latitude= latitude_goal - self.latitude_robot
            self.dif_longitude= longitude_goal - self.longitude_robot
            self.dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5 
        else:
            latitude_goal= 0
            longitude_goal= 0
        #Se obtienen valores de latitud y longitud del GPS goal (topico fixgoal) y se calcula la diferencia que hay respecto al goal
     
    
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
       
        #Evaluacion de casos especiales en donde la diferencia de angulo no corresponde al valor necesario para girar hacia el goal. 
        if (angle_goal < -0.07 and angle_goal > -1.50 and yaw > 2.355): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.15
            self.robot_vel.Turn =  kang * (dif_angle + 6.28)*1.4
            if(abs(self.robot_vel.Turn) > 1 ):
                self.robot_vel.Turn = kang * (dif_angle + 6.28)
        elif (angle_goal < -1.64 and yaw > 0.785): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.15
            self.robot_vel.Turn =  kang * (dif_angle + 6.28)*1.4
            if(abs(self.robot_vel.Turn) > 1 ):
                self.robot_vel.Turn =  kang * (dif_angle + 6.28)
        elif (angle_goal > 1.64 and yaw < -0.785): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.15
            self.robot_vel.Turn =  kang * (dif_angle - 6.28)*1.4
            if(abs(self.robot_vel.Turn) > 1 ):
                self.robot_vel.Turn =  kang * (dif_angle - 6.28)
        elif (angle_goal > 0.07 and angle_goal < 1.50 and yaw < -2.355): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.15
            self.robot_vel.Turn =  kang * (dif_angle - 6.28)*1.4
            if(abs(self.robot_vel.Turn) > 1 ):
                self.robot_vel.Turn =  kang * (dif_angle - 6.28)   
        else:
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.15
            self.robot_vel.Turn =  self.robot_vel.Turn =  kang * dif_angle*1.4
            if(abs(self.robot_vel.Turn) > 1 ):
                self.robot_vel.Turn =  self.robot_vel.Turn =  kang * dif_angle
        #Si la velocidad angular de UMA es pequena puede incrementar su velocidad lineal     
        if( (abs(self.robot_vel.Turn)) < 0.1 ): 
            #El UMA se queda a 50cm de distancia de separacion del goal y no se mueva si el objetivo esta a mas de 500m de distancia
            if(self.dist_to_goal  > (latlong/2.0) and self.dist_to_goal  < (latlong*500)):
                self.robot_vel.Velocity = self.robot_vel.Velocity + 0.25 
                if (self.robot_vel.Velocity >= (kdist*self.dist_to_goal) ): 
                    self.robot_vel.Velocity = kdist*self.dist_to_goal
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

    rospy.init_node("recorrer_invernadero", anonymous=True)  

    try:  

        GPSPubClass()  

    except:  

        rospy.logfatal("recorrer_invernadero died")  
