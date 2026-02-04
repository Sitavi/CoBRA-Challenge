import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from rclpy.duration import Duration
from geometry_msgs.msg import Pose

import time

import time
#from adafruit_servokit import ServoKit #pour utiliser composant PCA9685
#kit = ServoKit(channels=16)
import warnings
from time import sleep
import smbus2
import pygame as p
from pyPS4Controller.controller import Controller
from gpiozero import LED
import threading
import numpy as np

class Control(Node):
    def __init__(self):
        super().__init__('control')

        self.target_points = [[1.0, 12, 0], [4.0, 9.0, 0], [2.5, 2.5, 0]]
        self.next_point_index = 0
        self.end_of_mission = False

        ##############################################################################################################################
        #################### Define Control Values ###################################################################################
        ##############################################################################################################################

        # Sampling time
        Ts = 0.1

        # Altitude control
        self.Z_setpoint = 2

        P = 20.0
        I = 1.3
        D = 93.6
        N = 0.357
        self.a_z = [P+D*N, P*N*Ts-2*P+I*Ts-2*D*N, P-P*N*Ts+I*N*Ts*Ts-I*Ts+D*N]
        self.b_z = [1, N*Ts-2, 1-N*Ts]
        self.err_z = [0,0,0]
        self.commande_z = [0,0,0]

        # Rotation control
        self.YAW_setpoint = 0

        P = 4
        I = 0.00773
        D = 15
        N = 15.72
        self.a_yaw = [P+D*N, P*N*Ts-2*P+I*Ts-2*D*N, P-P*N*Ts+I*N*Ts*Ts-I*Ts+D*N]
        self.b_yaw = [1, N*Ts-2, 1-N*Ts]
        self.err_yaw = [0,0,0]
        self.commande_yaw = [0,0,0]

        # Translation control
        self.commande_translation = [0,0,0]

        ##############################################################################################################################
        ##############################################################################################################################
        ##############################################################################################################################

        self.pose_received_time = None
        self.orientation_received_time = None
        self.pose_timeout = Duration(seconds=Ts)
        self.latest_pose = Vector3Stamped().vector
        self.latest_pose.x = 0.0
        self.latest_pose.y = 0.0
        self.latest_pose.z = 0.0
        self.latest_orientation = Vector3Stamped().vector
        self.latest_orientation.x = 0.0
        self.latest_orientation.y = 0.0
        self.latest_orientation.z = 0.0
        self.latest_pose_tf_luna_z = None
        self.latest_pose_bno_055 =  None

        self.subscription_pose = self.create_subscription(
            Vector3Stamped,
            '/pose_xyz',
            self.pose_callback,
            10  # Hz
        )

        self.subscription_orientation = self.create_subscription(
            Vector3Stamped,
            '/orientation_rpy',
            self.orientation_callback,
            10  # Hz
        )

        self.timer = self.create_timer(Ts, self.timer_callback)

        myi2cbus = smbus2.SMBus(1) # Initialisation du bus I2C
        myPCA9685 = PCA9685(myi2cbus) # Initialisation du générateur de PWM

        self.brushless = {}
        self.brushless["d"] = Brushless(myPCA9685,1)
        self.brushless["g"] = Brushless(myPCA9685,2)
        self.brushless["av"] = Brushless(myPCA9685,3)
        self.brushless["ar"] = Brushless(myPCA9685,4)
        self.pince = Pince(myPCA9685, 6)
        self.treuil = Treuil(myPCA9685, 7)

        self.first_tag = True  

        self.tf_luna = LidarTFLuna(myi2cbus)
        self.bno_055 = BNO055(myi2cbus)

        self.bno_055.calibration()
        #self.bno_055.calibrate_zero()

        self.brushless["d"].cmd_vit_pourcent(1)
        self.brushless["g"].cmd_vit_pourcent(1)
        self.brushless["av"].cmd_vit_pourcent(1)
        self.brushless["ar"].cmd_vit_pourcent(1)
        self.treuil.cmd_vit_pourcent(1)
        self.pince.cmd_vit_pourcent(1)

        # Attributs d'état de la mannette
        self.controller_state = {
            'L1': 0,
            'R1': 0,
            'L2': 0, 
            'R2': 0,
            'L3_x': 0,
            'L3_y': 0,
            'R3_x': 0,
            'R3_y': 0,
            'stop': 1,
            'controlling_mode':0,
            'up_arrow': 0,
            'down_arrow': 0,
            'left_arrow': 0,
            'right_arrow': 0,
            'ps_button': 0
        }

        self.prev_controller_state = {
            'L1': 0,
            'R1': 0,
            'L2': 0, 
            'R2': 0,
            'L3_x': 0,
            'L3_y': 0,
            'R3_x': 0,
            'R3_y': 0,
            'stop': 1,
            'controlling_mode':0,
            'up_arrow': 0,
            'down_arrow': 0,
            'left_arrow': 0,
            'right_arrow': 0,
            'ps_button': 0
        }

        # Parameters for MyController
        controller_params = {
            "interface": "/dev/input/js0",
            "connecting_using_ds4drv": False,
            "brushless": self.brushless,
            "pince": self.pince,
            "treuil": self.treuil,
            "controller_state": self.controller_state,
            "logger": self.get_logger()
        }

        # Start a separate thread for the blocking function
        self.controller_thread = threading.Thread(target=self.controller_logic, args=(controller_params,))
        self.controller_thread.start()



        self.get_logger().info("Prêt - Initialisation réalisée")

    def controller_logic(self, controller_params):
        controller = MyController(**controller_params)
        controller.listen()

    def pose_callback(self, msg: Vector3Stamped):
        self.pose_received_time = self.get_clock().now()
        self.latest_pose = msg.vector  # Store the vector
        self.get_logger().debug('Pose received.')

    def orientation_callback(self, msg: Vector3Stamped):
        self.orientation_received_time = self.get_clock().now()
        self.latest_orientation = msg.vector

    def update_setpoints(self):

        match self.controller_state["controlling_mode"]:
            #self.Z_setpoint = 1.8
            case 0:  # Manual mode
                # Update translation
                self.commande_translation[0] = -self.controller_state["R3_y"]/2
                self.commande_yaw[0] = self.controller_state["R3_x"]/2
                self.commande_z[0] = self.controller_state["L3_y"]/2

            case 1: # mode asservi Z et YAW
                # Update Z axis : keep controller
                if self.prev_controller_state["L3_y"] == 0.0 and not self.controller_state["L3_y"] == 0.0 :
                    if(self.controller_state["L3_y"]  > 0.0 and self.Z_setpoint < 8.0):
                        self.Z_setpoint += 0.1
                    elif(self.controller_state["L3_y"] < 0.0 and self.Z_setpoint > 0.1):
                        self.Z_setpoint -= 0.1

                self.get_logger().info(f"Z setpoint = {self.Z_setpoint}")
                self.prev_controller_state["L3_y"] = self.controller_state["L3_y"]

                # Update translation
                self.commande_translation[0] = -self.controller_state["R3_y"]/2

                # Update Yaw
                if self.prev_controller_state["L3_x"] == 0.0 and not self.controller_state["L3_x"] == 0.0 :
                    if(self.controller_state["L3_x"] > 0.0):
                        self.YAW_setpoint -= 5
                    elif(self.controller_state["L3_x"] < 0.0):
                        self.YAW_setpoint += 5

                    self.YAW_setpoint = (self.YAW_setpoint + 180) % 360 - 180

                self.get_logger().info(f"YAW setpoint = {self.YAW_setpoint}")
                self.prev_controller_state["L3_x"] = self.controller_state["L3_x"]
        
            case 2: # mode asservi 3 points

                if(self.end_of_mission):
                    self.Z_setpoint = 1

                    # préhension
                    if self.distance_next_point <= 0.2:
                        pass

                else:
                    self.distance_next_point = np.sqrt((self.latest_pose.x-self.target_points[self.next_point_index][0])**2 + (self.latest_pose.y-self.target_points[self.next_point_index][1])**2)
                    if(self.distance_next_point < 1.0 and not self.next_point_index == 2):
                        self.next_point_index += 1
                    elif(self.distance_next_point < 1.0 and self.next_point_index == 2):
                        #self.end_of_mission = True
                        pass


                    
                    # Update Z axis : keep controller
                    if(self.next_point_index == 2):
                        self.Z_setpoint = max(min(self.distance_next_point, 2),1)
                    else:
                        self.Z_setpoint = 2
                    
                    self.get_logger().info(f"Z setpoint = {self.Z_setpoint}")
                    self.get_logger().info(f"YAW setpoint = {self.YAW_setpoint}")

                    # Update YAW
                    self.YAW_setpoint = 180 - np.arctan2(self.latest_pose.x-self.target_points[self.next_point_index][0], self.latest_pose.y-self.target_points[self.next_point_index][1])*180/np.pi

                    # Update translation
                    if(self.distance_next_point < 0.5):
                        self.commande_translation[0] = 0
                    else:
                        self.commande_translation[0] = min(max(-(self.distance_next_point*5), -25), 25)


        # Update treuil
        if self.controller_state["down_arrow"] == 1:
            self.commande_treuil = -100
        elif self.controller_state["up_arrow"] == 1:
            self.commande_treuil = 100
        else:
            self.commande_treuil = 0

        # Update pince
        if self.controller_state["left_arrow"] == 1:
            self.commande_pince = -100
        elif self.controller_state["right_arrow"] == 1:
            self.commande_pince = 100
        else:
            self.commande_pince = 0

    def timer_callback(self):
        now = self.get_clock().now()

        self.update_setpoints()
        self.latest_pose_tf_luna_z = self.tf_luna.read_distance()/100
        self.latest_pose_bno_055 = self.bno_055.read_offsetted_euler()

        if self.controller_state['L1'] == 1:
            self.bno_055.calibrate_zero()


        if self.controller_state["stop"] == 1:
            self.brushless["d"].cmd_vit_pourcent(0)
            self.brushless["g"].cmd_vit_pourcent(0)
            self.brushless["av"].cmd_vit_pourcent(0)
            self.brushless["ar"].cmd_vit_pourcent(0)
            self.treuil.cmd_vit_pourcent(0)
            self.pince.cmd_vit_pourcent(0)

        else:  
            self.get_logger().info(f"GOING TO POINT {self.next_point_index}")


            if (self.pose_received_time is not None and (now - self.pose_received_time) < self.pose_timeout):
                self.pose_z = self.latest_pose.z
                self.pose_yaw = self.latest_orientation.z

                if self.first_tag:
                    pass
                    #self.bno_055.calibrate(self.latest_orientation.z)
                    #self.get_logger().info(f"Initialized BNO with {self.latest_orientation.z}")
                self.first_tag = False

            else:
                self.get_logger().info('No visible Apriltags')
                self.pose_z = self.latest_pose_tf_luna_z
                self.pose_yaw = self.latest_pose_bno_055['yaw']
                #self.first_tag = True
        

            match self.controller_state["controlling_mode"]:

                case 0:  # Manual mode 
                    self.get_logger().info(f"Latest Z pose received: {self.pose_z}")
                    self.get_logger().info(f"Latest YAW orientation received: {self.pose_yaw}")

                case 1:  # Z and YAW control mode
                    ####### Z control #######
                    self.get_logger().info(f"Latest Z pose received: {self.pose_z}")

                    self.err_z[0] = self.Z_setpoint - self.pose_z

                    self.commande_z[0] = self.a_z[0]*self.err_z[0] + self.a_z[1]*self.err_z[1] + self.a_z[2]*self.err_z[2] - self.b_z[1]*self.commande_z[1] - self.b_z[2]*self.commande_z[2]

                    self.err_z[2] = self.err_z[1]
                    self.err_z[1] = self.err_z[0]

                    self.commande_z[2] = self.commande_z[1]
                    self.commande_z[1] = self.commande_z[0]

                    ####### YAW control #######

                    self.get_logger().info(f"Latest YAW orientation received: {self.pose_yaw}")

                    self.err_yaw[0] = ((self.YAW_setpoint - self.pose_yaw + 180) % 360) - 180

                    self.commande_yaw[0] = self.a_yaw[0]*self.err_yaw[0] + self.a_yaw[1]*self.err_yaw[1] + self.a_yaw[2]*self.err_yaw[2] - self.b_yaw[1]*self.commande_yaw[1] - self.b_yaw[2]*self.commande_yaw[2]

                    self.err_yaw[2] = self.err_yaw[1]
                    self.err_yaw[1] = self.err_yaw[0]

                    self.commande_yaw[2] = self.commande_yaw[1]
                    self.commande_yaw[1] = self.commande_yaw[0]

                case 2: # 3 points control mode
                    ####### Z control #######
                    self.get_logger().info(f"Latest Z pose received: {self.pose_z}")

                    self.err_z[0] = self.Z_setpoint - self.pose_z

                    self.commande_z[0] = self.a_z[0]*self.err_z[0] + self.a_z[1]*self.err_z[1] + self.a_z[2]*self.err_z[2] - self.b_z[1]*self.commande_z[1] - self.b_z[2]*self.commande_z[2]

                    self.err_z[2] = self.err_z[1]
                    self.err_z[1] = self.err_z[0]

                    self.commande_z[2] = self.commande_z[1]
                    self.commande_z[1] = self.commande_z[0]

                    ####### YAW control #######

                    self.get_logger().info(f"Latest YAW orientation received: {self.pose_yaw}")

                    self.err_yaw[0] = ((self.YAW_setpoint - self.pose_yaw + 180) % 360) - 180

                    self.commande_yaw[0] = self.a_yaw[0]*self.err_yaw[0] + self.a_yaw[1]*self.err_yaw[1] + self.a_yaw[2]*self.err_yaw[2] - self.b_yaw[1]*self.commande_yaw[1] - self.b_yaw[2]*self.commande_yaw[2]

                    self.err_yaw[2] = self.err_yaw[1]
                    self.err_yaw[1] = self.err_yaw[0]

                    self.commande_yaw[2] = self.commande_yaw[1]
                    self.commande_yaw[1] = self.commande_yaw[0]
            # Motors settings

            self.brushless["av"].cmd_vit_pourcent(max(-100, min(self.commande_z[0], 100)))
            self.brushless["ar"].cmd_vit_pourcent(max(-100, min(self.commande_z[0], 100)))

            self.get_logger().info(f"Commande Z: {self.commande_z[0]}")

            self.brushless["g"].cmd_vit_pourcent(max(-60, min(self.commande_yaw[0]+self.commande_translation[0], 60)))
            self.brushless["d"].cmd_vit_pourcent(max(-60, min(-self.commande_yaw[0]+self.commande_translation[0], 60)))

            self.get_logger().info(f"COMMANDE TREUIL={self.commande_treuil}")
            self.treuil.cmd_vit_pourcent(self.commande_treuil)
            self.pince.cmd_vit_pourcent(self.commande_pince)

def main(args=None):
    rclpy.init(args=args)
    node = Control()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Program interrupted by user (Ctrl+C)")
    finally:
        print("[INFO] Stopping all motors...")

        # Stop all brushless motors
        for key, motor in node.brushless.items():
            motor.stop_mot()
            print(f"[INFO] Motor '{key}' stopped.")

        node.pince.cmd_vit_pourcent(0)
        node.treuil.cmd_vit_pourcent(0)

        node.destroy_node()
        rclpy.shutdown()
        print("[INFO] Shutdown complete.")


if __name__ == '__main__':
    main()

##############################################################################################################################
#################### Classes de controle des moteurs #########################################################################
##############################################################################################################################

class PCA9685:  # pour commander les sorties pwm
    def __init__(self, bus, address_PCA9685=0x40):
        self.available = False
        try:
            # DÉFINITION des registres (valeurs initiales)
            self.address_PCA9685 = address_PCA9685
            self.bus = bus
            self.MODE1 = 0x00  # self.REGISTRE = adresse_registre
            self.MODE2 = 0x01

            # liste de dictionnaires. Chaque dictionnaire definit les registres d'une sortie PWM de la PCA9685
            self.PWMs = [{"ON_L": 0x06, "ON_H": 0x07, "OFF_L": 0x08, "OFF_H": 0x09},
                         {"ON_L": 0x0A, "ON_H": 0x0B, "OFF_L": 0x0C, "OFF_H": 0x0D},
                         {"ON_L": 0x0E, "ON_H": 0x0F, "OFF_L": 0x10, "OFF_H": 0x11},
                         {"ON_L": 0x12, "ON_H": 0x13, "OFF_L": 0x14, "OFF_H": 0x15},
                         {"ON_L": 0x16, "ON_H": 0x17, "OFF_L": 0x18, "OFF_H": 0x19},
                         {"ON_L": 0x1A, "ON_H": 0x1B, "OFF_L": 0x1C, "OFF_H": 0x1D},
                         {"ON_L": 0x1E, "ON_H": 0x1F, "OFF_L": 0x20, "OFF_H": 0x21},
                         {"ON_L": 0x22, "ON_H": 0x23, "OFF_L": 0x24, "OFF_H": 0x25},
                         {"ON_L": 0x26, "ON_H": 0x27, "OFF_L": 0x28, "OFF_H": 0x29}]

            self.PRE_SCALE = 0xFE  # Numero du registre qui code la periode
            self.freqPWM_Hz = 50  # Frequence des PWM
            self.periodPWM = 1 / self.freqPWM_Hz  # 20 millisecondes
            # self.PCA9685_PRE_SCALE_VALUE = (round(25*10**6/(4096*self.freqPWM_Hz)-1)) #32  # configure la fréquence de la PWM sur tous les canaux
            self.PCA9685_PRE_SCALE_VALUE = 125

            # CONFIGURATION des registres du PCA9685
            self.bus.write_byte_data(self.address_PCA9685, self.MODE1, 0x10)
            self.bus.write_byte_data(self.address_PCA9685, self.PRE_SCALE, self.PCA9685_PRE_SCALE_VALUE)
            self.bus.write_byte_data(self.address_PCA9685, self.MODE1, 0x00)
            self.bus.write_byte_data(self.address_PCA9685, self.MODE2, 0x04)

            # Intervalles d’allumage et d’extinction (largeur d'impulsion) pour chaque PWM de self.PWMs.
            for PWM in self.PWMs:
                self.bus.write_byte_data(self.address_PCA9685, PWM["ON_L"], 0)
                self.bus.write_byte_data(self.address_PCA9685, PWM["ON_H"], 0x0)
                self.bus.write_byte_data(self.address_PCA9685, PWM["OFF_L"], 0x40)
                self.bus.write_byte_data(self.address_PCA9685, PWM["OFF_H"], 1)

            self.available = True

        except OSError as e:
            print(f"[ERROR] I2C communication error during PCA9685 initialization: {e}")
            self.available = False

    def commande_moteur_vitesse_us(self, temps_off_us, num_PWM):
        if not self.available:
            print("[WARNING] PCA9685 device not available. commande_moteur_vitesse_us ignored.")
            return
        points_off = temps_off_us * 4095 / (self.periodPWM * 1e6)
        temps_H = int(points_off // 256)
        temps_L = int(points_off % 256)
        try:
            self.bus.write_byte_data(self.address_PCA9685, self.PWMs[num_PWM]["OFF_L"], temps_L)
            self.bus.write_byte_data(self.address_PCA9685, self.PWMs[num_PWM]["OFF_H"], temps_H)
        except OSError as e:
            print(f"[ERROR] I2C communication error during commande_moteur_vitesse_us: {e}")


class Brushless:
    # PCA9685    : instance de la classe PCA9685 utilisee
    # num_moteur : numero de la PWM de la PCA9685
    def __init__(self, PCA9685, num_moteur):
        self.num_moteur = num_moteur
        self.myPCA9685 = PCA9685
        self.valeur_repos = 1500  # valeur milieu de la commande => vitesse nulle
        self.cmd_vit_pourcent(0)

    def stop_mot(self):
        if not self.myPCA9685.available:
            print("[WARNING] PCA9685 device not available. stop_mot ignored.")
            return
        self.myPCA9685.commande_moteur_vitesse_us(0, self.num_moteur)

    def cmd_vit_pourcent(self, vitesse_pourcent):
        if not self.myPCA9685.available:
            print("[WARNING] PCA9685 device not available. cmd_vit_pourcent ignored.")
            return

        if vitesse_pourcent < -100:
            vitesse_pourcent = -100
        if vitesse_pourcent > 100:
            vitesse_pourcent = 100
        if 0 < vitesse_pourcent < 9:
            vitesse_pourcent = 4.5 + vitesse_pourcent / 2
        if 0 > vitesse_pourcent > -9:
            vitesse_pourcent = -4.5 + vitesse_pourcent / 2

        temps_etat_haut_us = self.valeur_repos + vitesse_pourcent * 5
        try:
            self.myPCA9685.commande_moteur_vitesse_us(temps_etat_haut_us, self.num_moteur)
        except OSError as e:
            print(f"[ERROR] I2C communication error in brushless cmd_vit_pourcent: {e}")


class Treuil:
    # PCA9685    : instance de la classe PCA9685 utilisee
    # num_PWM    : numero de la PWM de la PCA9685 connectee a la modulation de vitesse

    def __init__(self, PCA9685, num_PWM):
        self.num_PWM = num_PWM
        self.myPCA9685 = PCA9685
        self.P_horaire = LED(24)
        self.P_antihoraire = LED(23)
        self.cmd_vit_pourcent(0)
    
    def stop(self):
        self.myPCA9685.commande_moteur_vitesse_us(0,self.num_PWM)
        self.P_horaire.off()
        self.P_antihoraire.off()

    def cmd_vit_pourcent(self, vitesse_pourcent):
        if vitesse_pourcent >= 0:
            if vitesse_pourcent > 100:
                vitesse_pourcent = 100
            self.P_horaire.on()
            self.P_antihoraire.off()
            temps_etat_haut_us=200*vitesse_pourcent
            self.myPCA9685.commande_moteur_vitesse_us(temps_etat_haut_us,self.num_PWM)
        else:
            if vitesse_pourcent < -100:
                vitesse_pourcent = -100
            self.P_horaire.off()
            self.P_antihoraire.on()
            temps_etat_haut_us=-200*vitesse_pourcent
            self.myPCA9685.commande_moteur_vitesse_us(temps_etat_haut_us,self.num_PWM)
            
    def kill(self):
        self.P_horaire.close()
        self.P_antihoraire.close()

class Pince:
    # PCA9685    : instance de la classe PCA9685 utilisee
    # num_PWM    : numero de la PWM de la PCA9685 connectee a la modulation de vitesse

    def __init__(self, PCA9685, num_PWM):
        self.num_PWM = num_PWM
        self.myPCA9685 = PCA9685
        self.T_horaire = LED(8)
        self.T_antihoraire = LED(7)
        self.cmd_vit_pourcent(0)

    def stop(self):
        self.myPCA9685.commande_moteur_vitesse_us(0,self.num_PWM)
        self.T_horaire.off()
        self.T_antihoraire.off()
    
    def cmd_vit_pourcent(self, vitesse_pourcent):
        if vitesse_pourcent >= 0:
            if vitesse_pourcent > 100:
                vitesse_pourcent = 100
            self.T_horaire.on()
            self.T_antihoraire.off()
            temps_etat_haut_us=200*vitesse_pourcent
            self.myPCA9685.commande_moteur_vitesse_us(temps_etat_haut_us,self.num_PWM)
        else:
            if vitesse_pourcent < -100:
                vitesse_pourcent = -100
            self.T_horaire.off()
            self.T_antihoraire.on()
            temps_etat_haut_us=-200*vitesse_pourcent
            self.myPCA9685.commande_moteur_vitesse_us(temps_etat_haut_us,self.num_PWM)
    
    def kill(self):
        self.T_horaire.close()
        self.T_antihoraire.close()

##############################################################################################################################
######################## Classe de controle du TF LUNA #######################################################################
##############################################################################################################################

class LidarTFLuna: # pour lire la distance mesurée par le LiDAR TF Luna en utilisant le protocole I2C
    def __init__(self, bus, i2c_address=0x10):
        # Initialiser le bus I2C
        self.address = i2c_address
        self.bus = bus
    
    def read_distance(self):
        # Le TF Luna envoie les données de distance en 2 octets
        try:
            # Lire les 2 bytes (octets) de données de distance
            distance_data = self.bus.read_i2c_block_data(self.address, 0x00, 2)  #arguments: adresse unique du périphérique, adresse du premier registre à lire,  nbre de bytes à lire
            # Combiner les 2 octets en une seule valeur de distance en cm
            distance = (distance_data[0] + (distance_data[1] << 8))
            return distance
        except Exception as e:
            print(f"Erreur de lecture du LiDAR TF Luna : {e}")
            return None

##############################################################################################################################
#################### Classe de controle de la mannette #######################################################################
##############################################################################################################################

class MyController(Controller):
    def __init__(self, **kwargs):
        # Extract the parameters for the parent Controller class
        controller_kwargs = {
            'interface': kwargs.get('interface', '/dev/input/js0'),
            'connecting_using_ds4drv': kwargs.get('connecting_using_ds4drv', False)
        }

        # Initialize the parent Controller class with only the expected arguments
        super().__init__(**controller_kwargs)

        # Store additional parameters as instance variables
        self.brushless = kwargs.get('brushless', {})
        self.treuil = kwargs.get('treuil', {})
        self.pince = kwargs.get('pince', {})
        self.vitesse_moy = 0
        self.vitesse_diff = 0
        self.arret = True
        self.controller_state = kwargs.get('controller_state', {})
        self.logger = kwargs.get('logger', {})

    def on_L2_press(self, value):
        self.controller_state['L2'] = value
        
    def on_R2_press(self,value):
        self.controller_state['R2'] = value
    
    def on_R2_release(self):
        self.controller_state['R2'] = 0

    def on_L2_release(self):
        self.controller_state['L2'] = 0
 
    def on_L3_x_at_rest(self):
        pass
        
    def on_R1_press(self):
        self.controller_state['R1'] = 1
        
    def on_R1_release(self):
        self.controller_state['R1'] = 0
    
    def on_L3_right(self,value): #tourner à droite
        if value<16000 :
            value = 0
        value = value*10/3200
        if value >100 :
            value = 100

        self.controller_state['L3_x'] = value

    def on_L3_left(self,value): #tourner à gauche
        if value>-16000 :
            value = 0
        value = value*10/3200
        if value <-100 :
            value = -100
        
        self.controller_state['L3_x'] = value

    def on_L3_up(self,value): # monter
        value = -value
        deadzone = 3200 if self.controller_state['controlling_mode'] == 0 else 16000

        if value<deadzone :
            value = 0
        value = value*10/3200 # échelonnage sur 100 au lieu de 32000
        if value >100 :
            value = 100

        self.controller_state['L3_y'] = value

    def on_L3_down(self,value): # descendre
        value = -value
        deadzone = 3200 if self.controller_state['controlling_mode'] == 0 else 16000

        if value>-deadzone :
            value = 0
        value = value*10/3200 # échelonnage sur 100 au lieu de 32000
        if value <-100 :
            value = -100

        self.controller_state['L3_y'] = value

    def on_R3_up(self,value): # avancer
        value = -value

        if value<3200 :
            value = 0
        value = value*10/3200
        if value >100 :
            value = 100

        self.controller_state['R3_y'] = value
        #self.logger.info(f"AVANCER : {value}")

    
    def on_R3_down(self,value): # reculer
        value = -value

        if value>-3200 :
            value = 0
        value = value*10/3200
        if value <-100 :
            value = -100

        self.controller_state['R3_y'] = value
        #self.logger.info(f"RECULER : {value}")

        
    def on_x_press(self): # stop
        self.controller_state['stop'] = 1

    def on_x_release(self):
        pass

    def on_circle_press(self): # marche
        self.controller_state['stop'] = 0

    def on_circle_release(self):
        pass

    def on_triangle_press(self):
        self.controller_state["controlling_mode"] = 0

    def on_triangle_release(self):
        pass

    def on_R3_x_at_rest(self):
        self.controller_state['R3_x'] = 0

    def on_R3_right(self,value):
        if value<3200 :
            value = 0
        value = value*10/3200
        if value >100 :
            value = 100

        self.controller_state['R3_x'] = -value

    def on_R3_left(self,value):

        if value>-3200 :
            value = 0
        value = value*10/3200
        if value <-100 :
            value = -100

        self.controller_state['R3_x'] = -value

    def on_L1_press(self):
        self.controller_state['L1'] = 1

    def on_L1_release(self):
        self.controller_state['L1'] = 0

    # def on_L3_y_at_rest(self):
    #     pass

    # def on_R3_y_at_rest(self):
    #     pass

    def on_playstation_button_press(self):
        self.controller_state['ps_button'] = 1
        self.controller_state['controlling_mode'] = 2  # full asservissement, passage par 3 points

    def on_playstation_button_release(self):
        self.controller_state['ps_button'] = 0

    def on_square_press(self):
        self.controller_state["controlling_mode"] = 1 # mode asservissement Z et YAW
    
    def on_square_release(self):
        pass

    # TREUIL
    def on_up_arrow_press(self):
        self.controller_state['up_arrow'] = 1            

    def on_down_arrow_press(self):
        self.controller_state['down_arrow'] = 1
    
    def on_up_down_arrow_release(self):
        self.controller_state['up_arrow'] = 0
        self.controller_state['down_arrow'] = 0

    # PINCE
    def on_right_arrow_press(self):
        self.controller_state['right_arrow'] = 1

    def on_left_arrow_press(self):
        self.controller_state['left_arrow'] = 1
    
    def on_left_right_arrow_release(self):
        self.controller_state['right_arrow'] = 0
        self.controller_state['left_arrow'] = 0


class BNO055():
    def __init__(self, bus,i2c_address=0x28):
        self.address = i2c_address
        self.bus = bus

        # OFFSETS
        self.pitch_offset = 0
        self.roll_offset = 0
        self.yaw_offset = 0

        self.bus.write_byte_data(0x28,0x07,1)
        data = self.bus.read_byte_data(0x28,0x50)
        # print("0x50 du BNO055 : ",data)
        self.bus.write_byte_data(0x28,0x07,0)
        data = self.bus.read_byte_data(0x28,0x35)
        #Config en mode fusion
        self.bus.write_byte_data(0x28,0x08,0x08)
        self.bus.write_byte_data(0x28,0x0A,0x23)
        self.bus.write_byte_data(0x28,0x0B,0x00)
        self.bus.write_byte_data(0x28,0x09,0x1B)
        self.bus.write_byte_data(0x28,0x07,0)
        self.bus.write_byte_data(0x28,0x40,0x01)
        self.bus.write_byte_data(0x28,0x3B,0x01)
        self.bus.write_byte_data(0x28,0x3E,0x00)
        self.bus.write_byte_data(0x28,0x3D,0x0C)

    def calibration(self):
        
        print("Faire des 8 avec le capteur")
        n=0
        t0=time.time()
        while True :
            time.sleep(0.1)
            t1 = time.time()
            data = self.bus.read_byte_data(0x28,0x35)
            
            if (data & 0xC0 >>6)==3 and  (data & 0xC0 >>6)==3 and (data & 0xC0 >>6)==3 and  (data & 0xC0 >>6)==3 :
                print("Calibration réussie")
                break
            if (t1-t0)>5:
                print("Erreur de calibration")
                break

    def calibrate_zero(self):
        pitch_sum = roll_sum = yaw_sum = 0
        samples = 5
        
        for _ in range(samples):
            angles = self.read_euler()
            pitch_sum += angles['pitch']
            roll_sum += angles['roll']
            yaw_sum += angles['yaw']
            time.sleep(0.1)
        
        # Calculer et stocker les offsets
        self.pitch_offset = pitch_sum / samples
        self.roll_offset = roll_sum / samples
        self.yaw_offset = yaw_sum / samples 
        #self.yaw_offset = yaw_sum / samples - calibrate_yaw
        
        print(f"Position zéro calibrée: pitch={self.pitch_offset:.2f}°, roll={self.roll_offset:.2f}°, yaw={self.yaw_offset:.2f}°")

    def read_euler(self):
        registres_lus = self.bus.read_i2c_block_data(0x28,0x1A,6) 
        
        
        data={}
        data1 = registres_lus[0x1E-0x1A]
        data2 = registres_lus[0x1F-0x1A]
        data3=int(data2<<8)+data1
        if data3 > 32767 :
                data3 = data3 - 65536
        data["pitch"]= float(data3)/16
        #print(data1,data2,data["pitch"])
        data1 = registres_lus[0x1C-0x1A]
        data2 = registres_lus[0x1D-0x1A]
        data3=int(data2<<8)+data1
        if data3 > 32767 :
                data3 = data3 - 65536
        data["roll"]= float(data3)/16
        #print(data1,data2,data["roll"])
        data1 = registres_lus[0x1A-0x1A]
        data2 = registres_lus[0x1B-0x1A]
        data3=int(data2<<8)+data1
        if data3 > 32767 :
                data3 = data3 - 65536
        data["yaw"]= float(data3)/16

        #print(data1,data2,data["heading"])
        return data
    
    def read_offsetted_euler(self):
        angles = self.read_euler()
        
        angles['pitch'] -= self.pitch_offset
        angles['roll'] -= self.roll_offset
        angles['yaw'] -= self.yaw_offset
        
        angles['pitch'] = (angles['pitch'] + 180) % 360 - 180
        angles['roll'] = (angles['roll'] + 180) % 360 - 180
        angles['yaw'] = -((angles['yaw'] + 180) % 360 - 180)
        
        return angles
    
    def read_acceleration(self):
        data = self.bus.read_i2c_block_data(self.address, 0x08, 6)
        acc = {}
        for i, axis in enumerate(['x', 'y', 'z']):
                lsb = data[2*i]
                msb = data[2*i + 1]
                value = (msb << 8) | lsb
                if value > 32767:
                        value -= 65536

                acc[axis] = value * 0.00981  # à adapter si autre échelle
        return acc
    def read_linear_acceleration(self):
        data = self.bus.read_i2c_block_data(self.address, 0x28, 6)
        acc = {}
        for i, axis in enumerate(['x', 'y', 'z']):
                lsb = data[2 * i]
                msb = data[2 * i + 1]
                value = (msb << 8) | lsb
                if value > 32767:
                        value -= 65536
                acc[axis] = value * 0.00981  # conversion mg → m/s²
        return acc
