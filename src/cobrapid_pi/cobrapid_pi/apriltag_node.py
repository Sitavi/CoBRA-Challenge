# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion

import cv2
import numpy as np
#import matplotlib.pyplot as ppl
import dt_apriltags as apriltag
#from picamera2 import Picamera2
import time
import scipy.spatial.transform as scsptr

class AprilTagNode(Node):

    def __init__(self):
        super().__init__('apriltag_node')

        # Créer un publisher qui publie des messages de type apriltag_topic
        self.pose_pub = self.create_publisher(Pose, 'apriltag_topic', 10)

        # Créer un timer pour appeler la méthode publish_pose toutes les 1 seconde
        self.timer = self.create_timer(1.0, self.publish_pose)

        # ---- CAMERA SETTINGS ---- #
        camera_name = "picameraV2"
        width, height = 1640, 1232
        family = "tag36h11"
        tag_size = 159.5
        tag_coordonates_m_init_dict = {0 : [0.0,0.0,0.0], 11: [0.0,1.0,0.0], 12 : [0.0,2.0,0.0]}

        # Initialize camera before starting the tracking thread
        self.get_logger().info("Initializing AprilTag detector...")
        apriltag = Apriltag(camera_name, width, height, family, tag_size, tag_coordonates_m_init_dict)
        self.get_logger().info("Camera initialized successfully!")

    def publish_pose(self):
        # Créer un message Pose
        pose_msg = Pose()

        # Définir la position (x, y, z)
        pose_msg.position = Point(x=1.0, y=2.0, z=3.0)  # Position de l'objet

        # Définir l'orientation (ici un quaternion avec une rotation nulle)
        pose_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Orientation nulle (pas de rotation)

        # Publier le message sur le topic 'position_topic'
        self.pose_pub.publish(pose_msg)
        
        # Log pour montrer que le message a été publié
        self.get_logger().info(f"Position publiée: ({pose_msg.position.x}, {pose_msg.position.y}, {pose_msg.position.z})")
        self.get_logger().info(f"Orientation publiée: ({pose_msg.orientation.x}, {pose_msg.orientation.y}, {pose_msg.orientation.z}, {pose_msg.orientation.w})")


def main(args=None):
    rclpy.init(args=args)

    # Créer une instance du nœud AprilTagNode
    node = AprilTagNode()

    # Exécuter le nœud
    rclpy.spin(node)

    # Lorsque nous sortons de spin(), nous détruisons le nœud
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

class Apriltag:
    def __init__(self, camera_name, image_width_px, image_heigh_px, tag_family, tag_size_mm, tag_coordonates_m_init_dict):
        # Retreive attibutes
        self.camera_name = camera_name
        self.image_width_px = image_width_px
        self.image_heigh_px = image_heigh_px
        self.tag_family = tag_family
        self.tag_size_mm = tag_size_mm
        self.tag_coordonates_m_init_dict = tag_coordonates_m_init_dict

        # Start camera
        #self.picam2 = Picamera2()
        HEIGH,WIDTH=self.image_heigh_px,self.image_width_px
        #self.picam2.configure(self.picam2.create_preview_configuration({'size':(WIDTH,HEIGH)}))
        #self.picam2.start()

        # Load calibration param
        npz_file = f"{camera_name}_{self.image_width_px}_{self.image_heigh_px}_calibration.npz"
        with np.load(npz_file) as data:
            self.intrinsics = data['camera_matrix']
            self.dist_coeffs = data['dist_coefs']
        self.fx=self.intrinsics[0][0]
        self.fy=self.intrinsics[1][1]
        self.cx=self.intrinsics[0][2]
        self.cy=self.intrinsics[1][2]

        # Convert coordonates (tags on the floor on the (x,y) plan)
        self.tag_coordonates_dict = {}
        for id in list(self.tag_coordonates_m_init_dict.keys()):
            c = self.tag_coordonates_m_init_dict[id]
            for i in range(3):
                c[i] = c[i]*1000 # Conversion en mm
            self.tag_coordonates_dict[id] = (
                np.array([[c[0], c[1], c[2]], # coin inférieur gauche
                        [c[0]+self.tag_size_mm, c[1], c[2]], # coin inférieur droit
                        [c[0]+self.tag_size_mm, c[1]+self.tag_size_mm, c[2]], # coin supérieur droit
                        [c[0], c[1]+self.tag_size_mm, c[2]]]) # coin supérieur gauche
                )
            
        # Initialize tag detector
        self.at_detector = apriltag.Detector(families=self.tag_family,nthreads=1,quad_sigma=0.0,refine_edges=1,decode_sharpening=0.25,debug=0)

    # Fonction pour calculer l'aire d'un tag 
    def PolyArea2D(self, pts):
        l = np.hstack([pts, np.roll(pts, -1, axis=0)])
        a = 0.5 * abs(sum(x1 * y2 - x2 * y1 for x1, y1, x2, y2 in l))
        return a

    # Fonction pour représenter la position en 3D
    """def plotCamera3D(self, Cesc, rvec, ax=None):
        if ax is None:
            ax = ppl.axes(projection='3d')
        point = ax.scatter3D(Cesc[0], Cesc[1], Cesc[2], 'k', c='red')
        R, _ = cv2.Rodrigues(rvec)

        p1_cam = [-20, 20, 50]
        p2_cam = [20, 20, 50]
        p3_cam = [20, -20, 50]
        p4_cam = [-20, -20, 50]

        p1_esc = R.T @ p1_cam + Cesc
        p2_esc = R.T @ p2_cam + Cesc
        p3_esc = R.T @ p3_cam + Cesc
        p4_esc = R.T @ p4_cam + Cesc
        camera_plot = [ax.plot3D((Cesc[0], p1_esc[0]), (Cesc[1], p1_esc[1]), (Cesc[2], p1_esc[2]), '-k'),
                    ax.plot3D((Cesc[0], p2_esc[0]), (Cesc[1], p2_esc[1]), (Cesc[2], p2_esc[2]), '-k'),
                    ax.plot3D((Cesc[0], p3_esc[0]), (Cesc[1], p3_esc[1]), (Cesc[2], p3_esc[2]), '-k'),
                    ax.plot3D((Cesc[0], p4_esc[0]), (Cesc[1], p4_esc[1]), (Cesc[2], p4_esc[2]), '-k'),
                    ax.plot3D((p1_esc[0], p2_esc[0]), (p1_esc[1], p2_esc[1]), (p1_esc[2], p2_esc[2]), '-k'),
                    ax.plot3D((p2_esc[0], p3_esc[0]), (p2_esc[1], p3_esc[1]), (p2_esc[2], p3_esc[2]), '-k'),
                    ax.plot3D((p3_esc[0], p4_esc[0]), (p3_esc[1], p4_esc[1]), (p3_esc[2], p4_esc[2]), '-k'),
                    ax.plot3D((p4_esc[0], p1_esc[0]), (p4_esc[1], p1_esc[1]), (p4_esc[2], p1_esc[2]), '-k')]

        return camera_plot, point"""

    # Calcul de la position de la caméra
    def getCamera3D(self, rvec, tvec):
        # Centre optique de la caméra en tant que point 3D exprimé dans le système de la scène
        # t = -R @ Cesc => Cesc = -R^-1 @ t, mais R^-1 = R.T => Cesc = -R.T @ t
        R, _ = cv2.Rodrigues(rvec)
        Cesc = (-R.T @ tvec).reshape(3)

        return Cesc
    
    def get_position(self):

        objectPoints=self.tag_coordonates_dict

        lines = []

        image = self.picam2.capture_array()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
        results=self.at_detector.detect(gray, estimate_tag_pose=True, camera_params=[self.fx,self.fy,self.cx,self.cy], tag_size=0.173) #tag_size=0.052

        coord_fusion = []
        angle_fusion = []
        areas = []
        #print(f"results = {results}")
        for r in results:
            print(r.tag_id)

            if r.tag_id not in self.tag_coordonates_dict:
                print(f"Detected tag n°{r.tag_id} not declared in dictionary")
            else:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                imagePoints = r.corners

                ptA, ptB, ptC, ptD = imagePoints
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))

                areas.append((self.PolyArea2D(imagePoints)))

                _, rotation, translation = cv2.solvePnP(objectPoints[r.tag_id], imagePoints, self.intrinsics, self.dist_coeffs)

                camera = self.getCamera3D(rotation, translation)
                coord_fusion.append(camera)

                R_matrix, _ = cv2.Rodrigues(rotation)
                #euler_angles = scp.spatial.transform.from_matrix(R_matrix).as_euler('xyz', degrees=False)  # Convertit en angles d'Euler (radians)
                #angle_fusion.append(rotation)

                euler_angles = scsptr.Rotation.from_matrix(R_matrix).as_euler('xyz', degrees=False)  # Pour SciPy >= 1.4.0
                angle_fusion.append(euler_angles)

        if len(results) > 0:
            total_weight = np.sum(areas)
            ratio = []
            for area in areas:
                ratio.append(area / total_weight)
            ratio = np.array(ratio)
            coord_fusion = np.array(coord_fusion)
            angle_fusion = np.array(angle_fusion)
            camera = np.array([0, 0, 0])
            sin_angle = np.array([0, 0, 0])
            cos_angle = np.array([0, 0, 0])
            for i in range(len(ratio)):
                camera = camera + (coord_fusion[i] * ratio[i])
                sin_angle = sin_angle + (np.sin(angle_fusion[i]).reshape(3) * ratio[i])
                cos_angle = cos_angle + (np.cos(angle_fusion[i]).reshape(3) * ratio[i])
            angle = np.arctan2(sin_angle, cos_angle)

            pos_xyz = camera
            pos_ang = angle

        else:
            pos_xyz = np.array([np.nan, np.nan, np.nan])
            pos_ang = np.array([np.nan, np.nan, np.nan])
            
        return pos_xyz, pos_ang
