import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, TransformBroadcaster, LookupException
from scipy.spatial.transform import Rotation as R
import tf_transformations
import numpy as np
from geometry_msgs.msg import Vector3Stamped

class CameraTagPoseBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tag_pose_broadcaster')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.position_pub = self.create_publisher(Vector3Stamped, "/pose_xyz", 10)
        self.orientation_pub = self.create_publisher(Vector3Stamped, "/orientation_rpy", 10)

        # Tags connus et leur pose dans le monde (à adapter selon votre config)

        listePoints3D = {

            84: (0.0, 14.0, 0.0), 85: (1.0, 14.0, 0.0), 86: (2.0, 14.0, 0.0), 87: (3.0, 14.0, 0.0), 88: (4.0, 14.0, 0.0), 89: (5.0, 14.0, 0.0),
            78: (0.0, 13.0, 0.0), 79: (1.0, 13.0, 0.0), 80: (2.0, 13.0, 0.0), 81: (3.0, 13.0, 0.0), 82: (4.0, 13.0, 0.0), 83: (5.0, 13.0, 0.0),
            72: (0.0, 12.0, 0.0), 73: (1.0, 12.0, 0.0), 74: (2.0, 12.0, 0.0), 75: (3.0, 12.0, 0.0), 76: (4.0, 12.0, 0.0), 77: (5.0, 12.0, 0.0),
            66: (0.0, 11.0, 0.0), 67: (1.0, 11.0, 0.0), 68: (2.0, 11.0, 0.0), 69: (3.0, 11.0, 0.0), 70: (4.0, 11.0, 0.0), 71: (5.0, 11.0, 0.0),
            60: (0.0, 10.0, 0.0), 61: (1.0, 10.0, 0.0), 62: (2.0, 10.0, 0.0), 63: (3.0, 10.0, 0.0), 64: (4.0, 10.0, 0.0), 65: (5.0, 10.0, 0.0),
            54: (0.0, 9.0, 0.0), 55: (1.0, 9.0, 0.0), 56: (2.0, 9.0, 0.0), 57: (3.0, 9.0, 0.0), 58: (4.0, 9.0, 0.0), 59: (5.0, 9.0, 0.0),
            48: (0.0, 8.0, 0.0), 49: (1.0, 8.0, 0.0), 50: (2.0, 8.0, 0.0), 51: (3.0, 8.0, 0.0), 52: (4.0, 8.0, 0.0), 53: (5.0, 8.0, 0.0),
            42: (0.0, 7.0, 0.0), 43: (1.0, 7.0, 0.0), 44: (2.0, 7.0, 0.0), 45: (3.0, 7.0, 0.0), 46: (4.0, 7.0, 0.0), 47: (5.0, 7.0, 0.0),
            36: (0.0, 6.0, 0.0), 37: (1.0, 6.0, 0.0), 38: (2.0, 6.0, 0.0), 39: (3.0, 6.0, 0.0), 40: (4.0, 6.0, 0.0), 41: (5.0, 6.0, 0.0),
            30: (0.0, 5.0, 0.0), 31: (1.0, 5.0, 0.0), 32: (2.0, 5.0, 0.0), 33: (3.0, 5.0, 0.0), 34: (4.0, 5.0, 0.0), 35: (5.0, 5.0, 0.0),
            24: (0.0, 4.0, 0.0), 25: (1.0, 4.0, 0.0), 26: (2.0, 4.0, 0.0), 27: (3.0, 4.0, 0.0), 28: (4.0, 4.0, 0.0), 29: (5.0, 4.0, 0.0),
            18: (0.0, 3.0, 0.0), 19: (1.0, 3.0, 0.0), 20: (2.0, 3.0, 0.0), 21: (3.0, 3.0, 0.0), 22: (4.0, 3.0, 0.0), 23: (5.0, 3.0, 0.0),
            12: (0.0, 2.0, 0.0), 13: (1.0, 2.0, 0.0), 14: (2.0, 2.0, 0.0), 15: (3.0, 2.0, 0.0), 16: (4.0, 2.0, 0.0), 17: (5.0, 2.0, 0.0),
            6:  (0.0, 1.0, 0.0),  7: (1.0, 1.0, 0.0),  8: (2.0, 1.0, 0.0),  9: (3.0, 1.0, 0.0), 10: (4.0, 1.0, 0.0), 11: (5.0, 1.0, 0.0),
            0:  (0.0, 0.0, 0.0),  1: (1.0, 0.0, 0.0),  2: (2.0, 0.0, 0.0),  3: (3.0, 0.0, 0.0),  4: (4.0, 0.0, 0.0),  5: (5.0, 0.0, 0.0),

            93:(2.5, 2, 0), 
            95:(3, 2.5,0),
            96:(2.5,3,0),
            97:(2, 2.5,0),

            # 90:(0.0, -14.0, 3.83),
            # 91:(2.70, -14.0, 3.83),
            # 100:(5.37, -14.0, 3.83),

            #94: (0.0, 0.0, 0.0),
            100: (0.085, 0.27, 0), 101: (0.085, 1.27, 0), 102: (0.085, 2.27, 0), 103: (0.085, 3.27, 0),
            104: (0.085, 4.27, 0), 105: (0.085, 5.27, 0), 106: (0.085, 6.27, 0), 107: (0.085, 7.27, 0),
            108: (0.085, 8.27, 0), 109: (0.085, 9.27, 0), 110: (1.085, 0.27, 0), 111: (1.085, 1.27, 0),
            112: (1.085, 2.27, 0), 113: (1.085, 3.27, 0), 114: (1.085, 4.27, 0), 115: (1.085, 5.27, 0),
            116: (1.085, 6.27, 0), 117: (1.085, 7.27, 0), 118: (1.085, 8.27, 0), 119: (1.085, 9.27, 0),
            120: (2.085, 0.27, 0), 121: (2.085, 1.27, 0), 122: (2.085, 2.27, 0), 123: (2.085, 3.27, 0),
            124: (2.085, 4.27, 0), 125: (2.085, 5.27, 0), 126: (2.085, 6.27, 0), 127: (2.085, 7.27, 0),
            128: (2.085, 8.27, 0), 129: (2.085, 9.27, 0), 130: (3.085, 0.27, 0), 131: (3.085, 1.27, 0),
            132: (3.085, 2.27, 0), 133: (3.085, 3.27, 0), 134: (3.085, 4.27, 0), 135: (3.085, 5.27, 0),
            136: (3.085, 6.27, 0), 137: (3.085, 7.27, 0), 138: (3.085, 8.27, 0), 139: (3.085, 9.27, 0),
            140: (4.085, 0.27, 0), 141: (4.085, 1.27, 0), 142: (4.085, 2.27, 0), 143: (4.085, 3.27, 0),
            144: (4.085, 4.27, 0), 145: (4.085, 5.27, 0), 146: (4.085, 6.27, 0), 147: (4.085, 7.27, 0),
            148: (4.085, 8.27, 0), 149: (4.085, 9.27, 0), 150: (5.085, 0.27, 0), 151: (5.085, 1.27, 0),
            152: (5.085, 2.27, 0), 153: (5.085, 3.27, 0), 154: (5.085, 4.27, 0), 155: (5.085, 5.27, 0),
            156: (5.085, 6.27, 0), 157: (5.085, 7.27, 0), 158: (5.085, 8.27, 0), 159: (5.085, 9.27, 0),
            160: (6.085, 0.27, 0), 161: (6.085, 1.27, 0), 162: (6.085, 2.27, 0), 163: (6.085, 3.27, 0),
            164: (6.085, 4.27, 0), 165: (6.085, 5.27, 0), 166: (6.085, 6.27, 0), 167: (6.085, 7.27, 0),
            168: (6.085, 8.27, 0), 169: (6.085, 9.27, 0), 170: (7.085, 0.27, 0), 171: (7.085, 1.27, 0),
            172: (7.085, 2.27, 0), 173: (7.085, 3.27, 0), 174: (7.085, 4.27, 0), 175: (7.085, 5.27, 0),
            176: (7.085, 6.27, 0), 177: (7.085, 7.27, 0), 178: (7.085, 8.27, 0), 179: (7.085, 9.27, 0),
            180: (8.085, 0.27, 0), 181: (8.085, 1.27, 0), 182: (8.085, 2.27, 0), 183: (8.085, 3.27, 0),
            184: (8.085, 4.27, 0), 185: (8.085, 5.27, 0), 186: (8.085, 6.27, 0), 187: (8.085, 7.27, 0),
            188: (8.085, 8.27, 0), 189: (8.085, 9.27, 0), 190: (9.085, 0.27, 0), 191: (9.085, 1.27, 0),
            192: (9.085, 2.27, 0), 193: (9.085, 3.27, 0), 194: (9.085, 4.27, 0), 195: (9.085, 5.27, 0),
            196: (9.085, 6.27, 0), 197: (9.085, 7.27, 0), 198: (9.085, 8.27, 0), 199: (9.085, 9.27, 0)
        }

        default_quaternion = (0.0, 0.0, 0.0, 1.0)

        self.known_tags = {
            f"tag36h11:{tag_id}": {
                "translation": pos,
                "quaternion": default_quaternion
            }
            for tag_id, pos in listePoints3D.items()
        }

        self.max_transform_age_sec = 0.2  # On ignore les transforms plus vieilles que 200ms
        self.timer = self.create_timer(0.1, self.publish_camera_pose)  # 10 Hz

    def publish_camera_pose(self):
        camera_transforms = []

        for tag_id, tag_info in self.known_tags.items():
            try:
                cam_to_tag = self.tf_buffer.lookup_transform(
                    'camera', tag_id, rclpy.time.Time()
                )

                # Vérifier l'âge de la transform
                stamp = cam_to_tag.header.stamp
                now = self.get_clock().now().to_msg()
                age_sec = (now.sec + now.nanosec * 1e-9) - (stamp.sec + stamp.nanosec * 1e-9)

                if age_sec > self.max_transform_age_sec:
                    #self.get_logger().info(f"Ignoré {tag_id} : transform trop ancienne ({age_sec:.3f}s)")
                    continue

                # Convertir en matrices homogènes
                t_tag_cam = np.array([
                    cam_to_tag.transform.translation.x,
                    cam_to_tag.transform.translation.y,
                    cam_to_tag.transform.translation.z,
                ])
                q_tag_cam = np.array([
                    cam_to_tag.transform.rotation.x,
                    cam_to_tag.transform.rotation.y,
                    cam_to_tag.transform.rotation.z,
                    cam_to_tag.transform.rotation.w,
                ])
                T_tag_cam = self.transform_to_matrix(t_tag_cam, q_tag_cam)
                T_cam_tag = np.linalg.inv(T_tag_cam)

                T_tag_world = self.transform_to_matrix(tag_info['translation'], tag_info['quaternion'])
                T_cam_world = T_tag_world @ T_cam_tag
                camera_transforms.append(T_cam_world)

            except LookupException:
                pass
            except Exception as e:
                self.get_logger().warn(f'Erreur sur {tag_id} : {e}')

        if not camera_transforms:
            self.get_logger().info("Aucune transform valide disponible pour le calcul de la pose.")
            return

        # Moyenne des matrices de transformation
        T_cam_world_avg = sum(camera_transforms) / len(camera_transforms)
        t_avg, q_avg = self.matrix_to_transform(T_cam_world_avg)

        # Transform to be in the dirigeable's orientation
        q_cam = R.from_quat(q_avg)
        q_flip_x = R.from_euler('x', 180, degrees=True)
        q_dirigeable = (q_cam * q_flip_x).as_quat()

        # Print position and orientation (roll, pitch, yaw)
        r = R.from_quat(q_dirigeable)
        roll, pitch, yaw = r.as_euler('XYZ', degrees=True)  # intrinsèque XYZ = roulis, tangage, lacet
        self.get_logger().info(f"Camera position: x={t_avg[0]:.3f}, y={t_avg[1]:.3f}, z={t_avg[2]:.3f}")
        self.get_logger().info(f"Roll (roulis): {roll:.2f}°, Pitch (tangage): {pitch:.2f}°, Yaw (lacet): {yaw:.2f}°")

        # Publier la première transform : world -> camera
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = 'camera'
        tf_msg.transform.translation.x = t_avg[0]
        tf_msg.transform.translation.y = t_avg[1]
        tf_msg.transform.translation.z = t_avg[2]
        tf_msg.transform.rotation.x = q_avg[0]
        tf_msg.transform.rotation.y = q_avg[1]
        tf_msg.transform.rotation.z = q_avg[2]
        tf_msg.transform.rotation.w = q_avg[3]

        # Publier la deuxième transform : world -> dirigeable (rotation 180° autour de X caméra)
        tf_dirigeable = TransformStamped()
        tf_dirigeable.header.stamp = self.get_clock().now().to_msg()
        tf_dirigeable.header.frame_id = 'world'
        tf_dirigeable.child_frame_id = 'dirigeable'
        tf_dirigeable.transform.translation.x = t_avg[0]
        tf_dirigeable.transform.translation.y = t_avg[1]
        tf_dirigeable.transform.translation.z = t_avg[2]
        tf_dirigeable.transform.rotation.x = q_dirigeable[0]
        tf_dirigeable.transform.rotation.y = q_dirigeable[1]
        tf_dirigeable.transform.rotation.z = q_dirigeable[2]
        tf_dirigeable.transform.rotation.w = q_dirigeable[3]

        # Position
        pos_msg = Vector3Stamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.vector.x = t_avg[0]
        pos_msg.vector.y = t_avg[1]
        pos_msg.vector.z = t_avg[2]

        # Orientation
        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = self.get_clock().now().to_msg()
        rpy_msg.vector.x = roll
        rpy_msg.vector.y = pitch
        rpy_msg.vector.z = yaw

        self.tf_broadcaster.sendTransform(tf_msg)
        self.tf_broadcaster.sendTransform(tf_dirigeable)
        self.position_pub.publish(pos_msg)
        self.orientation_pub.publish(rpy_msg)


    def transform_to_matrix(self, translation, quaternion):
        T = tf_transformations.quaternion_matrix(quaternion)
        T[0:3, 3] = translation
        return T

    def matrix_to_transform(self, T):
        translation = T[0:3, 3]
        quaternion = tf_transformations.quaternion_from_matrix(T)
        return translation, quaternion

def main(args=None):
    rclpy.init(args=args)
    node = CameraTagPoseBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
