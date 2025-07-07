import math
import numpy as np
import rclpy
import tf2_ros

from rclpy.node import Node
from geometry_msgs.msg import TransformStamped

from map_listener import MapViewer

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from geometry_msgs.msg import TransformStamped


def euler_from_quaternion(qx, qy, qz, qw):
    """
    Quaternion (x,y,z,w) to Euler angles (roll, pitch, yaw)
    """

    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(ai, aj, ak):
    """
    Euler angels to Quaternion(x,y,z,w)
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q




class RobotClient(Node):
    def __init__(self, target_frame="map", source_frame="ego_racecar/base_link"):

        super().__init__('robot_pose_client')

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.target_frame = target_frame
        self.source_frame = source_frame

        self.timer = self.create_timer(0.005, self.get_robot_pose)
        self.mapViewer = None

        

    def listener_callback(self, msg):
       
        # width, height : size of map
        # data : the probabilities of each occupancy grids
        #

        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        
        # 0~100: free, 100: occupied, -1: unknown
        # for more information, you can site that https://mathworks.com/help/robotics/ug/occupancy-grids.html
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[data == 0] = [255, 255, 255]      # white = free
        img[data == 100] = [0, 0, 0]          # black = wall
        img[data == -1] = [128, 128, 128]     # gray = unknown

        
        img = np.flipud(img)
        img = np.rot90(img)
        pygame.surfarray.make_surface(img)

        mapViewer 

    def get_robot_pose(self):

            try:
                
                trans = self.tf_buffer.lookup_transform(
                "map",                                      # Target frame
                "ego_racecar/base_link",                    # Source frame
                rclpy.time.Time())       # 최신 Transform 사용


                x = trans.transform.translation.x
                y = trans.transform.translation.y

                qx = trans.transform.rotation.x
                qy = trans.transform.rotation.y
                qz = trans.transform.rotation.z
                qw = trans.transform.rotation.w

                _, _, yaw = euler_from_quaternion(qx, qy, qz, qw)

                return x, y, yaw

            except Exception as e:
                self.get_logger().warn(f"TF Error: {e}")
                return None

       
def main(args=None):
    rclpy.init(args=args)
    node = RobotClient()
    mapViewer = MapViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
