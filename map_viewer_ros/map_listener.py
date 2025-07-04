import sys
from robot_pose_manager import RobotClient

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


import numpy as np
import pygame


class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        self.robot_client = RobotClient()

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)

        # pygame 초기화
        pygame.init()
        self.screen = pygame.display.set_mode((1280, 720))
        pygame.display.set_caption("Map Navigation")
        self.running = True
        self.map_surface = None

    def map_callback(self, msg):
        self.map_info = msg.info


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

        self.map_surface = pygame.surfarray.make_surface(img)


    def map_to_pixel(x, y, info):
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution
        px = int((x - origin_x) / res)
        py = int((y - origin_y) / res)
        return px, py


        

    def draw_robot(screen, robot_pose, map_info):
        if robot_pose is None:
            return

        x, y, yaw = robot_pose

        # map information
        # resolution, origin_x, origin_y
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y

        # transform between map to image
        img_x = int((x - origin_x) / resolution)
        img_y = int((y - origin_y) / resolution)

        # coordinate transform (pygame window -> map_frame)
        img_y = map_info.height - img_y

        #pygame.draw.circle(screen, (255, 0, 0), (img_x, img_y), 4)


            
        end_x = img_x + int(10 * math.cos(yaw))
        end_y = img_y - int(10 * math.sin(yaw))
        pygame.draw.line(screen, (0, 255, 0), (img_x, img_y), (end_x, end_y), 2)

    def run(self):
        rate = self.create_rate(10)
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.01)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            if self.map_surface is not None:
                self.screen.blit(pygame.transform.scale(self.map_surface, (1280, 720)), (0, 0))

                pose = self.robot_client.get_robot_pose()


                if pose:
                    x, y, yaw = pose
                    print(f"Robot Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

                    # 화면에 표시
                    screen.fill((255, 255, 255))
                    pygame.draw.circle(screen, (255, 0, 0), (int(x * 100 + 400), int(y * 100 + 300)), 5)
                    pygame.display.flip()  
                
                # 여기에 좌표를 변환해서 point 표시할 수 있음
                
               #self.draw_robot(self.screen, ''' robot pose ''', self.map_callback)

            pygame.display.flip()

        pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    viewer = MapViewer()
    viewer.run()
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

