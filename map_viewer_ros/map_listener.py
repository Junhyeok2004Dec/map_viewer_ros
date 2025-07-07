import sys
from threading import Thread

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

import numpy as np
import pygame

class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')

        pygame.init()
        
        self.screen = pygame.display.set_mode((1280, 720))


        pygame.display.set_caption("Map Navigation")
        

        self.running = True
        self.map_info = msg.info
        self.map_surface = None

    def map_callback(self, msg):
        self.map_surface = pygame.surfarray.make_surface(img)
        
        
    def map_setting(self, msg):
        self.map_surface = msg
        
        


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
    ros_thread.destroy_node()
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

