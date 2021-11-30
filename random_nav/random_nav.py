import time
import numpy as np

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from nav_msgs.msg import OccupancyGrid
from jmu_ros2_util import map_utils, transformations

"""
repeat forever:

   # randomly select a free location in the map
   while goal location not selected:
       select a random location in the map
       check the map to see if that location is free
   
   Ask the navigation system to navigate to the selected goal location
   
   Wait until the the goal is reached or aborted
"""

class RandomNavNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('nav_demo')

        self.create_timer(.1, self.timer_callback)

        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

        # Create the action client and wait until it is active
        self.ac = ActionClient(self, NavigateToPose, '/NavigateToPose')
        self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        self.get_logger().info("NAVIGATION SERVER AVAILABLE...")

        self.goal = None
        self.goal.pose.header.frame_id = 'map'  # SEEMS TO BE IGNORED!
        #find random values to assign instead
        
        self.map = None

    def timer_callback(self):
        while True:
            while self.goal is None:
                x = randrange(0.0, self.map.width)
                y = randrange(0.0, self.map.height)
                val = self.map.get_cell(x, y)
                if val == 0:
                    self.goal = NavigateToPose.Goal()
                    x = self.goal.pose.pose.position.x
                    y = self.goal.pose.pose.position.y
            self.goal_future = self.ac.send_goal_async(self.goal)
            while not self.goal_future.done():
                pass 
            self.goal = None
            

                
    
    def map_callback(self, map_msg):
        """Process the map message.  This doesn't really do anything useful, it is
        purely intended as an illustration of the Map class.

        """
        if self.map is None:  # No need to do this every time map is published.

            self.map = map_utils.Map(map_msg)


def main():
    print('Hi from random_nav.')
    rclpy.init()
    #Do we need to make a node? How does rclpy know to stop? 
    node = RandomNavNode()
    rclpy.spin_until_future_complete(node, node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
