import argparse
import time
import numpy as np

import rclpy
import rclpy.node
from rclpy.action.client import ActionClient
from rclpy.task import Future

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from jmu_ros2_util import map_utils, transformations
import random

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

        latching_qos = QoSProfile(depth=1,
                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.map = None
        self.nav_done = False
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 
                qos_profile=latching_qos)

        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.goal = NavigateToPose.Goal()
        self.get_logger().info("WAITING FOR NAVIGATION SERVER...")
        self.ac.wait_for_server()
        self.get_logger().info("NAVIGATION SERVER AVAILABLE...")
        


    def send_goal(self):
        self.goal.pose.header.frame_id = 'map'  # SEEMS TO BE IGNORED!
        if self.map is None: 
            x = np.random.randint(0.0, 6)
            y = np.random.randint(0.0, 6)
            self.get_logger().info("map is none")
        else:
            x = np.random.randint(0.0, self.map.width)
            y = np.random.randint(0.0, self.map.height)

        self.goal.pose.pose.position.x = float(x) #random num in width
        self.goal.pose.pose.position.y = float(y) #random num in height

        # We need to convert theta to a quaternion....
        quaternion = transformations.quaternion_from_euler(0, 0, 0, 'rxyz')
        self.goal.pose.pose.orientation.x = quaternion[0]
        self.goal.pose.pose.orientation.y = quaternion[1]
        self.goal.pose.pose.orientation.z = quaternion[2]
        self.goal.pose.pose.orientation.w = quaternion[3]

        self.timeout = 10

        
        self.get_logger().info("SENDING GOAL TO NAVIGATION SERVER...")
        self.start_time = time.time()

        self.cancel_future = None
        self.goal_future = self.ac.send_goal_async(self.goal)

        self.create_timer(.1, self.timer_callback)
        self.future_event = Future()

        return self.future_event


    def timer_callback(self):
        """ Periodically check in on the progress of navigation. """
        if not self.goal_future.done():
            self.get_logger().info("NAVIGATION GOAL NOT YET ACCEPTED")

        elif self.cancel_future is not None:  # We've cancelled and are waiting for ack.
            if self.cancel_future.done():
                self.future_event.set_result(None)
                self.get_logger().info("EXITING!")

        else:

            if self.goal_future.result().status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("NAVIGATION SERVER REPORTS SUCCESS. EXITING!")
                self.nav_done = True
                self.future_event.set_result(None)

            if self.goal_future.result().status == GoalStatus.STATUS_ABORTED:
                self.get_logger().info("NAVIGATION SERVER HAS ABORTED. EXITING!")
                self.nav_done = True
                self.future_event.set_result(None)

            elif time.time() - self.start_time > self.timeout:
                self.get_logger().info("TAKING TOO LONG. CANCELLING GOAL!")
                self.cancel_future = self.goal_future.result().cancel_goal_async()
                self.future_event.set_result(None)
                self.nav_done = True
        if self.nav_done == True:
                self.send_goal()
    
    def map_callback(self, map_msg):
        """Process the map message.  This doesn't really do anything useful, it is
        purely intended as an illustration of the Map class.
        """
        if self.map is None:  # No need to do this every time map is published.

            self.map = map_utils.Map(map_msg)

            # Use numpy to calculate some statistics about the map:
            total_cells = self.map.width * self.map.height
            pct_occupied = np.count_nonzero(self.map.grid == 100) / total_cells * 100
            pct_unknown = np.count_nonzero(self.map.grid == -1) / total_cells * 100
            pct_free = np.count_nonzero(self.map.grid == 0) / total_cells * 100
            map_str = "Map Statistics: occupied: {:.1f}% free: {:.1f}% unknown: {:.1f}%"
            self.get_logger().info(map_str.format(pct_occupied, pct_free, pct_unknown))

            # Here is how to access map cells to see if they are free:
            # x = 2.06
            # y = -1.2
            x = self.goal.pose.pose.position.x
            y = self.goal.pose.pose.position.y
            val = self.map.get_cell(x, y)
            if val == 100:
                free = "occupied"
                self.get_logger().info("NAVIGATION SERVER HAS ABORTED. space was occupied!")
                self.ac.destroy()
                self.future_event.set_result(None)
            elif val == 0:
                free = "free"
            else:
                free = "unknown"
            self.get_logger().info("HEY! Map position ({:.2f}, {:.2f}) is {}".format(x, y, free))


def main():
    print('Hi from random_nav.')
    rclpy.init() 
    node = RandomNavNode()

    future = node.send_goal()

    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
