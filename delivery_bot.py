#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

# Define positions 
POSITIONS = {
    "home": (0, 0, 0),
    "kitchen": (0.72, 0.49, 0),
    "table1": (2.09, -0.61, 0),
    "table2": (0.87, -0.61, 0),
    "table3": (-0.64, -0.61, 0)
}


TIMEOUT = 10  # Time in seconds to wait for confirmation

class RestaurantDeliveryRobot:
    def __init__(self):
        rospy.init_node('restaurant_delivery_robot', anonymous=True)
        
        # Action client for move_base
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        # Subscribers
        rospy.Subscriber("/order", String, self.order_callback)
        rospy.Subscriber("/confirmation", String, self.confirmation_callback)
        rospy.Subscriber("/cancel", String, self.cancel_callback)

        # Robot states
        self.current_orders = []
        self.task_cancelled = False
        self.confirmation_received = False

    def send_goal(self, location):
        """ Sends the robot to the given location """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location[0]
        goal.target_pose.pose.position.y = location[1]
        goal.target_pose.pose.orientation.w = 1.0  # Fixed orientation

        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo(f"Reached {location}")

    def confirmation_callback(self, msg):
        """ Updates confirmation status when received """
        if msg.data.strip().lower() == "ok":
            rospy.loginfo("Confirmation received!")
            self.confirmation_received = True

    def wait_for_confirmation(self):
        """ Waits for confirmation within a timeout period """
        rospy.loginfo("Waiting for confirmation...")
        self.confirmation_received = False
        timeout_time = rospy.Time.now() + rospy.Duration(TIMEOUT)

        while not self.confirmation_received and rospy.Time.now() < timeout_time:
            if self.task_cancelled:  # Stop waiting if cancelled
                return False
            rospy.sleep(1)

        return self.confirmation_received

    def cancel_callback(self, msg):
        """ Handles task cancellation """
        rospy.loginfo(f"Task cancelled: {msg.data}")
        self.task_cancelled = True

    def order_callback(self, msg):
        """ Handles incoming orders """
        if self.task_cancelled:
            rospy.loginfo("New order received. Resetting cancellation state.")
            self.task_cancelled = False  # **RESET TASK CANCELLATION** 

        order_data = msg.data.split(',')  # Format: "table1,table2"
        self.current_orders = [table.strip() for table in order_data if table.strip() in POSITIONS]
        self.process_orders()

    def process_orders(self):
        if not self.current_orders:
            rospy.loginfo("No valid orders received.")
            return
        
        rospy.loginfo(f"Processing orders: {self.current_orders}")

        # Move to kitchen
        self.send_goal(POSITIONS["kitchen"])
        if self.task_cancelled:
            rospy.loginfo("Task cancelled while going to kitchen. Returning home.")
            self.send_goal(POSITIONS["home"])
            return

        if not self.wait_for_confirmation():
            rospy.loginfo("No confirmation at kitchen. Returning home.")
            self.send_goal(POSITIONS["home"])
            return
        
        # Move to tables
        tables_to_visit = self.current_orders[:]
        final_tables_delivered = []
        for table in tables_to_visit:
            if self.task_cancelled:
                rospy.loginfo("Task cancelled while going to table. Returning to kitchen and home.")
                self.send_goal(POSITIONS["kitchen"])
                self.send_goal(POSITIONS["home"])
                return

            self.send_goal(POSITIONS[table])
            if self.task_cancelled:
                rospy.loginfo("Task cancelled at table. Returning to kitchen and home.")
                self.send_goal(POSITIONS["kitchen"])
                self.send_goal(POSITIONS["home"])
                return

            if not self.wait_for_confirmation():
                rospy.loginfo(f"No confirmation at {table}, skipping to next.")
                continue

            final_tables_delivered.append(table)

        if final_tables_delivered:
            rospy.loginfo("All confirmed deliveries completed. Returning to kitchen before going home.")
            self.send_goal(POSITIONS["kitchen"])
        
        self.send_goal(POSITIONS["home"])
        rospy.loginfo("All tasks completed.")

if __name__ == '__main__':
    try:
        robot = RestaurantDeliveryRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Navigation interrupted.")
