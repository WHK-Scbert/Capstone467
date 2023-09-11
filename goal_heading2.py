#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import httpx
import subprocess

class GoalHeadingNode(Node):

    def __init__(self):
        super().__init__('goal_heading')
        self.data_publisher = self.create_publisher(Float64, 'goal_heading', 10)
        self.url = 'http://192.168.20.145:8081/DOA_value.html'
        self.timer = self.create_timer(1, self.fetch_and_publish_data)

    def fetch_and_publish_data(self):
        try:
            # Fetch data from the URL
            response = httpx.get(self.url)
            raw_data = response.text
            data_value = raw_data.strip().split(',')


            command = f'curl -s {self.url}'
            data2 = subprocess.check_output(command, shell=True).decode()
            data_value2 = data2.split(',')
            #print(data_value2[1])

            # Check if the request was successful
            if response.status_code == 200:
                # Split the data by comma and get the second value
                heading = float(data_value2[1])
                print("Heading is " + str(heading))
                msg = Float64()
                msg.data = heading
                self.data_publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error("An error occurred: %s", str(e))

def main(args=None):
    rclpy.init(args=args)
    node = GoalHeadingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
