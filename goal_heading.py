#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import httpx

class GoalHeadingNode(Node):

    def __init__(self):
        super().__init__('goal_heading')
        self.data_publisher = self.create_publisher(Float64, 'goal_heading_topic', 10)
        self.url = 'https://192.168.50.5:8081/DOA_value.html/'
        self.timer = self.create_timer(1, self.fetch_and_publish_data)

    def fetch_and_publish_data(self):
        try:
            # Fetch data from the URL
            response = httpx.get(self.url)

            # Check if the request was successful
            if response.status_code == 200:
                # Split the data by comma and get the second value
                '''
                data_values = response.text.strip().split(',')
                if len(data_values) >= 2:
                    second_value = float(data_values[1])  # Convert to float

                    # Publish the second value as a Float64 to the ROS 2 topic
                    msg = Float64()
                    msg.data = second_value
                    self.data_publisher.publish(msg)
                    self.get_logger().info("Published goal heading: %.2f", second_value)
                else:
                    self.get_logger().warn("Data format is not as expected")
            else:
                self.get_logger().error("Failed to fetch data from the URL. Status code: %d", response.status_code)

        except Exception as e:
            self.get_logger().error("An error occurred: %s", str(e))
            '''
                print(response.text)
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
