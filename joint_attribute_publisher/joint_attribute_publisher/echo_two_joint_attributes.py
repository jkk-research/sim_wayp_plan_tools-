import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointAttributeEcho(Node):
    def __init__(self):
        super().__init__('joint_attribute_echo')

        # Specify the two joint names and their corresponding attributes
        self.joint_attributes = {
            'rear_left_wheel_joint': 'velocity',  # Replace with your first joint's name and desired attribute
            'front_left_wheel_steering_joint': 'position'   # Replace with your second joint's name and desired attribute
        }

        # Subscriber to the joint states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a dictionary of publishers for each joint
        self.joint_publishers = {}
        for joint_name in self.joint_attributes.keys():
            attribute = self.joint_attributes[joint_name]
            topic_name = f'/gamma/{attribute}'
            self.joint_publishers[joint_name] = self.create_publisher(Float64, topic_name, 10)

    def joint_state_callback(self, msg):
        for joint_name, attribute in self.joint_attributes.items():
            try:
                # Find the index of the desired joint
                index = msg.name.index(joint_name)

                # Get the desired attribute
                if attribute == 'position':
                    value = msg.position[index]
                elif attribute == 'velocity':
                    value = msg.velocity[index]
                elif attribute == 'effort':
                    value = msg.effort[index]
                else:
                    self.get_logger().error(f"Unknown attribute: {attribute}")
                    return

                # Create and publish the Float64 message
                velocity_msg = Float64()
                velocity_msg.data = value
                self.joint_publishers[joint_name].publish(velocity_msg)

                # self.get_logger().info(f"{joint_name} {attribute}: {value}")

            except ValueError:
                self.get_logger().warn(f"Joint {joint_name} not found in JointState message.")
            except IndexError:
                self.get_logger().warn(f"Attribute {attribute} not available for joint {joint_name}.")

def main(args=None):
    rclpy.init(args=args)
    joint_attribute_echo = JointAttributeEcho()
    rclpy.spin(joint_attribute_echo)

    # Destroy the node explicitly
    joint_attribute_echo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
