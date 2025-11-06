import rclpy
import random
from rclpy.node import Node
from interfaces_package.msg import JerryCommand


class CommandPublisher(Node):
    # A ROS2 node that publishes a JerryCommand

    def __init__(self):
        super().__init__('command_publisher_node')

        # Create publisher for JerryCommand messages
        self.jerry_command_publisher = self.create_publisher(
            JerryCommand,         # message type
            'jerry_command',      # topic name
            10                    # QoS queue size
        )

        timer_period: float = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_command)

        self.incremental_id: int = 0

    def publish_command(self):
        # Method that is periodically called by the timer

        jerry_command = JerryCommand()

        # Fill the message with random commands and values
        directions = ['up', 'down', 'left', 'right']
        jerry_command.cmd = random.choice(directions)
        jerry_command.steps = random.randint(1, 5)
        jerry_command.id = self.incremental_id  # assuming message has an id field

        # Publish the message
        self.jerry_command_publisher.publish(jerry_command)

        self.get_logger().info(
            f"Publishing command #{self.incremental_id}: Move {jerry_command.cmd} for {jerry_command.steps} steps."
        )

        self.incremental_id += 1


def main(args=None):
    """
    The main function.
    :param args: Not used directly by the user, but instead by ROS2 to configure
    certain aspects of the Node.
    """

    try:
        rclpy.init(args=args)

        jerry_command_publisher_node = CommandPublisher()

        rclpy.spin(jerry_command_publisher_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
