import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates


class ModelStateSubscriber(Node):
    '''     Python class ModelStateSubscriber
            Subscribes to the /model_states topic and prints the model names, poses, and velocities
            
    '''
    def __init__(self):
        super().__init__('model_states_subscriber')
        
        # Subscriber to the /model_states topic
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_state_callback,
            10)
        self.subscription  # prevent unused variable warning

    def model_state_callback(self, msg):
        # Loop through all models and print their names and poses
        for i, model_name in enumerate(msg.name):
            position = msg.pose[i].position
            orientation = msg.pose[i].orientation
            linear_velocity = msg.twist[i].linear
            angular_velocity = msg.twist[i].angular
            
            self.get_logger().info(
                f'  Model: {model_name}\n'
                f'  Position:         x={position.x},         y={position.y},         z={position.z}\n'
                f'  Orientation:      x={orientation.x},      y={orientation.y},      z={orientation.z}, w={orientation.w}'
                f'  Linear Velocity:  x={linear_velocity.x},  y={linear_velocity.y},  z={linear_velocity.z}\n'
                f'  Angular Velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}\n'
            )

def main(args=None):
    rclpy.init(args=args)
    model_state_subscriber = ModelStateSubscriber()
    rclpy.spin(model_state_subscriber)
    
    # Shutdown
    model_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
