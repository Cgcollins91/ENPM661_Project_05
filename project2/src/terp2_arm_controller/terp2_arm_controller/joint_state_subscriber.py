import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        
        # Subscriber to the /model_states topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.joint_angles = {}
        self.joint_velocities = {}

    def joint_states_callback(self, msg):
        joint_names = ['joint_arm_1', 'joint_arm_2', 'joint_arm_3', 'joint_arm_4', 'joint_arm_5']

        for i, model_name in enumerate(msg.name):
            if model_name not in joint_names:
                continue
            else:
                position = msg.position[i]
                velocity = msg.velocity[i]
                effort = msg.effort[i]
                
                self.get_logger().info(
                    f'  Joint: {model_name}\n'
                    f'  Position: {position}\n'
                    f'  Velocity: {velocity}\n'
                    f'  Effort: {effort}\n'
                )
                self.joint_angles[model_name]     = position
                self.joint_velocities[model_name] = velocity
            

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    
    # Shutdown
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





