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

link_arm_1_xyz  = [ 0.0102112642270262, 0.0397887357729738, 0]
link_arm_2_xyz  = [-0.425, 0.0479521392077196, -5.04698321809284E-18]
link_arm_3_xyz  = [-0.37499896693226, -0.0499499434508651, 1.40926562802143E-08]
link_arm_4_xyz  = [ 0.0557979256916667, 0.0449730992840096, 3.2724099398948E-17]
link_arm_5_xyz  = [ 0.0102112642270262, -0.0397887357729738, -1.38777878078145E-17]

joint_arm_1_xyz = [ 0, 0, 0.01]
joint_arm_2_xyz = [ 0.04995, 0.0500000000000008,  0]
joint_arm_3_xyz = [-0.85,  0, 0]
joint_arm_4_xyz = [ -0.75, -0.09995, 0]
joint_arm_5_xyz = [0.15, 0.05, 0]



