import rclpy
import rclpy.node

from behavior_interface.msg import Command
from sensor_msgs.msg import Joy


BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_BACK = 6
BUTTON_SELECT = 7
BUTTON_LOGITECH = 8
BUTTON_CLICK_LEFT_PAD = 9
BUTTON_CLICK_RIGHT_PAD = 10

AXIS_LEFT_HORIZONTAL = 0  # Left  joystick
AXIS_LEFT_VERTICAL = 1  # Left  joystick
AXIS_LT = 2  # Left  progressive button
AXIS_RIGHT_HORIZONTAL = 3  # Right joystick
AXIS_RIGHT_VERTICAL = 4  # Right joystick
AXIS_RT = 5  # Right progressive button
AXIS_CROSS_HORIZONTAL = 6  # Cross
AXIS_CROSS_VERTICAL = 7  # Cross


class Node(rclpy.node.Node):
    def __init__(self, node_name='command'):
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Publisher pour le topic command
        self.command_publisher = self.create_publisher(Command, 'command', 10)

        self.get_logger().info("Joystick Teleop Node initialized")

        # Initialise l'état du bouton RB (deadman + trigger du décollage) 
        self.deadman_pressed = False
        self.hover_pressed = False
        self.emergency_pressed = False
        self.forward = False
        self.backward = False
        self.left = False
        self.right = False
        self.up = False
        self.down = False
        self.turnleft = False
        self.turnright = False

    def joy_callback(self, msg):
        command_msg = Command()
        

        # ATTENTION A DETECTER QUE le bouton n'était pas enfoncé et on l'enfonce
        


        if msg.buttons[BUTTON_RB] == 1 and not self.deadman_pressed:
                command_msg.command = "TakeOff"
                self.command_publisher.publish(command_msg)
                self.get_logger().info("RB pressed: Takeoff initiated")
                self.deadman_pressed = True

        elif msg.buttons[BUTTON_RB] == 0 and self.deadman_pressed:
                command_msg.command = "Land"
                self.command_publisher.publish(command_msg)
                self.get_logger().info("RB released: Landing initiated")
                self.deadman_pressed = False
                self.hover_pressed = False       # va falloir la mettre à d'autres endroits
                

        if self.deadman_pressed:

                if msg.buttons[BUTTON_LB] == 1 and not self.emergency_pressed:
                    command_msg.command = "EmergencyStop"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("LB pressed: Emergency Stop !")
                    self.emergency_pressed = True
                
                if msg.axes[AXIS_RT] <= -0.5 and not self.hover_pressed:
                    command_msg.command = "Hover"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("RT pressed: Hover initiated")
                    self.hover_pressed = True


                if msg.axes[AXIS_LEFT_VERTICAL] >= 0.5 and not self.forward:
                    command_msg.command = "Forward"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log forward")
                    self.forward = True
                
                if msg.axes[AXIS_LEFT_VERTICAL] <= -0.5 and not self.backward:
                    command_msg.command = "Backward"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log backward")
                    self.backward = True

                if msg.axes[AXIS_LEFT_HORIZONTAL] <= -0.5 and not self.right:
                    command_msg.command = "Right"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log right")
                    self.right = True
                
                if msg.axes[AXIS_LEFT_HORIZONTAL] >= 0.5 and not self.left:
                    command_msg.command = "Left"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log left")
                    self.left = True

                if msg.axes[AXIS_RIGHT_VERTICAL] >= 0.5 and not self.up:
                    command_msg.command = "Up"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log up")
                    self.up = True

                if msg.axes[AXIS_RIGHT_VERTICAL] <= -0.5 and not self.down:
                    command_msg.command = "Down"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log down")
                    self.down = True

                if msg.axes[AXIS_RIGHT_HORIZONTAL] <= -0.5 and not self.turnright:
                    command_msg.command = "TurnRight"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log turnright")
                    self.turnright = True

                if msg.axes[AXIS_RIGHT_HORIZONTAL] >= 0.5 and not self.turnleft:
                    command_msg.command = "TurnLeft"
                    self.command_publisher.publish(command_msg)
                    self.get_logger().info("log turnleft")
                    self.turnleft = True



                


        if msg.buttons[BUTTON_LB] == 0 and self.emergency_pressed:
                self.emergency_pressed = False
        if msg.axes[AXIS_RT] >= 0 and self.hover_pressed:
                self.hover_pressed = False 


        if msg.axes[AXIS_LEFT_VERTICAL] <= 0.5 and self.forward:
            self.forward = False
                
        if msg.axes[AXIS_LEFT_VERTICAL] >= -0.5 and self.backward:
            self.backward = False

        if msg.axes[AXIS_LEFT_HORIZONTAL] >= -0.5 and self.right:
            self.right = False
        
        if msg.axes[AXIS_LEFT_HORIZONTAL] <= 0.5 and self.left:
            self.left = False

        if msg.axes[AXIS_RIGHT_VERTICAL] <= 0.5 and self.up:
            self.up = False

        if msg.axes[AXIS_RIGHT_VERTICAL] >= -0.5 and self.down:
            self.down = False

        if msg.axes[AXIS_RIGHT_HORIZONTAL] >= -0.5 and self.turnright:
            self.turnright = False

        if msg.axes[AXIS_RIGHT_HORIZONTAL] <= 0.5 and self.turnleft:
            self.turnleft = False
        

            # elif msg.axes[AXIS_LEFT_VERTICAL] > 0.5:
            #     command_msg.command = "MoveForward"
            # elif msg.axes[AXIS_LEFT_VERTICAL] < -0.5:
            #     command_msg.command = "MoveBackward"
            # elif msg.axes[AXIS_LEFT_HORIZONTAL] > 0.5:
            #     command_msg.command = "MoveRight"
            # elif msg.axes[AXIS_LEFT_HORIZONTAL] < -0.5:
            #     command_msg.command = "MoveLeft"
            
            # elif msg.buttons[BUTTON_Y] == 1:
            #     command_msg.command = "EmergencyStop"

        # Ne publie que si une commande est envoyée
        # if command_msg.command:
        #     self.command_publisher.publish(command_msg)
        #     self.get_logger().info(f"Published command: {command_msg.command}")

        # elif msg.buttons[BUTTON_LOGITECH] == 1:
        #     command_msg.command = "Wtf"
        #     self.command_publisher.publish(command_msg)
        #     self.get_logger().info("Zoumba triggered: Hula Hoop initiated")
    
        



def main(args=None):
    rclpy.init(args=args)
    joy_teleop = Node('joy_teleop')
    rclpy.spin(joy_teleop)
    joy_teleop.destroy_node()
    rclpy.shutdown()
