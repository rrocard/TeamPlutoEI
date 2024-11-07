import rclpy
import rclpy.node

from behavior_interface.msg import Command, BehaviorStatus
from sensor_msgs.msg import Joy




class Node(rclpy.node.Node):
    def __init__(self, node_name='command'):  #a voir si interference avec le 'command' de joy_teleop
        super().__init__(node_name)
        self.subscription = self.create_subscription(
            Command,
            'command',
            self.command_callback,
            10
        )

        # liste des behaviors possibles
        self.behaviors = ['Land', 'TakeOff', 'Hover', 'MoveForward', 'MoveBackward' 'MoveLeft', 'MoveUp', 'MoveRight', 'TurnLeft', 'TurnRight']
        self.commands = {
            'TakeOff' : [(0, 'TakeOff')],
            'Land' : [(0, 'Land')],
            'EmergencyStop' : [(0, 'Hover'), (1, 'Land')],
            'Hover' : [(0, 'Hover')],
            'Forward' : [(0, 'Hover'), (0.5, 'MoveForward')],
            'Backward' : [(0, 'Hover'), (0.5, 'MoveBackward')],
            'CrabLeft' : [(0, 'Hover'), (0.5, 'MoveLeft')],
            'CrabRight' : [(0, 'Hover'), (0.5, 'MoveRight')],
            'TurnLeft' : [(0, 'Hover'), (0.5, 'TurnLeft')],
            'TurnRight' : [(0, 'Hover'), (0.5, 'TurnRight')],
            'Up' : [(0, 'Hover'), (0.5, 'MoveUp')],
            'Down' : [(0, 'Hover'), (0.5, 'MoveDown')],
            
            'Wtf' : [(0, 'Hover'), (3, 'TurnLeft'), (3.2, 'MoveLeft')], # il devrait faire des ronds aberrants
            

        }
        
        self.behavior_publisher = self.create_publisher(BehaviorStatus, 'behavior', 10)

        # Timer pour check la queue d'evenements
        self.timer = self.create_timer(0.05, self._on_time)
        
        # Queue des evenements programmés
        self.event_queue = []
        
        # Initialisation du temps
        self.start_time = None

        self.get_logger().info("Command Node Initialized")
       
 
    def command_callback(self, msg):
        """
        Callback déclenché quand une commande est publiée. 
        Programme les activations de behavior suivant les timings de ladite commande
        """
        #désactivation prealable des potentiels behavior activés
        self._deactivate_all_behaviors()
        
        # Temps présent
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Degage les evenements dans la queue
        self.event_queue = []        # Etudie la nouvelle commande
        command_name = msg.command
        if command_name not in self.commands:
            self.get_logger().warn(f"Unknown command: {command_name}")
            return
        
        # Recupere les behaviors et delais pour la command
        events = self.commands[command_name]
        
        # Ajout des events a la queue avec les temps programmés
        for delay, behavior in events:
            event_time = self.start_time + delay  # ajusté par rapport au temps présent
            self.event_queue.append((event_time, behavior))

        # tri des évenements par date
        self.event_queue.sort(key=lambda x: x[0])

        self.get_logger().info(f"Command '{command_name}' received, scheduling behaviors.")
    
    
    def _on_time(self):
        """
        Cette methode est appelée periodiquement
        Elle check si des behavior doivent etre engagés, et les active le cas echeant
        """

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # Verification de si des evenements doivent etre engagés
        while self.event_queue and self.event_queue[0][0] <= current_time:
            event_time, behavior = self.event_queue.pop(0)
            self._activate_behavior(behavior)
            self.get_logger().info(f"Executing behavior '{behavior}' at time {current_time}")
    
    def _activate_behavior(self, behavior_name):
        """
        Cette methode active le behavior en publiant le behavior au topic
        """
        behavior_msg = BehaviorStatus()
        behavior_msg.name = behavior_name
        behavior_msg.status = True
        self.behavior_publisher.publish(behavior_msg)
    
    def _deactivate_all_behaviors(self):      
        self.get_logger().info("Deactivating all behaviors...")
        for behavior in self.behaviors :
            behavior_msg = BehaviorStatus()
            behavior_msg.name = behavior
            behavior_msg.status = False
            self.behavior_publisher.publish(behavior_msg)
        

def main(args=None):
    rclpy.init(args=args)
    command = Node('command')
    rclpy.spin(command)
    command.destroy_node()
    rclpy.shutdown()




