import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from behavior_interface.msg import BehaviorStatus
from .base_behavior import BaseBehavior  # Importing the base behavior class

class AutoOff(BaseBehavior):
    def __init__(self, behavior_name: str, duration_sec =  0.2):
        super().__init__(behavior_name)
        self.duration_sec = duration_sec  # Duration to stay active in seconds
        self.timer = None
        self.timer = self.create_timer(self.duration_sec, self.request_off)
        self.timer.cancel()


    def on_status(self, status: bool):
        if self.active:
            self.timer.reset()
        else:
            pass

    def request_off(self):
        self.active = False
        self.on_status(self.active)
        self.timer.cancel()


 
