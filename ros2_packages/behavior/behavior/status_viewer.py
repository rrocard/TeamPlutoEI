import rclpy
import rclpy.node
import tkinter as tk
from multiprocessing import Lock

from behavior_interface.msg import BehaviorStatus

import sys

class Node(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('status_viewer')
        self.sub_behaviors_status = self.create_subscription(BehaviorStatus, 'behaviors_status', self._on_behaviors_status, 1)
        self.pub_ping= self.create_publisher(BehaviorStatus, 'behavior', 1)
        
        self.create_timer(0.5, callback = self._on_ping)
        self.create_timer(0.05, callback = self._on_evt_loop)
        self.create_timer(4.0, callback = self._on_last_seen)
        
        self.ping = BehaviorStatus()
        self.ping.name = 'ping'

        self.mutex = Lock()
        self.kill_me = False

        self.labels = {}
        self.last_seen = {}
        self.last_seen_time = self.get_clock().now()
        
        self.realize()

    def terminate(self):
        self.kill_me = True

            
    def _on_behaviors_status(self, msg):
        with self.mutex:
            if msg.name not in self.labels:
                label = tk.Label(self.pad, text=msg.name)
                label.configure(anchor='w', padx=5)
                label.pack(fill=tk.X)
                self.labels[msg.name] = label
            else:
                label = self.labels[msg.name]
            self.last_seen[msg.name] = self.get_clock().now()
            if msg.status :
                bg_color = self.active_bg
                fg_color = self.active_fg
            else :
                bg_color = self.inactive_bg
                fg_color = self.inactive_fg
            label.configure(background = bg_color, foreground = fg_color)
            
    def _on_ping(self) :
        self.pub_ping.publish(self.ping)

    def _on_evt_loop(self):
        if self.kill_me:
            ugly_way_to_quit_with_a_non_existing_function()
        self.root.update_idletasks()
        self.root.update()

    def _on_last_seen(self):
        with self.mutex:
            for lbl, date in self.last_seen.items():
                if date < self.last_seen_time:
                    self.labels[lbl].configure(background = self.error_bg, foreground = self.error_fg)
        self.last_seen_time = self.get_clock().now()

    def realize(self):
        self.root = tk.Tk()
        self.root.title('Behaviors')
        self.root.configure(relief='flat', padx=5, pady=5)

        self.root.protocol("WM_DELETE_WINDOW", self.terminate)

        title = tk.Label(self.root,text='Behaviors')
        title.pack()

        self.pad = tk.Frame(self.root)
        self.pad.configure(relief='sunken', borderwidth=2)
        self.pad.pack()

        self.active_bg = '#0000aa'
        self.active_fg = '#ccccff'
        self.inactive_bg = self.root.cget('bg')
        self.inactive_fg = '#0000aa'
        self.error_bg = '#aaaa00'
        self.error_fg = '#ff0000'


def main(args=None):
    rclpy.init(args=args)
    n = Node()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()
