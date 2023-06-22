#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


import socket
import sys  
import rich
import time
encoding = 'utf-8'

host = '192.168.3.2'
port = 10000    


class Tcpclient(Node):

    def __init__(self):
        self.cc = 0
        super().__init__('tcpclient')
        self.subscription = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.listener_callback,
            10)
        self.subscription
        # create socket
        print('# Creating socket')
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error:
            print('Failed to create socket')
            sys.exit()

        print('# Getting remote IP address') 
        try:
            remote_ip = socket.gethostbyname( host )
        except socket.gaierror:
            print('Hostname could not be resolved. Exiting')
            sys.exit()

        # Connect to remote server
        print('# Connecting to server, ' + host + ' (' + remote_ip + ')')
        self.s.connect((remote_ip , port))

    def listener_callback(self, msg):
        print(self.cc)
        if msg.linear.y != 0 or msg.linear.z != 0 or msg.angular.x != 0 or msg.angular.y != 0 :
            print("Not admissible cmd_vel")
        elif msg.linear.x != 0 and msg.angular.z != 0 :
            print("Close")
            try:
                self.s.sendall(b'q')
            except socket.error:
                print ('Send failed')
                sys.exit()

            """print('# Receive data from server')
            reply = self.s.recv(4096)
            rich.print((reply.decode(encoding)))"""
  
        elif msg.linear.x > 0 and msg.angular.z == 0 :
            print("8")
            try:
                self.s.sendall(b'8')
            except socket.error:
                print ('Send failed')
                sys.exit()

            """print('# Receive data from server')
            reply = self.s.recv(4096)
            rich.print((reply.decode(encoding)))"""

        elif msg.linear.x < 0 and msg.angular.z == 0 :
            print("2")
            try:
                self.s.sendall(b'2')
            except socket.error:
                print ('Send failed')
                sys.exit()

            """print('# Receive data from server')
            reply = self.s.recv(4096)
            rich.print((reply.decode(encoding)))"""    

        elif msg.linear.x == 0 and msg.angular.z < 0 :
            print("6")
            try:
                self.s.sendall(b'6')
            except socket.error:
                print ('Send failed')
                sys.exit()

            """print('# Receive data from server')
            reply = self.s.recv(4096)
            rich.print((reply.decode(encoding)))"""   

        elif msg.linear.x == 0 and msg.angular.z > 0 :
            print("4")
            try:
                self.s.sendall(b'4')
            except socket.error:
                print ('Send failed')
                sys.exit()

            """print('# Receive data from server')
            reply = self.s.recv(4096)
            rich.print((reply.decode(encoding)))"""   

        elif msg.linear.x == 0 and msg.angular.z == 0 :
            print("5")
            try:
                self.s.sendall(b'5')
            except socket.error:
                print ('Send failed')
                sys.exit()

            """print('# Receive data from server')
            reply = self.s.recv(4096)
            rich.print((reply.decode(encoding)))"""
        self.cc += 1
        print('End of loops')   


def main(args=None):
    rclpy.init(args=args)

    tcpclient = Tcpclient()

    rclpy.spin(tcpclient)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tcpclient.s.close()
    tcpclient.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()