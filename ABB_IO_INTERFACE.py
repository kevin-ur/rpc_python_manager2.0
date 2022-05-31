#!/usr/bin/env python3

import time
from datetime import datetime
from threading import Lock
from concurrent.futures import ThreadPoolExecutor
import socket

# from time import time# 客户端 发送一个数据，再接收一个数据
# client = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #声明socket类型，同时生成链接对象
# server_ip='192.168.125.1'
# server_port=1025
# client.connect((server_ip,server_port)) #建立一个链接，连接到本地的6969端口

class ABB_IO_INTERFACE:
    def __init__(self, server_ip, server_port,simulation=True):
        self.ip = server_ip
        self.port = server_port
        self.sim=simulation
        self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #声明socket类型，同时生成链接对象
        if (simulation==False):
           self.client.connect((server_ip,server_port))

     
    def vacuum_on(self):
        if (self.sim==True):
            return "simulation"
        msg = 'vacuum_on'  #strip默认取出字符串的头尾空格
        self.client.send(msg.encode('utf-8'))  #发送一条信息 python3 只接收btye流
        data = self.client.recv(1024) #接收一个信息，并指定接收的大小 为1024字节
        print('recv:',data.decode()) #输出我接收的信息
        return data.decode()

    def vacuum_off(self):
        if (self.sim==True):
            return "simulation"
        msg = 'vacuum_off'  #strip默认取出字符串的头尾空格
        self.client.send(msg.encode('utf-8'))  #发送一条信息 python3 只接收btye流
        data = self.client.recv(1024) #接收一个信息，并指定接收的大小 为1024字节
        print('recv:',data.decode()) #输出我接收的信息
        return data.decode()
    def query_di01(self):
            if (self.sim==True):
                return "False"
            msg = 'update_io'  #strip默认取出字符串的头尾空格
            self.client.send(msg.encode('utf-8'))  #发送一条信息 python3 只接收btye流
            data = self.client.recv(1024) #接收一个信息，并指定接收的大小 为1024字节
            print('recv:',data.decode()) #输出我接收的信息
            return data.decode()

# abb_io = ABB_IO_INTERFACE("127.0.0.1", 21)
# if abb_io.vacuum_on()=='hh':
#     print("Hello")
# elif True:
#     print("Jakc")

