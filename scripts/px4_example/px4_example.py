#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Thats just a code to make uav flight on circle and drain battery ##

import rospy
import numpy as np
from math import cos,sin
from numpy.linalg import norm
import time
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import sys

class Action:
    
    def __init__(self, namespace):

        self.namespace = namespace
        self.current_state = State()
        self.p = PoseStamped()
        self.last_req = rospy.Time.now()

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True
        
        header = "/{}".format(self.namespace)

        self.pose_init = [int(rospy.get_param(header+'/x_i')),
                          int(rospy.get_param(header+'/y_i')),
                          int(rospy.get_param(header+'/z_i'))]

        # Publicadores e subscritores com namespace específico
        self.local_pos_pub = rospy.Publisher(header + "/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.state_sub = rospy.Subscriber(header + "/mavros/state", State, callback=self.get_state_cb)
        self.arming_client = rospy.ServiceProxy(header + "/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(header + "/mavros/set_mode", SetMode)
        self.vel_pub = rospy.Publisher(header + '/mavros/setpoint_velocity/cmd_vel', PoseStamped, queue_size=10)
        self.accel_pub = rospy.Publisher(header + '/mavros/setpoint_accel/accel', PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber(header + '/mavros/local_position/odom', Odometry, self.pose_cb)
        #self.odom_sub = rospy.Subscriber(header + '/tf_local_pose', Odometry, self.pose_cb)
        
        self.rate = rospy.Rate(20)

        rospy.loginfo("Publicando setpoints iniciais para {}".format(namespace))
        for _ in range(100):
            if rospy.is_shutdown():
                break
            self.local_pos_pub.publish(self.p)
            self.rate.sleep()

    def pose_cb(self, data):
        self.lugar = data

    def get_state_cb(self, state):
        self.current_state = state

    def envia_acao(self, target):
        # Armar e configurar modo OFFBOARD
        if not self.current_state.armed:
            rospy.loginfo("Armando o drone...")
            if self.arming_client(self.arm_cmd).success:
                rospy.loginfo("Drone armado com sucesso.")
            else:
                rospy.logerr("Falha ao armar o drone.")
        
        if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0):
            rospy.loginfo("Alterando para modo OFFBOARD...")
            if self.set_mode_client(self.offb_set_mode).mode_sent:
                rospy.loginfo("Modo OFFBOARD ativado.")
            else:
                rospy.logerr("Falha ao alterar para modo OFFBOARD.")
            self.last_req = rospy.Time.now()

        # Configurar e enviar setpoint
        self.p.header.stamp = rospy.Time.now()
        self.p.header.frame_id = "map"
        ## aqui e uma gambiarra pois é preciso compensar 
        self.p.pose.position = Point(target[0]-self.pose_init[0], 
                                     target[1]-self.pose_init[1], 
                                     target[2]-self.pose_init[2])
        self.p.pose.orientation = Quaternion(0, 0, 0, 1)

        rospy.loginfo("Enviando alvo: {}".format(target))
        self.local_pos_pub.publish(self.p)

    def wait_target(self, alvo):
        rospy.loginfo("Aguardando chegada ao alvo: {}".format(alvo))
        while norm(np.array([self.lugar.pose.pose.position.x,
                             self.lugar.pose.pose.position.y,
                             self.lugar.pose.pose.position.z]) - np.array(alvo)) > 1:
            self.envia_acao(alvo)
            self.rate.sleep()

class RVizMission:
    
    def __init__(self):
        rospy.loginfo("Inicializando RVizMission")
        self.mission_marker = rospy.Publisher('/rviz/mission_marker', Marker, queue_size=10)
        self.pt_interesse = []
        rospy.Subscriber("/pontos_interesse", MarkerArray, callback=self.pt_interesse_cb)

    def pt_interesse_cb(self, data):
        self.pt_interesse = [[marker.pose.position.x, marker.pose.position.y] for marker in data.markers]
        rospy.loginfo("Pontos de interesse recebidos: {}".format(self.pt_interesse))

    def construct_circle_target(self):
        rospy.loginfo("Construindo missão em formato de círculo.")
        x, y = [], []
        for point in self.pt_interesse:
            theta = np.linspace(0, 2 * np.pi, 10)
            radius = 2
            a = point[0] + radius * np.cos(theta)
            b = point[1] + radius * np.sin(theta)
            x += a.tolist()
            y += b.tolist()
        rospy.loginfo("Missão construída.")
        return x, y
    
    def draw_mission(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.type = 4  # LINE_STRIP
        marker.id = 33
        marker.scale.x = 0.20
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [Point(x1, y1, 10) for x1, y1 in zip(x, y)]
        
        rospy.loginfo("Missão desenhada no RViz.")
        return marker

def execute():
    if len(sys.argv) < 2:
        rospy.logerr("Por favor, forneça o namespace do UAV como argumento.")
        return

    namespace = sys.argv[1]
    node_name = "{}".format(namespace)
    rospy.init_node(node_name+"_missao", anonymous=False)
    uav = Action(namespace=namespace)
    rviz = RVizMission()

    rospy.sleep(1)

    x = []
    y = []
    a = np.arange(-3.14,3.14,1)

    for a1 in a:
      x.append(0 +  int(sys.argv[2])* cos(a1))
      y.append(0 +  int(sys.argv[2])* sin(a1))

    while not rospy.is_shutdown():
        for point in zip(x,y):
            uav.envia_acao( [point[0], point[1], 5])
            uav.wait_target([point[0], point[1], 5])

if __name__ == '__main__':
    try:
        execute()
    except rospy.ROSInterruptException:
        pass