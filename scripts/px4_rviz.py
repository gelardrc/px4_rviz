#! /usr/bin/env python
# -*- coding: utf-8 -*-
## Just a dumb way to show px4 drone in rviz ##
## author : gelo ##
## gabriel.guitar@gmail.com ##

import rospy
import sys
from collections import deque
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

local_position = PoseStamped()

def local_position_cb(msg):
    global local_position
    local_position = msg.pose


def main(id,header):
    global local_position
    path_points = deque(maxlen=2000) 

    #print(header+'/tf_pose')
    pub_marker = rospy.Publisher(header+'/marker', Marker, queue_size=10)
    pub_path = rospy.Publisher(header+'/path', Marker, queue_size=10)

    #rospy.Subscriber(header+'/tf_local_pose',Odometry , local_position_cb)
    #rospy.Subscriber(header+'/tf_local_pose',Odometry , local_position_cb)
    rospy.Subscriber(header + '/mavros/local_position/odom', Odometry, local_position_cb)



    while not rospy.is_shutdown():
        uav  = Marker()
        uav.header.frame_id = "map"
        uav.header.stamp = rospy.Time.now()
        uav.type = uav.MESH_RESOURCE
        uav.id = int(id) 

        uav.mesh_resource = 'package://px4_rviz/models/uav_rviz_model/iris.stl' 
        uav.mesh_use_embedded_materials = True  # Need this to use textures for mesh
        uav.scale.x = 1.00
        uav.scale.y = 1.80
        uav.scale.z = 1.80
        uav.pose = local_position.pose

        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = rospy.Time.now()
        path_marker.type = path_marker.LINE_STRIP
        path_marker.id = int(id) + 100  # ID diferente do UAV

        # Configuração da linha
        path_marker.scale.x = 0.1  # Espessura da linha
        path_marker.color.r = 1.0  # Cor vermelha
        path_marker.color.a = 1.0  # Opacidade

        # Adiciona o ponto atual ao caminho
        current_point = local_position.pose.position
        path_points.append(current_point)
        path_marker.points = path_points

        #rate = rospy.Rate(1)

        pub_marker.publish(uav)
        pub_path.publish(path_marker)

        
        #rate.sleep()



if __name__ == '__main__':
    
    #print(sys.argv)
    
    if len(sys.argv) < 2 :
        str_id = "1"
        header = ""
    
    else : 
        str_id = sys.argv[1] ## first argument
        header  = "/uav"+sys.argv[1]

    rospy.init_node("uav_"+str_id,anonymous=False)
    
    main(id = str_id,header=header)
    
    #rospy.spin()

    