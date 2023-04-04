#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point32
from sensor_msgs.msg import PointCloud

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

'''
1. publish 함수 정의
2. Mgeo 데이터 읽어오기
3. publish 하여 rviz 로 확인
'''

class get_mgeo :
    def __init__(self):
        #TODO: (1) publish 함수 정의
        rospy.init_node('mgeo_test', anonymous=True)
        self.lane_boundary_pub = rospy.Publisher('/lane_boundary', PointCloud, queue_size=1)
        self.lane_node_pub = rospy.Publisher('/lane_node', PointCloud, queue_size=1)
        self.link_pub = rospy.Publisher('/link', PointCloud, queue_size=1)
        self.node_pub = rospy.Publisher('/node', PointCloud, queue_size=1)

        #TODO: (2) Mgeo 데이터 읽어오기
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)
        
        self.lanes = mgeo_planner_map.lane_boundary_set.lanes
        self.lane_nodes = mgeo_planner_map.lane_node_set.nodes
        self.links = mgeo_planner_map.link_set.lines
        self.nodes = mgeo_planner_map.node_set.nodes
        
        self.lane_msg = self.getAllLanes()
        self.lane_node_msg = self.getAllLaneNodes()
        self.link_msg=self.getAllLinks()
        self.node_msg=self.getAllNode()

        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():

            self.lane_boundary_pub.publish(self.lane_msg)
            self.lane_node_pub.publish(self.lane_node_msg)
            self.link_pub.publish(self.link_msg)
            self.node_pub.publish(self.node_msg)
                
            rate.sleep()


    def getAllLanes(self):
        point_cloud = PointCloud()
        point_cloud.header.frame_id = '/map'
        
        for idx in self.lanes:
            for point in self.lanes[idx].points:
                temp_pt = Point32()
                temp_pt.x = point[0]
                temp_pt.y = point[1]
                temp_pt.z = 0.0
                point_cloud.points.append(temp_pt)
        
        return point_cloud


    def getAllLaneNodes(self):
        point_cloud = PointCloud()
        point_cloud.header.frame_id = '/map'
        
        for idx in self.lane_nodes:
            temp_pt = Point32()
            temp_pt.x = self.lane_nodes[idx].point[0]
            temp_pt.y = self.lane_nodes[idx].point[1]
            temp_pt.z = 0.0
            point_cloud.points.append(temp_pt)
        
        return point_cloud


    def getAllLinks(self):
        all_link=PointCloud()
        all_link.header.frame_id='map'

        #TODO: (2) Link 정보 Point Cloud 데이터로 변환
        for idx in self.links :
            for point in self.links[idx].points:
                temp_pt = Point32()
                temp_pt.x = point[0]
                temp_pt.y = point[1]
                temp_pt.z = 0.0
                all_link.points.append(temp_pt)

        return all_link
    
    def getAllNode(self):
        all_node=PointCloud()
        all_node.header.frame_id='map'

        #TODO: (3) Node 정보 Point Cloud 데이터로 변환
        for node_idx in self.nodes :
            tmp_point=Point32()
            tmp_point.x=self.nodes[node_idx].point[0]
            tmp_point.y=self.nodes[node_idx].point[1]
            tmp_point.z=0.
            all_node.points.append(tmp_point)

        return all_node


if __name__ == '__main__':
    
    test_track=get_mgeo()