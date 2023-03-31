#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
import sys
import os
import copy
import numpy as np
import json
import time

from math import cos,sin,sqrt,pow,atan2,pi
import heapq
from geometry_msgs.msg import Point32,PoseStamped
from nav_msgs.msg import Odometry,Path

current_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_path)

from lib.mgeo.class_defs import *

class dijkstra_path_pub :
    def __init__(self):
        rospy.init_node('dijkstra_path_pub', anonymous=True)

        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size = 1)

        #TODO: (1) Mgeo data 읽어온 후 데이터 확인
        load_path = os.path.normpath(os.path.join(current_path, 'lib/mgeo_data/R_KR_PG_K-City'))
        mgeo_planner_map = MGeo.create_instance_from_json(load_path)

        self.nodes = mgeo_planner_map.node_set.nodes
        self.links = mgeo_planner_map.link_set.lines
        
        self.start_node = 'A119BS010184'
        self.end_node = 'A119BS010148'

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = '/map'
        
        self.global_path_msg = self.push_path_msg(self.start_node, self.end_node)
        rospy.loginfo('end')

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()
            
    def push_path_msg(self, start_node, end_node):
        resurt, path = self.find_shortest_path(start_node, end_node)
        
        out_path = Path()
        out_path.header.frame_id = '/map'
        
        for point in path['point_path']:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1
            out_path.poses.append(pose)
        
        return out_path
    
    def find_shortest_path(self, start_node_id, end_node_id):
        distances = dict()
        from_node = {}
        from_link = {}
        for node_id in self.nodes.keys():
            distances[node_id] = float('inf')
        
        distances[start_node_id] = 0
        queue = []
        heapq.heappush(queue, [distances[start_node_id], start_node_id])
        
        while queue:
            current_distance, current_node_id = heapq.heappop(queue)
            
            if current_node_id == end_node_id:
                break
            
            if distances[current_node_id] < current_distance:
                continue
            
            for link in self.nodes[current_node_id].get_to_links():
                adjacent_node_id = link.to_node.idx
                distance = current_distance + link.cost
                
                if distance < distances[adjacent_node_id]:
                    distances[adjacent_node_id] = distance
                    from_node[adjacent_node_id] = current_node_id
                    from_link[adjacent_node_id] = link.idx
                    heapq.heappush(queue, [distance, adjacent_node_id])
        
        node_path = []
        link_path = []
        shortest_path = end_node_id
        
        while shortest_path != start_node_id:
            node_path.insert(0, shortest_path)
            link_path.insert(0, from_link[shortest_path])
            shortest_path = from_node[shortest_path]
        
        point_path = []        
        for link_id in link_path:
            link = self.links[link_id]
            for point in link.points:
                point_path.append([point[0], point[1], 0])
        
        return True, {'node_path': node_path, 'link_path':link_path, 'point_path':point_path}


if __name__ == '__main__':
    
    dijkstra_path_pub = dijkstra_path_pub()