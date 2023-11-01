#!/usr/bin/env python

import os
import csv
import math
from enum import Enum
from nav_msgs.msg import Path

LIMIT_X = 50
DT = 0.1


class waypoint():
    def __init__(self):
        self.ID = None
        self.x = None
        self.y = None
        self.yaw = None


class TrajectoryType(Enum):
    LINE = 1
    CIRCLE = 2
    WAVE = 3


class WaypointProduce(object):
    def __init__(self):
        self.TrajectoryType = TrajectoryType.WAVE.name
        self.waypoints = self.refer_path()
    
        csv_path = "/home/yjq/self-driving-vehicle-101/src/waypoint_loader/waypoints/points_produce.csv"
        with open(csv_path,"w",encoding="utf-8",newline="") as f:
            csv_writer = csv.writer(f)
            
            for i in range(len(self.waypoints)):
                csv_writer.writerow([self.waypoints[i].x, self.waypoints[i].y, self.waypoints[i].yaw])
            f.close()


    def generate_path(self):
        path = []
        if (self.TrajectoryType == TrajectoryType.LINE.name):
            for i in range(int(LIMIT_X/DT)):
                PP = waypoint()
                PP.ID = i
                PP.x = 0 + DT*i
                PP.y = 0
                path.append(PP)
        elif (self.TrajectoryType == TrajectoryType.CIRCLE.name):
            for i in range(int(LIMIT_X/DT/3)):
                PP = waypoint()
                PP.ID = i
                PP.x = 0 + 3 * (1 - math.cos(i * 2 * math.pi * DT / LIMIT_X))
                PP.y = 0 + math.sin(i * 2 * math.pi * DT / LIMIT_X)
                path.append(PP)
        elif (self.TrajectoryType == TrajectoryType.WAVE.name):
            for i in range(int(LIMIT_X/DT)):
                PP = waypoint()
                PP.ID = i
                PP.x = 0 + DT * i
                PP.y = 0 + 10.0 * math.sin(0.15*DT*i )
                path.append(PP)
        return path

    def refer_path(self):
        points = self.generate_path()
        for i in range(len(points)):
            if(i == 0):
                dx = points[1].x - points[0].x
                dy = points[1].y - points[0].y
            elif(i == len(points)-1):
                dx = points[i].x - points[i - 1].x
                dy = points[i].y - points[i - 1].y
            else:
                dx = points[i + 1].x - points[i].x
                dy = points[i + 1].y - points[i].y
            yaw = math.atan2(dy,dx)
            points[i].yaw = yaw
        return points


if __name__ == '__main__':
    WaypointProduce()

