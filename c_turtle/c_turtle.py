#!/usr/bin/env python3

from math import radians, inf, isnan, pi, cos, sin, sqrt

import rclpy
from rclpy import Publisher, Node
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from flatland_msgs.srv import MoveModel

LASER_RANGE = 3
LASER_FREQ = 10

RAD22_5 = radians(22.5)
RAD45 = radians(45)
RAD67_5 = radians(67.5)
RAD90 = radians(90)
RAD112_5 = radians(112.5)
RAD135 = radians(135)
RAD157_5 = radians(157.5)


def clamp(val, minVal, maxVal):
    return float(max(min(val, maxVal), minVal))


def pointDist(p1, p2) -> float:
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def laserToPoint(laser):
    return (laser[1] * cos(laser[0]), laser[1] * sin(laser[0]))


class CTurtle(Node):
    maxLinVel = 2.0
    maxAngVel = 3.0
    linAcc = 1.5
    linDec = 3.0
    angAcc = 4.0
    angDec = 4.0

    minDistFromWall = 1.0
    k = 3

    def __init__(self) -> None:
        super().__init__("CTurtle")

        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_yaw", 0.0)

        self.initial_pose_x = self.get_parameter("initial_pose_x").value
        self.initial_pose_y = self.get_parameter("initial_pose_y").value
        self.initial_pose_yaw = self.get_parameter("initial_pose_yaw").value

        self.vel = Twist()

        self.pub: Publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(LaserScan, "/scan", self._scanCallback, 1)

    def _scanCallback(self, scan):
        lasers = [0] * len(scan.ranges)
        angle = scan.angle_min
        for i in range(len(scan.ranges)):
            dist = scan.ranges[i]
            lasers[i] = (angle, dist if not isnan(dist) else inf)
            angle += scan.angle_increment

        dirs = self._processLasers(lasers, scan.angle_increment)
        self.reactToScan(dirs)

    def _processLasers(self, lasers, angleIncrement):
        angleMin = lasers[0][0]

        dirs = {
            "front": {"dist": inf, "ang": 0},
            "front_left": {"dist": inf, "ang": RAD22_5 * 2},
            "front_right": {"dist": inf, "ang": -RAD22_5 * 2},
            "left": {"dist": inf, "ang": RAD22_5 * 4},
            "right": {"dist": inf, "ang": -RAD22_5 * 4},
            "back_left": {"dist": inf, "ang": RAD22_5 * 6},
            "back_right": {"dist": inf, "ang": -RAD22_5 * 6},
            "back": {"dist": inf, "ang": pi},
            "minDir": "front",
        }

        minDist = inf
        for laser in lasers:
            angle = laser[0]
            dist = laser[1]

            absAngle = abs(angle)
            if absAngle <= RAD22_5:
                key = "front"
            elif absAngle <= RAD67_5:
                key = "front_left" if angle > 0 else "front_right"
            elif absAngle <= RAD112_5:
                key = "left" if angle > 0 else "right"
            elif absAngle <= RAD157_5:
                key = "back_left" if angle > 0 else "back_right"
            else:
                key = "back"

            if dist < dirs[key]["dist"]:
                dirs[key]["dist"] = dist
                dirs[key]["ang"] = angle
                if dist < minDist:
                    dirs["minDir"] = key
                    minDist = dist

        return dirs

    def reactToScan(self, dirs):
        minDir = dirs["minDir"]
        minDist = dirs[minDir]["dist"]
        minAng = dirs[minDir]["ang"]

        if minDist == inf:
            self.wiggle()
            return

        if minDir.endswith("right"):
            front = min(dirs["front"]["dist"], dirs["front_left"]["dist"])
        elif minDir.endswith("left"):
            front = min(dirs["front"]["dist"], dirs["front_right"]["dist"])
        else:
            front = min(
                dirs["front"]["dist"],
                dirs["front_right"]["dist"],
                dirs["front_left"]["dist"],
            )
        self.linVel = CTurtle.maxLinVel * front / LASER_RANGE

        if minDir == "front":
            if dirs["left"]["dist"] == inf and dirs["right"]["dist"] != inf:
                wallSide = "right"
            elif dirs["left"]["dist"] != inf and dirs["right"]["dist"] == inf:
                wallSide = "left"
            else:
                minLeft = min(dirs["front_left"]["dist"], dirs["left"]["dist"])
                minRight = min(dirs["front_right"]["dist"], dirs["right"]["dist"])
                wallSide = "left" if minLeft < minRight else "right"
        elif minDir.endswith("left"):
            wallSide = "left"
        else:
            wallSide = "right"

        if wallSide == "left":
            angDistTerm = cos(minAng) + (CTurtle.minDistFromWall - minDist)
        else:
            angDistTerm = cos(pi - minAng) + (minDist - CTurtle.minDistFromWall)
        self.angVel = -CTurtle.k * self.linVel * angDistTerm

        self.moveTurtle()

    @property
    def linVel(self):
        return self.vel.linear.x

    @linVel.setter
    def linVel(self, newLinVel):
        desiredVel = clamp(newLinVel, -CTurtle.maxLinVel, CTurtle.maxLinVel)

        a = (desiredVel - self.linVel) * LASER_FREQ
        if a > 0:
            if a <= CTurtle.linAcc:
                self.vel.linear.x = desiredVel
            else:
                self.vel.linear.x = self.linVel + CTurtle.linAcc / LASER_FREQ
        elif a < 0:
            if a >= CTurtle.linDec:
                self.vel.linear.x = desiredVel
            else:
                self.vel.linear.x = self.linVel - CTurtle.linDec / LASER_FREQ

    @property
    def angVel(self):
        return self.vel.angular.z

    @angVel.setter
    def angVel(self, newAngVel):
        desiredVel = clamp(newAngVel, -CTurtle.maxAngVel, CTurtle.maxAngVel)

        a = (desiredVel - self.angVel) * LASER_FREQ
        if a > 0:
            if a <= CTurtle.angAcc:
                self.vel.angular.z = desiredVel
            else:
                self.vel.angular.z = self.angVel + CTurtle.angAcc / LASER_FREQ
        elif a < 0:
            if a >= CTurtle.angDec:
                self.vel.angular.z = desiredVel
            else:
                self.vel.angular.z = self.angVel - CTurtle.angDec / LASER_FREQ

    def wiggle(self):
        self.linVel = self.maxLinVel
        v = CTurtle.maxAngVel * random()
        if random() > 0.5:
            self.angVel += v
        else:
            self.angVel -= v
        if abs(self.angVel) >= CTurtle.maxAngVel:
            self.angVel = 0

        self.moveTurtle()

    def moveTurtle(self):
        self.pub.publish(self.vel)


def main(args=None):
    rclpy.init()

    cTurtle = CTurtle()
    rclpy.spin(cTurtle)

if __name__ == "__main__":
    main()