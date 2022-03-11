#!/usr/bin/env python

import argparse
import math
import sys

import numpy
import rclpy
import std_msgs
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnEntity


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class VirtualCO2Publisher:
    VIRTUAL_SOURCE = dotdict({
        "latitude": 35.19465,
        "longitude": -106.59625
    })

    def __init__(self, id, node):
        self.id = id
        self.pub = node.create_publisher(String, "{}/co2".format(id), 10)
        self.node = node

    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360))
        ]

    def calculateCO2(self, position):

        [y, x] = self.differenceInMeters(position, self.VIRTUAL_SOURCE)

        if x >= 0:
            return 420

        Q = 5000
        K = 2
        H = 2
        u = 1

        value = (Q / (2 * math.pi * K * -x)) * math.exp(- (u * ((pow(y, 2) + pow(H, 2))) / (4 * K * -x)))

        if value < 0:
            return 420
        else:
            return 420 + value

    def position_callback(self, data):

        co2 = self.calculateCO2(data)

        self.pub.publish(String(data="M 55146 52516 {} 55.0 0.0 0.0 800 55.0 55.0 00".format(co2)))

    def publish(self):
        self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(self.id),
                                      self.position_callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def spawn(self):
        print("differenceInMeters:", self.differenceInMeters(dotdict({"latitude": self.VIRTUAL_SOURCE.latitude + 0.0001,
                                           "longitude": self.VIRTUAL_SOURCE.longitude + 0.0001
                                           }), self.VIRTUAL_SOURCE))
        print("virtual_co2: spawning co2 spheres")
        spawn_entity_client = self.node.create_client(srv_type=SpawnEntity, srv_name='spawn_entity')
        spawn_entity_client.wait_for_service(timeout_sec=1.0)
        req = SpawnEntity.Request()
        req.xml = open("/workspace/src/dragonfly/resource/co2_sphere_model.sdf", "r").read().replace("\n", "")
        co2_spawn_count = 0
        co2_outof = 0
        #co2_spawn_sum = 0
        # avg  1.5765102669487028
        # 0.0001 lat/log -> ~9.082532/~11.11333 m
        for delta_lat in numpy.arange(-0.0001, 0.0001, .000005):
            for delta_long in numpy.arange(-0.0001, 0.0001, .000005):
                co2_outof += 1
                req.name = "co2:" + str(co2_spawn_count)
                co2_val = self.calculateCO2(dotdict({"latitude": self.VIRTUAL_SOURCE.latitude + delta_lat,
                                           "longitude": self.VIRTUAL_SOURCE.longitude + delta_long
                                           })) - 420
                # maximize | -(1250 e^((4 + y^2)/(8 x)))/(π x)  ~= 292.74915
                #co2_val = 1 if co2_val == 420 else 1 - (co2_val / 3)  # @TODO figure out what the range is supposed to look like
                #co2_val = 0 if co2_val < 0 else co2_val
                if co2_val != 0:
                    #print("Got co2_val:", co2_val)
                    req.xml = req.xml.replace("<transparency>0.5</transparency>", "<transparency>{}</transparency>".
                                              format(co2_val))
                    co2_spawn_count += 1
                    x, y = self.differenceInMeters(dotdict({"latitude": self.VIRTUAL_SOURCE.latitude + delta_lat,
                             "longitude": self.VIRTUAL_SOURCE.longitude + delta_long
                             }), self.VIRTUAL_SOURCE)
                    req.initial_pose.position.x = x
                    req.initial_pose.position.y = y
                    req.initial_pose.position.z = 5.0  # meters
                    spawn_entity_client.call_async(req)
        print("virtual_co2: ", co2_spawn_count, "/", co2_outof, "co2 spheres requested to be spawned")


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('virtual_co2')

    parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publisher = VirtualCO2Publisher(args.id, node)
    publisher.spawn()  # TODO add a ros param to enable/disable this call
    publisher.publish()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
