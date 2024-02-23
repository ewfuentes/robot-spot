#! /usr/bin/env python3

import argparse
import sys

import rospy


def callback(data):
    rospy.loginfo()
    ...


def main(topic_list):
    rospy.init_node('observation_node')
    rospy.loginfo(topic_list)
    for topic in topic_list:
        rospy.Subscriber(topic, , callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        ...

    rospy.loginfo("finished")


if __name__ == '__main__':
    cameras = ['frontleft', 'frontright', 'left', 'right', 'back']
    topic_list = [
        f"/spot/apriltag/{camera}/tag_detections"
        for camera in cameras
    ]
    main(topic_list)
