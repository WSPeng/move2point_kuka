#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from move_to_a_point import Move2Point


def main ():
    # init the node
    rospy.init_node('move2point', anonymous=True)

    # define the frequency
    rate = rospy.Rate(10)

    if not Move2Point.__init__():
        return -1
    else:
        Move2Point.move2goal()

    return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
