#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from network_coordinator import ArgosNetCoordinator
from typing import List


def main(args: List[str]) -> any:
    config_file = None
    for arg in args:
        if ".yaml" in arg:
            config_file = arg
            break

    if config_file is None:
        print("usage: network_coordinator.py <config_file>")
        return 1

    rospy.init_node(name='argos_net_coord')
    net_coord = ArgosNetCoordinator(config_file)
    net_coord.run_network_coordinator()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
