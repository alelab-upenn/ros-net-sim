#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
from network_coordinator import NetworkCoordinator
from typing import List


def main(args: List[str]) -> any:
    config_file = None
    for arg in args:
        if ".yaml" in arg:
            config_file = arg
            break

    if config_file is None:
        print("usage: network_coordinator.py <config_file>")
    else:
        net_coord = NetworkCoordinator(config_file)
        net_coord.run_network_coordinator()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
