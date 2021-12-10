#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gzip
import json
import multiprocessing
import os
import random
import socket
import subprocess
import threading
import time

import numpy as np
import select
from pyquaternion import Quaternion
import yaml

import protobuf_msgs.channel_data_pb2 as cd
import protobuf_msgs.physics_update_pb2 as phyud
from network_coordinator import NetworkCoordinator


def run_protobuf_server(config):
    if config['phy_use_uds']:
        server_address = config['phy_uds_server_address']
        # Make sure the socket does not already exist
        try:
            os.unlink(server_address)
        except OSError:
            if os.path.exists(server_address):
                raise
        # Create a UDS socket
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    else:
        server_address = (config['phy_ip_server_address'], config['phy_ip_server_port'])
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

    sock.bind(server_address)

    try:
        prev_time=0
        print("Hit Ctrl-c to exit")
        sock.listen(1)
        connection, client_address = sock.accept()
        while True:
            try:
                num_nodes = len(config['mac_list'])
                data = gzip.decompress(NetworkCoordinator.recv_one_message(connection))
                data = gzip.compress(gen_response(parse_request(data),num_nodes))
                cur_time = time.time()
                if cur_time - prev_time > 2:
                    prev_time=time.time()
                    time_update = phyud.PhysicsUpdate()
                    time_update.ParseFromString(gzip.decompress(data))
                    channel_data = cd.ChannelData()
                    channel_data.ParseFromString(gzip.decompress(time_update.channel_data))
                    print(channel_data)

                NetworkCoordinator.send_one_message(connection, data)
            except socket.error:
                raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("\nExiting physics simulator dummy main process")

    finally:
        sock.close()
        if config['phy_use_uds']:
            os.unlink(server_address)


def parse_request(req):
    time_update = phyud.PhysicsUpdate()
    time_update.ParseFromString(req)
    return time_update


def gen_response(time_update,num_nodes):
    time_update.msg_type = phyud.PhysicsUpdate.END
    data = generate_data(num_nodes)
    time_update.channel_data = gzip.compress(data)
    return time_update.SerializeToString()


def generate_data(num_nodes):
    channel_data = cd.ChannelData()

    for __ in range(num_nodes):
        channel_data.node_list.extend([float(20 * np.random.ranf((1, 1)))])
        channel_data.node_list.extend([float(20 * np.random.ranf((1, 1)))])
        channel_data.node_list.extend([float(20 * np.random.ranf((1, 1)))])
        channel_data.node_list.extend(Quaternion.random())

    los = 0
    other = 0

    for i in range(random.randint(1, 2)):  # select pairs of nodes
        path_details = channel_data.path_details.add()
        a, b = random.sample((range(1, num_nodes + 1)), k=2)

        path_details.ids.extend([a, b])
        path_details.los = True
        los += int(path_details.los)

        for j in range(random.randint(0, 3)):  # number of paths for given nodes
            k = random.randint(1, 3)  # number of hops in path
            path_details.num_hops.append(k)
            # import pdb; pdb.set_trace()
            path_details.hop_points.extend(np.random.rand(1, (4 * k)).tolist()[0])
            other += 1

    return channel_data.SerializeToString()


def driver_process(config):
    addresses = []
    socket_list = []
    try:
        addresses = [config['phy_driver_uds_server_address'] + str(i) for i, mac_i in enumerate(config['mac_list'])]

        def connect(id, addr):
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.bind(addr)
            socket_list.append(sock)
            sock.listen(1)
            connection, _ = sock.accept()
            print(f"Connected to {addr}")
            if id == 1: time.sleep(0.5)
            mac_tuple = ("1a", "2a")
            src_mac, dst_mac = mac_tuple if id == 0 else mac_tuple[::-1]

            request = '''{"type": "driver_request","src_mac": "%s","dst_mac": "%s"}''' % (src_mac, dst_mac)

            try:
                while True:
                    print(f"Sent: " + (request))
                    NetworkCoordinator.send_one_message(connection, request.encode("utf-8"))
                    r, __, __ = select.select([connection, ], [], [], 0)
                    if r: print(f"Received: " + (NetworkCoordinator.recv_one_message(connection)).decode("utf-8"))
                    time.sleep(5)
            except socket.error:
                return

        threads = []
        id = 0
        for a in addresses:
            thread = threading.Thread(target=connect, args=(id, a,))
            thread.setDaemon(True)
            thread.start()
            threads.append(thread)
            id += 1
        for t in threads: t.join()

    finally:
        print("\nExiting physics simulator dummy driver process")
        for sock in socket_list: sock.close()
        for address in addresses: os.unlink(address)


def main(args):
    # Capture config
    config_file = None
    for arg in args:
        if ".yaml" in arg:
            config_file = arg
            break
        
    if config_file is None:
        print("usage: phy_sim_dummy.py <config_file>")
    else:
        with open(config_file) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            if config['do_driver_transfer']:
                ranging_process = multiprocessing.Process(target=driver_process, args=(config,))
                ranging_process.start()
            run_protobuf_server(config)
            if config['do_driver_transfer']:
                ranging_process.join()
    return 0


if __name__ == '__main__':
    import sys

    sys.exit(main(sys.argv))
