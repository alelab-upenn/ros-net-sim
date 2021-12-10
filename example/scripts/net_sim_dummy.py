#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gzip
import json
import multiprocessing
import os
import socket
import struct
import time

import numpy as np
import select
import yaml

import protobuf_msgs.channel_data_pb2 as cd
import protobuf_msgs.network_update_pb2 as netud
from network_coordinator import NetworkCoordinator



def run_protobuf_server(config):
    if config['net_use_uds']:
        server_address = config['netsim_uds_server_address']
        # Make sure the socket does not already exist
        try:
            os.unlink(server_address)
        except OSError:
            if os.path.exists(server_address):
                raise

        # Create a UDS socket
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    else:
        server_address = (config['netsim_ip_server_address'], config['netsim_ip_server_port'])
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

    sock.bind(server_address)

    try:
        print("Hit Ctrl-c to exit")
        sock.listen(1)
        connection, client_address = sock.accept()
        while True:
            try:
                data = NetworkCoordinator.recv_one_message(connection)
                data = gzip.compress(gen_response(parse_request(gzip.decompress(data)),config['ip_list']))
                NetworkCoordinator.send_one_message(connection, data)
            except socket.error:
                raise KeyboardInterrupt

    except KeyboardInterrupt:
        print("\nExiting network simulator dummy main process")

    finally:
        sock.close()
        if config['net_use_uds']:
            os.unlink(server_address)


def parse_request(req):
    time_update = netud.NetworkUpdate()
    time_update.ParseFromString(req)

    if time_update.channel_data:
        parse_phy_message(gzip.decompress(time_update.channel_data))
    time_update.channel_data = b""
    return time_update


def gen_response(time_update,ip_list):
    time_update.msg_type = netud.NetworkUpdate.END
    for i, id_i in enumerate(time_update.pkt_id):
        ip_dst=socket.inet_ntoa(struct.pack('!L', time_update.dst_ip[i]))
        if ip_dst in ip_list:
            print("Packet captured. Destination: %s. Delaying for 500ms." % ip_dst)
            time.sleep(0.5)
        time_update.ber.append(1e-9)
        time_update.rx_ip.append(time_update.dst_ip[i])  # not dealing with broadcast
    del time_update.pkt_lengths[:]
    return time_update.SerializeToString()


def parse_phy_message(data):
    channel_data = cd.ChannelData()
    channel_data.ParseFromString(data)
    num_nodes = int(len(channel_data.node_list) / 7)
    nodes = np.array(channel_data.node_list).reshape((num_nodes, 7))

    los = 0
    other = 0

    if len(channel_data.path_details):
        path_details = {}
        for path_detail in channel_data.path_details: # for each pair of nodes
            path_id = tuple(path_detail.ids)
            if not path_detail or not path_id:
                continue
            path_details[path_id] = {'los': path_detail.los}
            los += int(path_detail.los)

            hop_points_index = 0
            for num_hops_in_path in path_detail.num_hops: # for each path for given nodes
                points_in_path = path_detail.hop_points[hop_points_index: hop_points_index + (num_hops_in_path * 4)]
                path_details[path_id].setdefault(num_hops_in_path, []).extend(points_in_path)

                hop_points_index += (num_hops_in_path * 4)
                other += 1

    else:
        path_details = None

def driver_process(config):
    if config['net_use_uds']:
        server_address = config['net_driver_uds_server_address']
        try:
            os.unlink(server_address)
        except OSError:
            if os.path.exists(server_address): raise

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    else:
        server_address = (config['netsim_ip_server_address'], config['netsim_ip_ranging_port'])
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

    try:
        sock.bind(server_address)
        sock.listen(1)
        driver_connection, _ = sock.accept()
        while True:
            r, __, __ = select.select((driver_connection,), [], [], 2)
            if not r: continue
            try:
                driver_requests = NetworkCoordinator.recv_one_message(driver_connection)
                if not driver_requests: break
                driver_requests = json.loads(gzip.decompress(driver_requests))

                driver_responses = []
                for request in driver_requests:
                    if request.get("type", None) == "driver_request":
                        src_mac = request.get("src_mac", None)
                        dst_mac = request.get("dst_mac", None) if src_mac else None
                        if dst_mac:
                            request["type"] = "driver_reply"
                            driver_responses.append(request)
                if driver_responses:
                    NetworkCoordinator.send_one_message(driver_connection,
                                          gzip.compress(json.dumps(driver_responses).encode("utf-8")))
            except socket.error:
                return
    finally:
        print("\nExiting network simulator dummy driver process")
        sock.close()
        if config['net_use_uds']: os.unlink(server_address)



def main(args):
    # Capture config
    config_file = None
    for arg in args:
        if ".yaml" in arg:
            config_file = arg
            break
        
    if config_file is None:
        print("usage: net_sim_dummy.py <config_file>")
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
