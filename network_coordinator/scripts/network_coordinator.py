#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import gzip
import ipaddress
import json
import multiprocessing
import os
import socket
import struct
import subprocess
import sys
import threading
import time

import array
import fcntl
import numpy as np
import select
import yaml

import protobuf_msgs.physics_update_pb2 as phyud
import protobuf_msgs.network_update_pb2 as netud

class NetworkCoordinator:

    def __init__(self, config_file):

        # event used to indicate whether threads are allowed to run
        self.run_event = multiprocessing.Event()

        # barriers used to sync the protobuf threads at the beginning and end of their loops
        self.protobuf_sync_barrier_top = threading.Barrier(2, timeout=5)
        self.protobuf_sync_barrier_bottom = threading.Barrier(2, timeout=10)

        # global "time"
        self.time_counter = AtomicCounter()

        # global packet ID
        self.packet_id = AtomicCounter()

        # incoming packets will be stored in this dict until their data is sent to the network simulator
        self.incoming_packet_buffer = {}

        # to get TUN threads to wait while the buffer is consumed
        self.incoming_packet_buffer_busy = threading.Lock()

        # packets will be stored in this dict until the network simulator asks for them to be sent to destination
        self.outgoing_packet_buffer = {}

        # stores ids of packets for each interval, and these are cleared as they time out
        self.dispatch_record = {}

        # node & channel information
        self.channel_data = None

        with open(config_file) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

            self.phy_use_uds = self.config['phy_use_uds']
            self.net_use_uds = self.config['net_use_uds']

    def _setup_network(self):
        for i, ip_i in enumerate(self.config['ip_list']):
            # ~ sudo ip tuntap add dev tun0 mode tun
            # ~ sudo ip link set tun0 up
            # ~ sudo ip addr add 192.168.0.1/32 dev tun0
            
            # NOTE: First command needs to read the root password from standard in (requiring the -S flag)
            subprocess.call(["sudo", "-S", "ip", "tuntap", "add", "dev", "tun" + str(i), "mode", "tun"])
            subprocess.call(["sudo", "ip", "link", "set", "tun" + str(i), "up"])
            subprocess.call(["sudo", "ip", "addr", "add", ip_i + "/32", "dev", "tun" + str(i)])

            # ~ sudo ip route add 192.168.0.2/32 dev tun0 src 192.168.0.1 table 1
            # ~ sudo ip rule add table 1 from 192.168.0.1 priority 2
            for j, ip_j in enumerate(self.config['ip_list']):
                if j != i:
                    subprocess.call(["sudo", "ip", "route", "add", ip_j + "/32", "dev", "tun" + str(i),
                                     "src", ip_i, "table", str(i + 1)])
            subprocess.call(["sudo", "ip", "rule", "add", "table", str(i + 1), "from", ip_i, "priority", "2"])

            # ~ #Conditional local
            # ~ sudo ip rule add iif tun0 lookup 101 priority 1
            # ~ sudo ip route add local 192.168.0.1 dev tun0 table 101
            subprocess.call(["sudo", "ip", "rule", "add", "iif", "tun" + str(i), "table", str(i + 101), "priority", "1"])
            subprocess.call(["sudo", "ip", "route", "add", "local", ip_i, "dev", "tun" + str(i), "table", str(i + 101)])

        # ~ sudo ip rule del pref 0 from all lookup local
        # ~ sudo ip rule add pref 10 from all lookup local
        subprocess.call(["sudo", "ip", "rule", "del", "pref", "0", "from", "all", "lookup", "local"])
        subprocess.call(["sudo", "ip", "rule", "add", "pref", "10", "from", "all", "lookup", "local"])
        return self.config['ip_list']

    @staticmethod
    def _remove_network(num_ips):
        for i in range(num_ips):
            # ~ sudo ip tuntap del dev tun0 mode tun
            # ~ sudo ip rule del table 1
            # ~ sudo ip rule del table 101
            subprocess.call(["sudo", "ip", "tuntap", "del", "dev", "tun" + str(i), "mode", "tun"])
            subprocess.call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
            subprocess.call(["sudo", "ip", "rule", "del", "table", str(i + 101)])

        # ~ sudo ip rule add pref 0 from all lookup local
        # ~ sudo ip rule del pref 10 from all lookup local
        subprocess.call(["sudo", "ip", "rule", "add", "pref", "0", "from", "all", "lookup", "local"])
        subprocess.call(["sudo", "ip", "rule", "del", "pref", "10", "from", "all", "lookup", "local"])

    def _read_from_tuns(self, i, tuns):
        while self.run_event.is_set():
            # read from TUN
            try:
                r, __, __ = select.select([tuns[i].fileno(), ], [], [], self.config['responsiveness_timeout'])
                if r:
                    data = os.read(tuns[i].fileno(), 4096)
                else:
                    continue
            except OSError:
                self.run_event.clear()  # to end all threads
                print("TUN " + str(i) + " seems to be gone, can't read from it")
                break

            # identify IPs and save into buffer
            version = data[0]
            version = version >> 4
            if version == 4:
                ip_src = int.from_bytes(data[12:16], byteorder="big")
                ip_dst = int.from_bytes(data[16:20], byteorder="big")
                with self.incoming_packet_buffer_busy:
                    self.incoming_packet_buffer[(self.packet_id.value, ip_src, ip_dst)] = data
                    if self.config['print_debug']: print(f"Read {(self.packet_id.value, ip_src, ip_dst)} from TUN {i}")
                    self.packet_id.increment()

        print("TUN " + str(i) + " exiting")
        tuns[i].close()

    def _run_protobuf_client_phy_coord(self):

        # Connect the socket to the port where the server is listening
        if self.phy_use_uds:
            # Create a UDS socket
            server_address = self.config['phy_uds_server_address']
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        else:
            server_address = (self.config['phy_ip_server_address'], self.config['phy_ip_server_port'])
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

        try:
            if self.try_connecting(sock, server_address, "Error (physics coordinator protobuf client):") == -1:
                self.run_event.clear()
                return
            waiting_for_update = False
            while self.run_event.is_set():
                if not waiting_for_update:
                    self.protobuf_sync_barrier_top.wait()
                    # send time update request
                    time_update = phyud.PhysicsUpdate()
                    time_update.time_val = self.time_counter.value
                    time_update.msg_type = phyud.PhysicsUpdate.BEGIN
                    self.send_one_message(sock, gzip.compress(time_update.SerializeToString()))

                waiting_for_update = True
                # wait for completion
                r, __, __ = select.select([sock, ], [], [], self.config['responsiveness_timeout'])
                if r:
                    # get response
                    data = self.recv_one_message(sock)
                    if not data: continue
                    waiting_for_update = False

                    time_update = phyud.PhysicsUpdate()
                    time_update.ParseFromString(gzip.decompress(data))
                    if time_update.msg_type != phyud.PhysicsUpdate.END:
                        raise ValueError("Network Coordinator got non-END message from Physics Coordinator!")
                    self.channel_data = time_update.channel_data if time_update.channel_data else b""

                    i = self.protobuf_sync_barrier_bottom.wait()
                    if i == 0:  # only one thread will run this
                        self.time_counter.increment()
                else:
                    continue
        except socket.error as msg:
            print("Error (physics coordinator protobuf client): %s" % (msg,))
            self.run_event.clear()  # to end all threads
        except threading.BrokenBarrierError:
            print("Physics coordinator client: Timeout/Abort while waiting for network simulator client")
            self.run_event.clear()  # to end all threads
        finally:
            print('Closing protobuf client socket (physics coordinator )', file=sys.stderr)
            self.protobuf_sync_barrier_bottom.abort()
            sock.close()

    def _run_protobuf_client_net_sim(self, tuns, ip_to_tun_map):

        # Connect the socket to the port where the server is listening
        if self.net_use_uds:
            # Create a UDS socket
            server_address = self.config['netsim_uds_server_address']
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        else:
            server_address = (self.config['netsim_ip_server_address'], self.config['netsim_ip_server_port'])
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        try:
            if self.try_connecting(sock, server_address, "Error (network simulator protobuf client):") == -1:
                self.run_event.clear()
                return

            waiting_for_update = False
            while self.run_event.is_set():
                if not waiting_for_update:
                    # send time update request
                    self.protobuf_sync_barrier_top.wait()
                    current_time = self.time_counter.value
                    time_update = netud.NetworkUpdate()
                    time_update.time_val = current_time
                    time_update.msg_type = netud.NetworkUpdate.BEGIN

                    # tell TUN threads to wait while buffer is emptied
                    with self.incoming_packet_buffer_busy:
                        temp_packet_buffer = self.incoming_packet_buffer
                        self.incoming_packet_buffer = {}
                    if temp_packet_buffer:
                        self.dispatch_record[current_time] = []
                    for key_tuple, pkt_data in temp_packet_buffer.items():
                        time_update.pkt_id.append(key_tuple[0])
                        time_update.src_ip.append(key_tuple[1])
                        time_update.dst_ip.append(key_tuple[2])
                        time_update.pkt_lengths.append(len(pkt_data))

                        self.dispatch_record[current_time].append(key_tuple)

                    self.outgoing_packet_buffer.update(temp_packet_buffer)

                    if self.channel_data:
                        time_update.channel_data = self.channel_data

                    self.send_one_message(sock, gzip.compress(time_update.SerializeToString()))

                waiting_for_update = True
                # wait for update
                r, __, __ = select.select([sock, ], [], [], self.config['responsiveness_timeout'])
                if r:
                    data = self.recv_one_message(sock)
                    if not data: continue
                    waiting_for_update = False

                    time_update = netud.NetworkUpdate()
                    time_update.ParseFromString(gzip.decompress(data))

                    for time_update_tuple in zip(time_update.pkt_id, time_update.src_ip, time_update.dst_ip,
                                                 time_update.ber, time_update.rx_ip):
                        if not self.run_event.is_set():
                            print("Network simulator was asked to exit while writing data")
                            break
                        ber = time_update_tuple[3]
                        key_tuple = time_update_tuple[:3]
                        if key_tuple not in self.outgoing_packet_buffer:
                            # The packet was already dropped, probably due to storage timeout
                            print(f"Packet {key_tuple} to be delivered was already dropped")
                            continue
                        data = self.outgoing_packet_buffer[key_tuple]

                        if key_tuple[2] == int(self.config['broadcast_address']):
                            # Convert UDP broadcast (to 255.255.255.255) to unicast.
                            data = bytearray(data)
                            data[16:20] = time_update_tuple[4].to_bytes(4, byteorder="big")  # set IP
                            data[10:12] = b'\x00\x00'  # clear chksum
                            data[10:12] = self.generate_ipv4_checksum(data[:20]).to_bytes(2, byteorder="big")
                            data[26:28] = b'\x00\x00'  # clear UDP chksum

                        if ber == 0:
                            pass
                        elif 0 < ber < 1:
                            if not isinstance(data, bytearray): data = bytearray(data)
                            data = self._apply_ber(ber, data)
                            if not data: continue
                        else:
                            continue

                        try:
                            os.write(tuns[ip_to_tun_map[time_update_tuple[4]]].fileno(), data)
                            if self.config['print_debug']: print(f"Wrote {key_tuple} to TUN {ip_to_tun_map[time_update_tuple[4]]}")
                        except (OSError, KeyError, ValueError) as msg:
                            #print("Network simulator client: Error while writing to TUN. %s" % (msg,))
                            continue
                    for key_tuple in zip(time_update.clear_pkt_id, time_update.clear_src_ip, time_update.clear_dst_ip):
                        if key_tuple in self.outgoing_packet_buffer:
                            del self.outgoing_packet_buffer[key_tuple]

                    time_to_clear = current_time - self.config['packet_holding_duration']
                    if time_to_clear in self.dispatch_record:
                        for key_tuple in self.dispatch_record[time_to_clear]:
                            if key_tuple in self.outgoing_packet_buffer:
                                del self.outgoing_packet_buffer[key_tuple]
                        del self.dispatch_record[time_to_clear]

                    if time_update.msg_type != netud.NetworkUpdate.END:
                        raise ValueError("Network Coordinator got non-END message from Network Simulator!")

                    i = self.protobuf_sync_barrier_bottom.wait()
                    if i == 0:  # only one thread will run this
                        self.time_counter.increment()

        except socket.error as msg:
            print("Error (network simulator protobuf client): %s" % (msg,))
            self.run_event.clear()  # to end all threads
        except threading.BrokenBarrierError:
            print("Network simulator client: Timeout/Abort while waiting for physics coordinator client")
            self.run_event.clear()  # to end all threads
        finally:
            self.protobuf_sync_barrier_bottom.abort()
            print('Closing protobuf client socket (network simulator)', file=sys.stderr)
            sock.close()

    @staticmethod
    def generate_ipv4_checksum(ip_header):
        if len(ip_header) % 2 == 1:
            ip_header += "\0"
        checksum = sum(array.array("H", ip_header))
        checksum = (checksum >> 16) + (checksum & 0xffff)
        checksum += (checksum >> 16)
        checksum = ~checksum
        # assumes little endian
        return (((checksum >> 8) & 0xff) | checksum << 8) & 0xffff

    def _apply_ber(self, ber, data):
        for byte_number, bit_to_flip in self._get_flip_locations(ber, len(data)):
            if byte_number < 20:
                return None
            data[byte_number] = data[byte_number] ^ 1 << bit_to_flip
        return data

    def _transfer_driver_requests(self, phy_driver_sockets, phy_socket_to_lock, netsim_driver_socket, netsim_lock):
        send_stuff_timer = None
        request_set_lock = threading.Lock()
        request_set = set()

        def send_stuff():
            with request_set_lock:
                driver_requests_string = ("[" + ",".join(request_set) + "]").encode("utf-8")
                request_set.clear()
            try:
                with netsim_lock:
                    self.send_one_message(netsim_driver_socket, gzip.compress(driver_requests_string))
                if self.config['print_debug']: print(f"Wrote driver request to network simulator")
            except socket.error as msg:
                print("Error (Driver request transfer): %s" % (msg,))

        while self.run_event.is_set():
            r, __, __ = select.select(phy_driver_sockets, [], [], self.config['responsiveness_timeout'])
            if r:
                try:
                    for readable_socket in r:
                        with phy_socket_to_lock[readable_socket]:
                            incoming_message_string = self.recv_one_message(readable_socket)
                        if not incoming_message_string: continue
                        if self.config['print_debug']: print(f"Read driver request")
                        incoming_message_string = incoming_message_string.decode("utf-8")
                        with request_set_lock:
                            request_set.add(incoming_message_string)
                except socket.error as msg:
                    print("Error (Driver request transfer): %s" % (msg,))

                if request_set and (send_stuff_timer is None or not send_stuff_timer.is_alive()):
                    send_stuff_timer = threading.Timer(self.config['driver_sync_time'], function=send_stuff)
                    send_stuff_timer.setDaemon(True)
                    send_stuff_timer.start()
            else:
                continue

    def _transfer_driver_responses(self, phy_driver_sockets, phy_socket_locks, netsim_driver_socket, netsim_lock):
        mac_to_index = {mac_i: i for i, mac_i in enumerate(self.config['mac_list'])}
        while self.run_event.is_set():
            r, __, __ = select.select([netsim_driver_socket, ], [], [], self.config['responsiveness_timeout'])
            if r:
                try:
                    with netsim_lock:
                        data_list = self.recv_one_message(netsim_driver_socket)
                    if not data_list: continue
                    if self.config['print_debug']: print(f"Read driver response from network simulator")

                    for data in json.loads(gzip.decompress(data_list)):
                        src_mac = data.get("src_mac", None)
                        if not src_mac: continue
                        socket_index = mac_to_index[src_mac]
                        appropriate_socket = phy_driver_sockets[socket_index]
                        with phy_socket_locks[socket_index]:
                            self.send_one_message(appropriate_socket, json.dumps(data).encode("utf-8"))
                        if self.config['print_debug']: print(f"Wrote driver response to socket {socket_index}")

                except socket.error as msg:
                    print("Error (Driver response transfer): %s" % (msg,))
            else:
                continue

    def _run_driver_process(self, num_interfaces):
        if self.net_use_uds:
            server_address = self.config['net_driver_uds_server_address']
            netsim_driver_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        else:
            server_address = (self.config['netsim_ip_server_address'], self.config['netsim_ip_ranging_port'])
            netsim_driver_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            netsim_driver_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            netsim_driver_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)

        try:
            if self.try_connecting(netsim_driver_socket, server_address, "Error (Network simulator driver client):") == -1:
                return
            netsim_lock = threading.Lock()

            phy_driver_sockets = []
            phy_socket_locks = []
            phy_socket_to_lock = {}
            for i in range(num_interfaces):
                phy_socket_server_address = self.config['phy_driver_uds_server_address'] + str(i)
                phy_driver_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                if self.try_connecting(phy_driver_socket, phy_socket_server_address, f"Error (Physics coordinator driver client {i}):") == -1:
                    continue
                phy_driver_sockets.append(phy_driver_socket)
                phy_socket_lock = threading.Lock()
                phy_socket_locks.append(phy_socket_lock)
                phy_socket_to_lock[phy_driver_socket] = phy_socket_lock
        except socket.error as msg:
            if str(msg).strip():
                print("Error (driver process): %s" % (msg,))
            return

        read_thread = threading.Thread(target=self._transfer_driver_requests, args=(
            phy_driver_sockets, phy_socket_to_lock, netsim_driver_socket, netsim_lock))
        write_thread = threading.Thread(target=self._transfer_driver_responses, args=(
            phy_driver_sockets, phy_socket_locks, netsim_driver_socket, netsim_lock))

        read_thread.start()
        write_thread.start()

        read_thread.join()
        write_thread.join()

    @staticmethod
    def try_connecting(sock, address, tag):
        i = 0
        while i < 100:
            try:
                sock.connect(address)
                break
            except socket.error:
                i += 1
                time.sleep(2)
        else:
            print(tag + " Could not connect")
            return -1
        return 0


    @staticmethod
    def _get_flip_locations(ber, packet_length):
        current_byte = 0
        while current_byte < packet_length:
            space_to_next_error_bit = np.random.geometric(p=ber)
            space_to_next_error_byte, bit_to_flip = divmod(space_to_next_error_bit, 8)
            current_byte = current_byte + space_to_next_error_byte
            if current_byte < packet_length:
                yield current_byte, bit_to_flip

    @staticmethod
    def send_one_message(sock, data):
        length = len(data)
        sock.sendall(struct.pack('!I', length) + data)

    @classmethod
    def recv_one_message(cls, sock):
        lengthbuf = cls.recvall(sock, 4)
        if not lengthbuf: return None
        length, = struct.unpack('!I', lengthbuf)
        return cls.recvall(sock, length)

    @staticmethod
    def recvall(sock, count):
        buf = bytearray()
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def run_network_coordinator(self):

        ip_list = self._setup_network()
        num_ips = len(ip_list)

        # constants for opening TUNs
        TUNSETIFF = 0x400454ca
        TUNSETOWNER = TUNSETIFF + 2
        IFF_TUN = 0x0001
        # IFF_TAP = 0x0002
        IFF_NO_PI = 0x1000

        tuns = []
        tun_threads = []
        phy_protobuf_thread = None
        netsim_protobuf_thread = None
        driver_process = None

        self.run_event.set()

        try:
            # read IPs
            ip_to_tun_map = {int(ipaddress.IPv4Address(ip_i)): i for i, ip_i in enumerate(ip_list)}

            # open TUNs
            for i in range(num_ips):
                tuns.append(open('/dev/net/tun', 'r+b', buffering=0))
                ifri = struct.pack('16sH', b'tun' + str(i).encode('ascii'), IFF_TUN | IFF_NO_PI)
                fcntl.ioctl(tuns[i], TUNSETIFF, ifri)
                fcntl.ioctl(tuns[i], TUNSETOWNER, 1000)

            # start TUN threads
            for i in range(num_ips):
                tun_threads.append(threading.Thread(target=self._read_from_tuns, args=(i, tuns,)))
                tun_threads[i].start()

            # start protobuf threads
            phy_protobuf_thread = threading.Thread(target=self._run_protobuf_client_phy_coord, args=())
            netsim_protobuf_thread = threading.Thread(target=self._run_protobuf_client_net_sim,
                                                      args=(tuns, ip_to_tun_map))
            phy_protobuf_thread.start()
            netsim_protobuf_thread.start()

            # start "driver" process
            if self.config['do_driver_transfer']:
                driver_process = multiprocessing.Process(target=self._run_driver_process, args=(num_ips,))
                driver_process.start()

            # block main thread
            print("Enter anything to stop\n")
            while self.run_event.is_set():
                r, __, __ = select.select([sys.stdin, ], [], [], self.config['responsiveness_timeout'])
                if r:
                    raise KeyboardInterrupt
                else:
                    continue
            raise KeyboardInterrupt

        except KeyboardInterrupt:
            print("Attempting to close threads")
            self.run_event.clear()
            if tun_threads:
                for i in tun_threads:
                    i.join()
            if phy_protobuf_thread:
                phy_protobuf_thread.join()
            if netsim_protobuf_thread:
                netsim_protobuf_thread.join()
            if driver_process:
                driver_process.join()
            print("Threads successfully closed")
        finally:
            self._remove_network(num_ips)


class AtomicCounter:
    def __init__(self, initial=0):
        """Initialize a new atomic counter to given initial value (default 0)."""
        self.value = initial
        self._lock = threading.Lock()

    def increment(self, num=1):
        """Atomically increment the counter by num (default 1) and return the
        new value.
        """
        with self._lock:
            self.value += num
        return self.value


def main(args):
    # Capture config
    config_file = None
    for arg in args:
        if ".yaml" in arg:
            config_file = arg
            break
        
    if config_file is None:
        print("usage: network_coordinator.py <config_file>")
    else:
        network_coordinator = NetworkCoordinator(config_file)
        network_coordinator.run_network_coordinator()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
