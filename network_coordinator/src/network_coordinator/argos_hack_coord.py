from __future__ import annotations
import enum
from io import FileIO
import os
import sys
import gzip
import signal
import yaml
import fcntl
import struct
import select
import ipaddress
import rospy
import socket
from typing import List, Tuple
from multiprocessing import Event
from threading import Barrier, BrokenBarrierError, Lock, Thread
from network_coordinator.coordinator import AtomicCounter
from network_coordinator.constants import IFF_TUN, IFF_NO_PI, TUNSETOWNER, TUNSETIFF, TUNL_INT_TO_IDX
from network_coordinator.utils import setup_network, remove_network, send_one_message, recv_one_message, try_connecting, get_flip_locations # noqa: E501
import protobuf_msgs.physics_update_pb2 as phyud
import protobuf_msgs.network_update_pb2 as netud
from argos_bridge.msg import losList
from argos_bridge.msg import los


class ArgosNetCoordinator:
    run_event: Event
    protobuf_sync_barrier_top: Barrier
    protobuf_sync_barrier_bottom: Barrier
    time_counter: AtomicCounter
    packet_id: AtomicCounter
    incoming_packet_buffer: dict
    outgoing_packet_buffer: dict
    incoming_packet_buffer_busy: Lock
    config: dict
    dispatch_record: dict
    phy_use_uds: bool
    net_use_uds: bool
    argos_model_to_ip: dict[str, str]
    argos_model_to_idx: dict[str, int]
    ip_to_idx: dict[str, int]
    argos_models: List[str]
    los: List[List[bool]]
    los_lock: Lock

    def __init__(self: ArgosNetCoordinator, config_file: str) -> None:
        self.run_event = Event()
        self.protobuf_sync_barrier_top = Barrier(2, timeout=5)
        self.protobuf_sync_barrier_bottom = Barrier(2, timeout=10)
        self.time_counter = AtomicCounter()
        self.packet_id = AtomicCounter()
        self.incoming_packet_buffer = {}
        self.argos_model_to_ip = {}
        self.incoming_packet_buffer_busy = Lock()
        self.outgoing_packet_buffer = {}
        self.dispatch_record = {}
        self.channel_data = None
        self.argos_models = []
        self.ip_to_idx = {}
        self.argos_model_to_idx = {}
        self.los = []
        self.los_lock = Lock()
        with open(config_file) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
            self.phy_use_uds = self.config['phy_use_uds']
            self.net_use_uds = self.config['net_use_uds']

    # Captures all network packets from tunnel interfaces
    def _read_from_tuns(self: ArgosNetCoordinator, i: int, tuns: List[FileIO]) -> None:
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
                    if self.config['print_debug']:
                        print(f"Read {(self.packet_id.value, ip_src, ip_dst)} from TUN {i}")
                    self.packet_id.increment()

        print("TUN " + str(i) + " exiting")
        tuns[i].close()

    def _run_protobuf_client_phy_coord(self: ArgosNetCoordinator) -> None:
        # Connect the socket to the port where the server is listening
        if self.phy_use_uds:
            # Create a UDS socket
            server_address: str = self.config['phy_uds_server_address']
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        else:
            server_address: Tuple[str, int] = (self.config['phy_ip_server_address'], self.config['phy_ip_server_port'])
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        try:
            if try_connecting(sock, server_address, "Error (physics coordinator protobuf client):") == -1:
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
                    send_one_message(sock, gzip.compress(time_update.SerializeToString()))

                waiting_for_update = True
                # wait for completion
                r, __, __ = select.select([sock, ], [], [], self.config['responsiveness_timeout'])
                if r:
                    # get response
                    data = recv_one_message(sock)
                    if not data:
                        continue
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
        except BrokenBarrierError:
            print("Physics coordinator client: Timeout/Abort while waiting for network simulator client")
            self.run_event.clear()  # to end all threads
        finally:
            print('Closing protobuf client socket (physics coordinator )', file=sys.stderr)
            self.protobuf_sync_barrier_bottom.abort()
            sock.close()

    def _run_protobuf_client_net_sim(self: ArgosNetCoordinator, tuns: List[FileIO], ip_to_tun_map: dict[int, int]) -> None: # noqa: E501
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
            if try_connecting(sock, server_address, "Error (network simulator protobuf client):") == -1:
                self.run_event.clear()
                return

            waiting_for_update = False
            current_time = 0
            while self.run_event.is_set():
                if not waiting_for_update:
                    # send a Network Update message | with captured packets
                    current_time = self._send_net_sim_update(sock)

                # recieve Network Update message from Net-Sim | release packets accordingly
                success = self._handle_net_sim_response(sock, tuns, ip_to_tun_map, current_time)
                if success:
                    waiting_for_update = False

                # Advance atomic clock
                if self.protobuf_sync_barrier_bottom.wait() == 0:
                    # only one thread will run this
                    self.time_counter.increment()

        except socket.error as e:
            print("Error (network simulator protobuf client): ", e)
            self.run_event.clear()
        except BrokenBarrierError:
            print("Network simulator client: Timeout/Abort while waiting for physics coordinator client")
            self.run_event.clear()  # to end all threads
        finally:
            self.protobuf_sync_barrier_bottom.abort()
            print('Closing protobuf client socket (network simulator)', file=sys.stderr)
            sock.close()

    def _send_net_sim_update(self: ArgosNetCoordinator, sock: socket.socket) -> int:
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

        send_one_message(sock, gzip.compress(time_update.SerializeToString()))
        return current_time

    def _handle_net_sim_response(
            self: ArgosNetCoordinator,
            sock: socket.socket, tuns: List[FileIO],
            ip_to_tun_map: dict, current_time: int) -> bool:
        # wait for update
        r, __, __ = select.select([sock, ], [], [], self.config['responsiveness_timeout'])
        if not r:
            print("WARNING: Net Sim didn't respond in time!")
            return False
        # Obtain response from Net-Sim
        data = recv_one_message(sock)
        if not data:
            print("WARNING: Net Sim responded with no data!")
            return False
        # Format data into NetworkUpdate object
        time_update = netud.NetworkUpdate()
        time_update.ParseFromString(gzip.decompress(data))

        # Get line of site data
        # NOTE: This is the hacky part
        # We should be doing this in the net-sim
        self.los_lock.acquire()
        latest_los = self.los
        self.los_lock.release()

        # Iterate over network packets
        packets: Tuple = zip(
            time_update.pkt_id,
            time_update.src_ip,
            time_update.dst_ip,
            time_update.ber,
            time_update.rx_ip)
        for time_update_tuple in packets:
            if not self.run_event.is_set():
                print("Net Sim was asked to exit while writing data")
                # NOTE: Should we raise an exception???
                # raise BrokenBarrierError("Net Sim is exiting")
                break
            # Pull apart packet info
            ber = time_update_tuple[3]
            key_tuple = time_update_tuple[:3] # Used as an identifier for packets
            if key_tuple not in self.outgoing_packet_buffer:
                print("Packet {0} was already dropped", key_tuple)
                continue

            # Check if robots are in Line of Site
            src_ip = TUNL_INT_TO_IDX[str(time_update_tuple[1])]
            dst_ip = TUNL_INT_TO_IDX[str(time_update_tuple[2])]

            print(latest_los)
            print(src_ip)
            print(dst_ip)

            if not latest_los[src_ip][dst_ip]:
                print("Packet {0} dropped, not in Line-Of-Site between {1} - {2}", key_tuple, src_ip, dst_ip)
                continue
            else:
                # Fetch true network packet
                data: bytes = self.outgoing_packet_buffer[key_tuple]

                if key_tuple[2] == int(self.config['broadcast_address']):
                    # Convert UDP broadcast (to 255.255.255.255) to unicast.
                    data = bytearray(data)
                    data[16:20] = time_update_tuple[4].to_bytes(4, byteorder="big")  # set IP
                    data[10:12] = b'\x00\x00'  # clear chksum
                    data[10:12] = self.generate_ipv4_checksum(data[:20]).to_bytes(2, byteorder="big")
                    data[26:28] = b'\x00\x00'  # clear UDP checksum

                # Apply psudo-random errors
                data = self._apply_bit_errors_to_packet(ber, data)

                # Forward packet to destination network device
                try:
                    dest_ip = time_update_tuple[4]
                    dest_tun = ip_to_tun_map[dest_ip]
                    tun_file = tuns[dest_tun]
                    os.write(tun_file.fileno(), data)
                    if self.config['print_debug']:
                        print(f"Wrote {key_tuple} to TUN {dest_tun}")
                except (OSError, KeyError, ValueError) as msg:
                    if self.config['print_debug']:
                        print("Network simulator client: Error while writing to TUN - ", msg)
                    continue

        # Remove packets with "clear" attributes
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

        return True

    def _apply_bit_errors_to_packet(self: ArgosNetCoordinator, bit_error_rate: float, net_packet: bytes) -> bytes:
        # NOTE: All packets will remain unaltered. This may be changed in the future!
        return net_packet

    @staticmethod
    def gen_empty_los_array(num_models: int) -> List[List[bool]]:
        los_map: List[List[bool]] = []
        for _ in range(num_models):
            los_map.append([False] * num_models)
        return los_map

    # Capture line of site data for every agent in simulation
    def _obtain_line_of_site_data(self: ArgosNetCoordinator) -> None:
        while self.run_event.is_set():
            los_map = ArgosNetCoordinator.gen_empty_los_array(len(self.argos_models))

            for model in self.argos_models:
                # Event used to wait for a single call to the callback funciton
                self.los_called = Event()

                # Aggregate los data when data is published to 'lineOfSight' topic
                def callback(data):
                    robots: List[los] = data.robots
                    from_idx: int = self.argos_model_to_idx[model]

                    los_map[from_idx][from_idx] = True
                    for robot in robots:
                        robot_name: str = self._clean_robot_name(robot.robotName)
                        if len(robot_name) == 0:
                            continue
                        to_idx: int = self.argos_model_to_idx[robot_name]
                        # Populate line of site relationship as "reachable"
                        los_map[from_idx][to_idx] = True
                        los_map[to_idx][from_idx] = True

                    self.los_called.set()
                # Setup & teardown line of site subscriber
                sub = rospy.Subscriber("/{0}/lineOfSight".format(model), losList, callback)
                # Blocking to allow for a single call to callback()
                self.los_called.wait(timeout=0.5)
                sub.unregister()
            # Populate los
            self.los_lock.acquire()
            self.los = los_map
            self.los_lock.release()

        print("Terminating Line of Site subscriptions")

    def _clean_robot_name(self: ArgosNetCoordinator, name: str) -> str:
        for model in self.argos_models:
            if model in name:
                return model
        return ""

    def run_network_coordinator(self: ArgosNetCoordinator) -> None:
        ip_list = setup_network(self.config)
        num_ips = len(ip_list)
        tuns: List[FileIO] = []
        tun_threads: List[Thread] = []
        phy_protobuf_thread: Thread = None
        netsim_protobuf_thread: Thread = None
        argos_los_thread: Thread = None

        self.argos_models = self.config['argos_models']
        self.argos_model_to_ip = {}
        # Create a mapping from IPs to indexes
        # and model names to indexes
        for idx, ip in enumerate(ip_list):
            model: str = self.argos_models[idx]
            self.argos_model_to_ip[model] = ip
            self.ip_to_idx[ip] = idx
            self.argos_model_to_idx[model] = idx

        self.los_lock.acquire()
        self.los = ArgosNetCoordinator.gen_empty_los_array(len(self.argos_models))
        self.los_lock.release()

        # Set theading event flag
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
                tun_threads.append(Thread(target=self._read_from_tuns, args=(i, tuns,)))
                tun_threads[i].start()
            # start protobuf threads
            phy_protobuf_thread = Thread(target=self._run_protobuf_client_phy_coord, args=())
            netsim_protobuf_thread = Thread(target=self._run_protobuf_client_net_sim, args=(tuns, ip_to_tun_map))
            phy_protobuf_thread.start()
            netsim_protobuf_thread.start()
            # start (ARGoS specific) line-of-site subscriber thread 
            argos_los_thread = Thread(target=self._obtain_line_of_site_data)
            argos_los_thread.start()

            # block main thread
            print("Enter anything to stop\n")
            while self.run_event.is_set():
                r, __, __ = select.select([sys.stdin, ], [], [], self.config['responsiveness_timeout'])
                if r:
                    raise KeyboardInterrupt

        except KeyboardInterrupt:
            print("Keyboard interrupt captured!")

        except Exception as e:
            print("Unexpected Exeption within Net Coord: ", e)

        # close all threads
        print("Attempting to close threads")
        self.run_event.clear()
        if tun_threads:
            for i in tun_threads:
                i.join()
        if phy_protobuf_thread:
            phy_protobuf_thread.join()
        if netsim_protobuf_thread:
            netsim_protobuf_thread.join()
        print("wating on los threads")
        if argos_los_thread:
            argos_los_thread.join()
        print("Threads successfully closed")
        remove_network(num_ips)


def main(args: List[str]) -> any:
    if len(args) != 2:
        print("usage: {0} <config_file>".format(args[0]))
    else:
        net_coord = ArgosNetCoordinator(args[1])
        net_coord.run_network_coordinator()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
