from subprocess import call
from typing import List, Tuple, Generator, Union
import numpy as np
from numpy.typing._array_like import _ArrayLikeFloat_co
import time
import array
import socket
import struct


def setup_network(config: dict) -> List[str]:
    for i, ip_i in enumerate(config['ip_list']):
        # ~ sudo ip tuntap add dev tun0 mode tun
        # ~ sudo ip link set tun0 up
        # ~ sudo ip addr add 192.168.0.1/32 dev tun0

        # NOTE: First command needs to read the root password from standard in (requiring the -S flag)
        call(["sudo", "-S", "ip", "tuntap", "add", "dev", "tun" + str(i), "mode", "tun"])
        call(["sudo", "ip", "link", "set", "tun" + str(i), "up"])
        call(["sudo", "ip", "addr", "add", ip_i + "/32", "dev", "tun" + str(i)])

        # ~ sudo ip route add 192.168.0.2/32 dev tun0 src 192.168.0.1 table 1
        # ~ sudo ip rule add table 1 from 192.168.0.1 priority 2
        for j, ip_j in enumerate(config['ip_list']):
            if j != i:
                call(["sudo", "ip", "route", "add", ip_j + "/32", "dev", "tun" + str(i),
                                    "src", ip_i, "table", str(i + 1)])
        call(["sudo", "ip", "rule", "add", "table", str(i + 1), "from", ip_i, "priority", "2"])

        # ~ #Conditional local
        # ~ sudo ip rule add iif tun0 lookup 101 priority 1
        # ~ sudo ip route add local 192.168.0.1 dev tun0 table 101
        call(["sudo", "ip", "rule", "add", "iif", "tun" + str(i), "table", str(i + 101), "priority", "1"])
        call(["sudo", "ip", "route", "add", "local", ip_i, "dev", "tun" + str(i), "table", str(i + 101)])

    # ~ sudo ip rule del pref 0 from all lookup local
    # ~ sudo ip rule add pref 10 from all lookup local
    call(["sudo", "ip", "rule", "del", "pref", "0", "from", "all", "lookup", "local"])
    call(["sudo", "ip", "rule", "add", "pref", "10", "from", "all", "lookup", "local"])
    return config['ip_list']


def remove_network(num_ips: int) -> None:
    for i in range(num_ips):
        # ~ sudo ip tuntap del dev tun0 mode tun
        # ~ sudo ip rule del table 1
        # ~ sudo ip rule del table 101
        call(["sudo", "-S", "ip", "tuntap", "del", "dev", "tun" + str(i), "mode", "tun"])
        call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
        call(["sudo", "ip", "rule", "del", "table", str(i + 101)])

    # ~ sudo ip rule add pref 0 from all lookup local
    # ~ sudo ip rule del pref 10 from all lookup local
    call(["sudo", "ip", "rule", "add", "pref", "0", "from", "all", "lookup", "local"])
    call(["sudo", "ip", "rule", "del", "pref", "10", "from", "all", "lookup", "local"])


def generate_ipv4_checksum(ip_header: bytearray) -> int:
    if len(ip_header) % 2 == 1:
        ip_header += "\0"
    checksum = sum(array.array("H", ip_header))
    checksum = (checksum >> 16) + (checksum & 0xffff)
    checksum += (checksum >> 16)
    checksum = ~checksum
    # assumes little endian
    return (((checksum >> 8) & 0xff) | checksum << 8) & 0xffff


def recv_one_message(sock: socket.socket) -> bytearray:
    lengthbuf = recvall(sock, 4)
    if not lengthbuf:
        return None
    length, = struct.unpack('!I', lengthbuf)
    return recvall(sock, length)


def recvall(sock: socket.socket, count: int) -> bytearray:
    buf = bytearray()
    while count:
        newbuf = sock.recv(count)
        if not newbuf:
            return None
        buf += newbuf
        count -= len(newbuf)
    return buf


def send_one_message(sock: socket.socket, data: bytes) -> None:
    length = len(data)
    sock.sendall(struct.pack('!I', length) + data)


def try_connecting(sock: socket.socket, address: Union[str, Tuple[str, int]], msg: str) -> int:
    i = 0
    while i < 100:
        try:
            sock.connect(address)
            break
        except socket.error:
            i += 1
            time.sleep(2)
    else:
        print(msg + " Could not connect")
        return -1
    return 0


def get_flip_locations(ber: _ArrayLikeFloat_co, packet_length: int) -> Generator[Tuple[int, int], None, None]:
    current_byte: int = 0
    while current_byte < packet_length:
        space_to_next_error_bit = np.random.geometric(p=ber)
        space_to_next_error_byte, bit_to_flip = divmod(space_to_next_error_bit, 8)
        current_byte += space_to_next_error_byte
        if current_byte < packet_length:
            yield current_byte, bit_to_flip
