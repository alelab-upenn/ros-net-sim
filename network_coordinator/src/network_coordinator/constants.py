# constants for opening TUNs
TUNSETIFF = 0x400454ca
TUNSETOWNER = TUNSETIFF + 2
IFF_TUN = 0x0001
# IFF_TAP = 0x0002
IFF_NO_PI = 0x1000

# Tunnel IP addresses, within a packet, are represented as integer file numbers
TUNL_INT_TO_IDX = {
    "3232235521": 0,
    "3232235522": 1,
    "3232235523": 2,
    "3232235524": 3
}
