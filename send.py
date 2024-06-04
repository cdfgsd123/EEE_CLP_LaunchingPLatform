#Sending serial message through Udp in python

import socket
import time

def send_udp_message(hex_array, ip, port):
    byte_array = bytes(hex_array)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(byte_array, (ip, port))
    sock.close()

# Example usage
while(1):
    if __name__ == "__main__":
        hex_array = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]
        ip = "192.168.10.30"  # IP address in Engnieering Club, ip can be find by findIP.ino
        port = 8088
        
        send_udp_message(hex_array, ip, port)
    time.sleep(0.1)