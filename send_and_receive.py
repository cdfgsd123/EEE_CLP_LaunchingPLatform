#Sending serial message through Udp in python

import socket
import time

def send_and_receive(ip, port, hex_array):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message = bytes(hex_array)

    try:
        sock.sendto(message, (ip, port))
        response, server = sock.recvfrom(26)
        response_array = [f"{byte:02X}" for byte in response]
        print(response_array[25]) #print gyroscope

    except socket.error as e:
        print("Error")

    finally:
        sock.close()

hex_array = [0x60, 0x0B, 0xFF, 0xFF, 0xFF, 0xFF]

#IP address in Engnieering Club, ip can be find by findIP.ino
ip = "192.168.10.30"
port = 8088

while(1):
    send_and_receive(ip, port, hex_array)
    time.sleep(0.1)
