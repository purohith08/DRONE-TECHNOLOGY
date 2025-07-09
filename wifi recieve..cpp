import socket  # Import socket module

# Set your PC's IP address and the UDP port
UDP_IP = "192.168.1.100"  # Replace with your PC's IP
UDP_PORT = 14550  # Must match the ESP32 code

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on {UDP_IP}:{UDP_PORT} for incoming UDP packets...")

while True:
    data, addr = sock.recvfrom(1024)  # Buffer size
    print(f"Received from {addr}: {data.decode()}")