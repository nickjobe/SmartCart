import socket

SERVER_IP = '10.242.254.39'
PORT = 5000

# Create a TCP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    client_socket.connect((SERVER_IP, PORT))
    message = "Hello from Client Pi!"
    client_socket.send(message.encode('utf-8'))
    
    # Receive acknowledgment
    response = client_socket.recv(1024).decode('utf-8')
    print(f"Server response: {response}")

except Exception as e:
    print(f"Error: {e}")

finally:
    client_socket.close()