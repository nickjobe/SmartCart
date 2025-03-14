import socket

# Define server IP and port
HOST = '0.0.0.0'  # Listens on all network interfaces
PORT = 5000      # Choose any available port

# Create a TCP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(5)

print(f"Server listening on port {PORT}...")

while True:
    client_socket, client_address = server_socket.accept()
    print(f"Connection from {client_address} established.")
    
    # Receive data from the client
    data = client_socket.recv(1024).decode('utf-8')
    if not data:
        break
    print(f"Received: {data}")

    client_socket.send("Message received".encode('utf-8'))  # Acknowledge message
    client_socket.close()