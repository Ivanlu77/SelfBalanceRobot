import socket
import threading

def process_command(command, data):
    if command == 'echo':
        return data
    elif command == 'ping':
        return "success"
    else:
        return f"Unknown command: {command}"

def handle_client(client_socket, addr):
    print(f"Connected to {addr}")
    try:
        while True:
            # Receive message from client
            message = client_socket.recv(1024).decode('utf-8')
            if not message:
                print(f"Client {addr} disconnected")
                break
                
            # Parse command and data
            if '::' in message:
                command, data = message.split('::', 1)
            else:
                command, data = message, ''
                
            response = process_command(command, data)
                
            # Send response
            client_socket.send(response.encode('utf-8'))
            
    except Exception as e:
        print(f"Error handling client {addr}: {e}")
    finally:
        client_socket.close()
        print(f"Connection to {addr} closed")

def get_local_ip():
    try:
        # Create a socket to get the local IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '0.0.0.0'

def start_server():
    # Create server socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    server.bind(('0.0.0.0', 5000))
    server.listen(5)
    
    local_ip = get_local_ip()
    
    print(f"Server started on IP: {local_ip}, Port: 5000")
    print("Waiting for connections...")
    
    try:
        while True:
            # Accept client connection
            client_socket, addr = server.accept()
            
            # Create new thread to handle client
            client_thread = threading.Thread(
                target=handle_client,
                args=(client_socket, addr)
            )
            client_thread.start()
            
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        server.close()

if __name__ == "__main__":
    start_server() 