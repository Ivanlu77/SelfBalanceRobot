import socket

def send_message(server_ip, message):
    # Skip empty messages
    if not message.strip():
        print("Error: Empty messages are not allowed")
        return ""
        
    # Create client socket
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    response = ""
    
    try:
        # Connect to server
        client.connect((server_ip, 5000))
        
        # Send message
        client.send(message.encode('utf-8'))
        
        # Receive response
        response = client.recv(1024).decode('utf-8')
        print(f"Response: {response}")
        
    except Exception as e:
        print(f"Error: {e}")

    finally:
        client.close()
        return response

def main():
    # Get server IP address
    server_ip = input("Enter Raspberry Pi IP address: ")
    f = True
    while f:
        if send_message(server_ip, "ping") == "success":
            f = False
        else:
            server_ip = input("Enter Raspberry Pi IP address: ")

    print("Enter messages in format 'command::data' (e.g., 'echo::hello world')")
    print("Type 'exit()' to exit")
    
    while True:
        message = input("> ")
        if message.lower() == 'exit()':
            break
            
        send_message(server_ip, message)

if __name__ == "__main__":
    main() 