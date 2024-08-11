import socket

def start_server():
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    
    server_address = ('localhost', 6547)
    print(f"Starting up on {server_address[0]} port {server_address[1]}")
    server_socket.bind(server_address)

    
    server_socket.listen(1)

    while True:  
        
        print("Waiting for a connection")
        connection, client_address = server_socket.accept()

        try:
            print(f"Connection from {client_address}")

            
            while True:
                data = connection.recv(16)
                if data:
                    print(f"Received: {data.decode()}")
                else:
                    print("No more data from", client_address)
                    break

        finally:
            
            connection.close()

if __name__ == "__main__":
    start_server()