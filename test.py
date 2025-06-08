import socket
import time

# Set up TCP server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 5000))  # Listen on all interfaces
server_socket.listen(1)

print("Waiting for ESP8266 connection...")

try:
    while True:
        print("Listening for new connection...")
        client_socket, addr = server_socket.accept()
        print(f"Connected by {addr}")

        try:
            while True:
                # Send data to ESP8266
                msg = input("Send to ESP8266: ")
                client_socket.sendall((msg + '\r\n').encode())
                time.sleep(1)

                # Optionally receive response
                '''
                data = client_socket.recv(1024)
                if not data:
                    print("ESP8266 disconnected.")
                    break  # exit inner loop to relisten
                print("ESP8266 says:", data.decode())
                '''

        except (ConnectionResetError, BrokenPipeError):
            print("Connection lost. Relistening...")
        finally:
            client_socket.close()

except KeyboardInterrupt:
    print("Server shutting down.")
finally:
    server_socket.close()
