import socket
import sqlite3
import threading
from CarManagement import CarManagement
from ParkingGroupManagement import ParkingGroupManagement

conn = sqlite3.connect('parking.db', check_same_thread=False)

cars = CarManagement(conn, False, True)
groups = ParkingGroupManagement(conn, False, True)

class CoordinatorServer:
    def __init__(self):
        self.server = None
        self.response = ""

    def handle_client(self, client_socket):
        while True:
            # Receive a command from the client (max 1024 bytes)
            command = client_socket.recv(1024).decode('utf-8').strip()

            if not command:
                break
            
            # Split command to determine the action
            command, *args = command.split(" ")

            # From CarManagement
            if command == "register_car":
                # Example: register_car number name password (e.g., "register_car car xyz car")
                number = args[0]
                name = args[1]
                password = args[2]
                self.response = str(cars.register_car(number, name, password))

            elif command == "authenticate_car":
                # Example: authenticate_car number password (e.g., "authenticate_car car car")
                number = args[0]
                password = args[1]
                self.response = str(cars.authenticate_car(number, password))

            elif command == "set_car_group":
                # Example: set_car_group number group_name (e.g., "set_car_group car A")
                number = args[0]
                group_name = args[1]
                self.response = str(cars.set_car_group(number, group_name))

            elif command == "reset_car_group":
                # Example: reset_car_group number (e.g., "reset_car_group car")
                number = args[0]
                self.response = str(cars.reset_car_group(number))

            # From ParkingGroupManagement
            elif command == "add_group":
                # Example: add_group group_name capacity pos_x pos_y phi (e.g., "add_group A 4 0 0 0")
                group_name = args[0]
                capacity = args[1]
                pos_x = float(args[2])
                pos_y = float(args[3])
                phi = float(args[4])
                self.response = str(groups.add_group(group_name, capacity, pos_x, pos_y, phi))

            elif command == "remove_group":
                # Example: remove_group group_name (e.g., "remove_group A")
                group_name = args[0]
                self.response = str(groups.remove_group(group_name))

            elif command == "get_group_position":
                # Example: get_group_position group_name (e.g., "get_group_position A")
                group_name = args[0]
                ret, res = groups.get_group_position(group_name)
                self.response = str(res)         
            
            elif command == "get_group_statistics":
                # Example: get_group_statistics (e.g., "get_group_statistics")
                ret, res = groups.get_group_statistics()     
                self.response = str(res)  

            # Reservation and guidance functions
            elif command == "reserve_parking":
                # Example: reserve_parking number (e.g., "reserve_parking car")
                number = args[0]

                # Get statistics
                ret, res = groups.get_group_statistics()
                if ret:
                    # Find least reserved group
                    min_c = 100000
                    min_n = "False"

                    for room in res:
                        if (room[2] < min_c):
                            min_c = room[2]
                            min_n = room[0]
                    
                    # Send that as self.response
                    res = cars.set_car_group(number, min_n)
                    if res: self.response = min_n
                    else: self.response = "False"
            
            elif command == "relinquish_parking":
                # Example: relinquish_parking number (e.g., "relinquish_parking car")
                number = args[0]
                self.response = str(cars.reset_car_group(number))

            else:
                self.response = "Unknown command"

            print(f"Received command: {command} : {self.response}")

            # Send the self.response back to the client
            client_socket.send(self.response.encode('utf-8'))

        client_socket.close()


    def start_server(self, host, port):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((host, port))
        self.server.listen(5)
        print(f"Server listening on {host}:{port}")

        while True:
            # Accept new client connection
            client_socket, addr = self.server.accept()
            client_handler = threading.Thread(target=self.handle_client, args=(client_socket,))
            client_handler.start()

def main():
    # Start the server on localhost and port 5555
    server = CoordinatorServer()
    try:
        server.start_server("127.0.0.1", 5555)

    except KeyboardInterrupt:
        print("\nStopping server")
        if server.server:
            server.server.close()
            conn.close()

    except Exception as e:
        print(f"Error: {e}")
        if server.server:
            server.server.close()
            conn.close()

if __name__ == "__main__":
    main()