import rclpy
from rclpy.node import Node

import socket
import sqlite3
import threading
from pas.CarManagement import CarManagement
from pas.ParkingGroupManagement import ParkingGroupManagement

host = "127.0.0.1"
port = 5555

class CoordinatorServer(Node):
    def __init__(self):
        super().__init__('coordinator_server')

        self.server = None
        self.response = ""

        self.conn = sqlite3.connect('parking.db', check_same_thread=False)

        self.cars = CarManagement(self.conn, False, True)
        self.groups = ParkingGroupManagement(self.conn, False, True)

        # Start server in a thread
        self.server_thread = threading.Thread(target=self.start_server, args=(host, port), daemon=True)
        self.server_thread.start()

    def handle_client(self, client_socket):
        while True:
            try:
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
                    self.response = str(self.cars.register_car(number, name, password))

                elif command == "authenticate_car":
                    # Example: authenticate_car number password (e.g., "authenticate_car car car")
                    number = args[0]
                    password = args[1]
                    self.response = str(self.cars.authenticate_car(number, password))

                elif command == "set_car_group":
                    # Example: set_car_group number group_name (e.g., "set_car_group car A")
                    number = args[0]
                    group_name = args[1]
                    self.response = str(self.cars.set_car_group(number, group_name))

                elif command == "reset_car_group":
                    # Example: reset_car_group number (e.g., "reset_car_group car")
                    number = args[0]
                    self.response = str(self.cars.reset_car_group(number))

                # From ParkingGroupManagement
                elif command == "add_group":
                    # Example: add_group group_name capacity pos_x pos_y phi (e.g., "add_group A 4 0 0 0")
                    group_name = args[0]
                    capacity = args[1]
                    pos_x = float(args[2])
                    pos_y = float(args[3])
                    phi = float(args[4])
                    self.response = str(self.groups.add_group(group_name, capacity, pos_x, pos_y, phi))

                elif command == "remove_group":
                    # Example: remove_group group_name (e.g., "remove_group A")
                    group_name = args[0]
                    self.response = str(self.groups.remove_group(group_name))

                elif command == "get_group_position":
                    # Example: get_group_position group_name (e.g., "get_group_position A")
                    group_name = args[0]
                    ret, res = self.groups.get_group_position(group_name)
                    if ret: self.response = f"{res[0]} {res[1]} {res[2]}"
                    else: self.response = "[]"        
                
                elif command == "get_group_statistics":
                    # Example: get_group_statistics (e.g., "get_group_statistics")
                    ret, res = self.groups.get_group_statistics()     
                    self.response = str(res)  

                # Reservation and guidance functions
                elif command == "reserve_parking":
                    # Example: reserve_parking number (e.g., "reserve_parking car")
                    number = args[0]

                    # Get statistics
                    ret, res = self.groups.get_group_statistics()
                    if ret:
                        # Find least reserved group
                        min_c = 100000
                        min_n = "False"

                        for room in res:
                            if (room[2]<room[1]) and (room[2]<min_c):
                                min_c = room[2]
                                min_n = room[0]
                        
                        # Send that as self.response
                        res = self.cars.set_car_group(number, min_n)
                        if res: self.response = min_n
                        else: self.response = "False"
                
                elif command == "relinquish_parking":
                    # Example: relinquish_parking number (e.g., "relinquish_parking car")
                    number = args[0]
                    self.response = str(self.cars.reset_car_group(number))

                else:
                    self.response = "Unknown command"

                self.get_logger().info(f"Received: {command} -> {self.response}")

                # Send the self.response back to the client
                client_socket.send(self.response.encode('utf-8'))

            except Exception as e:
                self.get_logger().error(f"Error handling client: {e}")
                break

        client_socket.close()


    def start_server(self, host, port):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((host, port))
        self.server.listen(5)
        self.get_logger().info(f"Coordinator server running on {host}:{port}")

        while rclpy.ok():
            try:
                # Accept new client connection
                client_socket, addr = self.server.accept()
                client_handler = threading.Thread(target=self.handle_client, args=(client_socket,))
                client_handler.start()

            except Exception as e:
                self.get_logger().error(f"Accept failed: {e}")
                break

    def destroy_node(self):
        self.get_logger().info("Shutting down Coordinator server node.")
        if self.server:
            self.server.close()
        if self.conn:
            self.conn.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = CoordinatorServer()

    try:
        rclpy.spin(node)
    
    except Exception as e:
        if rclpy.ok():
            node.get_logger().info(f"Error: {e}")
    
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()