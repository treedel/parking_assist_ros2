import socket

class CoordinatorClientInterface:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port

    def send_request(self, command):
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.connect((self.hostname, self.port))
        client.send(command.encode('utf-8'))
        response = client.recv(1024).decode('utf-8').strip()
        client.close()
        return response
    
    def reserve_parking(self, number):
        return self.send_request(f"reserve_parking {number}")
    
    def relinquish_parking(self, number):
        return self.send_request(f"relinquish_parking {number}") == "True"
    
    def get_group_position(self, group_name):
        response = self.send_request(f"get_group_position {group_name}")
        if response == "[]": return []
        position = [float(i) for i in response[1:-1].split(", ")]
        return position
        
if __name__ == "__main__":
    parking = CoordinatorClientInterface("127.0.0.1", 5555)

    # Demo usage
    print(parking.reserve_parking("vav"))
    print(parking.get_group_position("A"))
    print(parking.relinquish_parking("vav"))