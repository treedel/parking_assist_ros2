from CoordinatorClientInterface import CoordinatorClientInterface

interface = CoordinatorClientInterface("127.0.0.1", 5555)

class ClientCli:
    def __init__(self, number):
        self.number = number
        self.group_name = None

    def reserve_slot(self):
        print("Reserving your slot...")
        if self.group_name:
            print("You already have a reserved slot")
            return
        res = interface.reserve_parking(self.number)
        if res == "False":
            print("Error reserving slot!\n")
            return     
        self.group_name = res
        print(f"Seat reserved successfully. Group {self.group_name}\n")

    def start_guidance_system(self):
        print("Starting the guidance system...")
        pos = interface.get_group_position(self.group_name)
        print("Target position: ", pos)
        print("Guidance system is now running!\n")

    def exit_program(self):
        interface.relinquish_parking(self.number)
        self.group_name = None
        print("Exiting the program")
        exit()

    def run(self):
        while True:
            print("=== Main Menu ===")
            print("1. Reserve slot")
            print("2. Start Guidance System")
            print("3. Exit")
            
            choice = input("Enter your choice: ")
            if choice == '1':
                self.reserve_slot()
            elif choice == '2':
                self.start_guidance_system()
            elif choice == '3':
                self.exit_program()
            else:
                print("Invalid choice\n")

def main():
    print("Parking assistance system")
    print("=========================")
    number = input("Enter your car registration number: ")
    client = ClientCli(number)
    client.run()

if __name__ == "__main__":
    main()
