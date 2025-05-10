def reserve_seat():
    print("Reserving your seat...")
    # Add your logic here
    print("Seat reserved successfully!\n")


def start_guidance_system():
    print("Starting the guidance system...")
    # Add your logic here
    print("Guidance system is now running!\n")


def exit_program():
    print("Exiting the program.")
    exit()

def main():
    while True:
        print("=== Main Menu ===")
        print("1. Reserve Seat")
        print("2. Start Guidance System")
        print("3. Exit")
        
        choice = input("Enter your choice (1-3): ")
        
        if choice == '1':
            reserve_seat()
        elif choice == '2':
            start_guidance_system()
        elif choice == '3':
            exit_program()
        else:
            print("Invalid choice. Please enter 1, 2, or 3.\n")

if __name__ == "__main__":
    main()
