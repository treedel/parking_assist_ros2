import sqlite3

# The class that manages cars
class CarManagement:
    def __init__(self, conn, reset=False):
        self.conn = conn
        self.cursor = self.conn.cursor()

        if reset:
            query = "DROP TABLE IF EXISTS cars"
            self.cursor.execute(query)
            self.conn.commit()

        query = """CREATE TABLE IF NOT EXISTS cars(
                        registrationNumber VARCHAR PRIMARY KEY,
                        name VARCHAR,
                        pin VARCHAR,
                        groupName VARCHAR
                )"""
        self.cursor.execute(query)
        self.conn.commit()

    def register_car(self, registration_number, name, pin):
        query = 'SELECT registrationNumber from cars WHERE registrationNumber = ?'
        self.cursor.execute(query, (registration_number,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if res: return False

        query = "INSERT INTO cars (registrationNumber, name, pin) VALUES (?, ?, ?)"
        self.cursor.execute(query, (registration_number, name, pin))
        self.conn.commit()

        return True

    def authenticate_car(self, registration_number, pin):
        query = 'SELECT registrationNumber FROM cars WHERE registrationNumber = ? AND pin = ?'
        self.cursor.execute(query, (registration_number, pin))
        result = self.cursor.fetchone()
        self.conn.commit()
        
        if result: return True
        return False
    
    def get_car_group(self, registration_number):
        query = 'SELECT groupName from cars WHERE registrationNumber = ? AND groupName IS NOT NULL'
        self.cursor.execute(query, (registration_number,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if not res: return False
        return res[0]

    def set_car_group(self, registration_number, group_name):
        query = 'SELECT groupName from groups WHERE groupName = ? AND capacity>occupied'
        self.cursor.execute(query, (group_name,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if not res: return False

        query = 'SELECT registrationNumber from cars WHERE registrationNumber = ?'
        self.cursor.execute(query, (registration_number,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if not res: return False

        # If there is already an existing group selected
        query = 'SELECT registrationNumber from cars WHERE registrationNumber = ? AND groupName IS NULL'
        self.cursor.execute(query, (registration_number,))
        res = bool(self.cursor.fetchone())
        self.conn.commit()

        if not res: self.reset_car_group(registration_number)

        query = 'UPDATE cars SET groupName = ? WHERE registrationNumber = ?'
        self.cursor.execute(query, (group_name, registration_number))
        self.conn.commit()

        query = 'UPDATE groups SET occupied = occupied+1 WHERE groupName = ?'
        self.cursor.execute(query, (group_name,))
        self.conn.commit()

        return True

    def reset_car_group(self, registration_number):
        query = 'SELECT groupName FROM cars WHERE registrationNumber = ? AND groupName IS NOT NULL'
        self.cursor.execute(query, (registration_number,))
        group_name = self.cursor.fetchone()
        self.conn.commit()

        if not group_name: return False

        group_name = group_name[0]

        query = 'UPDATE cars SET groupName = NULL WHERE registrationNumber = ?'
        self.cursor.execute(query, (registration_number,))
        self.conn.commit()

        query = 'UPDATE groups SET occupied = occupied-1 WHERE groupName = ? AND occupied>0'
        self.cursor.execute(query, (group_name,))
        self.conn.commit()

        return True

if __name__ == "__main__":
    conn = sqlite3.connect('parking.db')

    # Create test environment
    auth = CarManagement(conn)
    auth.register_car('TN00AA0001', 'aaa', 'aaa')
    auth.register_car('TN00AA0002', 'aba', 'aba')
    auth.register_car('TN00AA0003', 'eea', 'eea')
    auth.register_car('TN00AA0004', 'car', 'car')

    # Testing API functions
    print(auth.authenticate_car('TN00AA0004', 'car'))
    print(auth.set_car_group('TN00AA0002', 'A'))
    print(auth.set_car_group('TN00AA0004', 'B'))
    print(auth.reset_car_group('TN00AA0004'))
    print(auth.set_car_group('TN00AA0004', 'A'))
    print(auth.get_car_group('TN00AA0004'))