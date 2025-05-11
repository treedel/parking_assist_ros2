import sqlite3

# The class that handle cars
class CarManagement:
    def __init__(self, conn, reset=False, reload=False):
        self.conn = conn
        self.cursor = self.conn.cursor()

        if reset:
            query = "DROP TABLE IF EXISTS cars"
            self.cursor.execute(query)
            self.conn.commit()

        query = """CREATE TABLE IF NOT EXISTS cars(
                        number VARCHAR PRIMARY KEY,
                        groupName VARCHAR
                )"""
        self.cursor.execute(query)
        self.conn.commit()

        if reload:
            query = 'UPDATE cars SET groupName = NULL'
            self.cursor.execute(query)
            self.conn.commit()
    
    def get_car_group(self, number):
        query = 'SELECT groupName from cars WHERE number = ? AND groupName IS NOT NULL'
        self.cursor.execute(query, (number,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if not res: return False
        return res[0]

    def set_car_group(self, number, group_name):
        query = 'SELECT number from cars WHERE number = ?'
        self.cursor.execute(query, (number,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if not res:
            query = "INSERT INTO cars (number, groupName) VALUES (?, NULL)"
            self.cursor.execute(query, (number,))
            self.conn.commit()

        query = 'SELECT groupName from groups WHERE groupName = ? AND capacity>occupied'
        self.cursor.execute(query, (group_name,))
        res = self.cursor.fetchone()
        self.conn.commit()
        if not res: return False

        # If there is already an existing group selected
        query = 'SELECT number from cars WHERE number = ? AND groupName IS NULL'
        self.cursor.execute(query, (number,))
        res = bool(self.cursor.fetchone())
        self.conn.commit()

        if not res: self.reset_car_group(number)

        query = 'UPDATE cars SET groupName = ? WHERE number = ?'
        self.cursor.execute(query, (group_name, number))
        self.conn.commit()

        query = 'UPDATE groups SET occupied = occupied+1 WHERE groupName = ?'
        self.cursor.execute(query, (group_name,))
        self.conn.commit()

        return True

    def reset_car_group(self, number):
        query = 'SELECT groupName FROM cars WHERE number = ? AND groupName IS NOT NULL'
        self.cursor.execute(query, (number,))
        group_name = self.cursor.fetchone()
        self.conn.commit()

        if not group_name: return False

        group_name = group_name[0]

        query = 'UPDATE cars SET groupName = NULL WHERE number = ?'
        self.cursor.execute(query, (number,))
        self.conn.commit()

        query = 'UPDATE groups SET occupied = occupied-1 WHERE groupName = ? AND occupied>0'
        self.cursor.execute(query, (group_name,))
        self.conn.commit()

        return True

if __name__ == "__main__":
    conn = sqlite3.connect('parking.db', True)

    # Create test environment
    cars = CarManagement(conn)

    # Testing API functions
    print(cars.set_car_group('TN00AA0002', 'A'))
    print(cars.set_car_group('TN00AA0004', 'B'))
    print(cars.reset_car_group('TN00AA0004'))
    print(cars.set_car_group('TN00AA0004', 'A'))
    print(cars.get_car_group('TN00AA0004'))