import sqlite3

# The class that handles parking locations
class ParkingGroupManagement:
    def __init__(self, conn, reset=False, refresh=True):
        self.conn = conn
        self.cursor = self.conn.cursor()

        if reset:
            query = "DROP TABLE IF EXISTS groups"
            self.cursor.execute(query)
            self.conn.commit()
        
        query = """CREATE TABLE IF NOT EXISTS groups(
                        groupName VARCHAR PRIMARY KEY,
                        capacity INTEGER,
                        occupied INTEGER DEFAULT 0,
                        posX DOUBLE,
                        posY DOUBLE,
                        phi DOUBLE
                )"""
        self.cursor.execute(query)
        self.conn.commit()

        if refresh:
            query = "UPDATE groups SET occupied=0"
            self.cursor.execute(query)
            self.conn.commit()
    
    def add_group(self, group_name, capacity, pos_x, pos_y, phi):
        query = 'SELECT groupName from groups WHERE groupName = ?'
        self.cursor.execute(query, (group_name,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if res: return False

        query = "INSERT INTO groups (groupName, capacity, posX, posY, phi) VALUES (?, ?, ?, ?, ?)"
        self.cursor.execute(query, (group_name, capacity, pos_x, pos_y, phi))
        self.conn.commit()

        return True
    
    def remove_group(self, group_name):
        query = 'SELECT groupName from groups WHERE groupName = ?'
        self.cursor.execute(query, (group_name,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if not res: return False

        query = "DELETE FROM groups WHERE groupName = ?"
        self.cursor.execute(query, (group_name,))
        self.conn.commit()

        return True
    
    def get_group_position(self, group_name):
        query = 'SELECT posX, posY, phi from groups WHERE groupName = ?'
        self.cursor.execute(query, (group_name,))
        res = self.cursor.fetchone()
        self.conn.commit()

        if not res: return False, []

        return True, list(map(float, res))
    
    def get_groups_statistics(self):
        query = 'SELECT groupName, capacity, occupied from groups'

        self.cursor.execute(query)
        res = self.cursor.fetchall()
        self.conn.commit()
        
        if not res: return False, []

        return True, res

if __name__ == "__main__":
    conn = sqlite3.connect('parking.db')

    # Create test environment
    groups = ParkingGroupManagement(conn)
    groups.add_group('A', 4, 0.3, 1.0, 1.57)
    groups.add_group('B', 4, 10.2, 1.1, -1.57)

    groups.add_group('1', 4, 1.2, 5.1, -1.57)
    groups.add_group('2', 4, 10.2, 5.1, -1.57)

    ret, position = groups.get_group_position('A')
    if (ret): print(position)
    else: print("Couldn't find the specified room")

    print(groups.get_groups_statistics())
    groups.remove_group('2')
    groups.remove_group('1')
    print(groups.get_groups_statistics())