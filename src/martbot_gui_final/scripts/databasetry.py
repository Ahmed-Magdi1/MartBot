import sqlite3

# Function to connect to the database
def connect_to_db(db_name="datatry.db"):
    return sqlite3.connect(db_name)

# Function to create a table
def create_table(connection):
    with connection:
        connection.execute("""
        CREATE TABLE IF NOT EXISTS products (
            code INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            price INTEGER NOT NULL,
            category TEXT NOT NULL,
            description TEXT NOT NULL
        );
        """)

# Function to insert data
def insert_data(connection,code,name, price, category,description):
    with connection:
        connection.execute("""
        INSERT INTO products (code,name, price, category, description) VALUES (?,?, ?, ?, ?);
        """, (code,name, price, category,description))

# Function to fetch all data
def fetch_all_data(connection):
    with connection:
        return connection.execute("SELECT * FROM products").fetchall()

# Function to fetch data by category
def fetch_data_by_category(connection, category):
    with connection:
        return connection.execute("SELECT * FROM products WHERE category = ?", (category,)).fetchall()

# Function to update a record
def update_data(connection, code, name=None, price=None, category=None,description=None):
    with connection:
        if name:
            connection.execute("UPDATE products SET name = ? WHERE code = ?", (name, code))
        if price:
            connection.execute("UPDATE products SET price = ? WHERE code = ?", (price, code))
        if category:
            connection.execute("UPDATE products SET category = ? WHERE code = ?", (category, code))
        if description:
            connection.execute("UPDATE products SET category = ? WHERE code = ?", (description, code))

# Function to delete a record
def delete_data(connection, code):
    with connection:
        connection.execute("DELETE FROM products WHERE code = ?", (code,))

# Main function to demonstrate all operations
def main():
    connection = connect_to_db()

    # Create the table
    create_table(connection)

    # Insert sample data
    print("Inserting data...")
    insert_data(connection,1,"Vcola", 15, "drinks"," VCola is a popular carbonated Egyptian made soft drink available, offering a bold and refreshing cola flavor. Known for its rich taste and satisfying fizz. It delivers a perfect balance of sweetness and tanginess, making it an enjoyable beverage for all occasions.Whether enjoyed on its own, served over ice, or paired with a meal, VCola is a go-to drink for those who love the classic cola experience. Its affordability and wide availability make it a favorite choice among consumers in Egypt.Perfect for social gatherings, road trips, or simply cooling down on a hot day, VCola captures the essence of refreshment in every sip.")
    insert_data(connection,2, "lemon fusion spuds chips", 13, "snacks","Lemon Fusion Spuds Chips bring a unique and tangy twist to classic potato chips. Made from premium-quality potatoes, these chips are thinly sliced, perfectly crisp, and infused with a zesty lemon flavor that awakens the taste buds. The citrusy punch is balanced with just the right amount of seasoning, creating a refreshing and savory snacking experience.Whether you're craving a light, flavorful snack or something to complement your favorite meals, Lemon Fusion Spuds Chips offer a bold and exciting taste with every bite. Perfect for sharing with friends, enjoying on the go, or pairing with dips, these chips deliver a satisfying crunch and a burst of lemony goodness.")
    insert_data(connection,3, "zeina tissue", 30, "cleaning items", "Zeina Tissues are a premium choice for those who value both softness and durability. Made from high-quality materials, these tissues offer a gentle touch on the skin while maintaining superior strength for everyday use. Whether for personal care, home, or office use, Zeina Tissues provide exceptional absorbency and reliability.Designed with a commitment to quality, Zeina Tissues are available in various sizes and packaging options, making them perfect for every occasion. From facial tissues to kitchen rolls, Zeina ensures a comforting and hygienic experience for you and your family. Trusted by households across Egypt, Zeina Tissues bring comfort and care to your daily life.")
    insert_data(connection,4, "oxi dish soap", 35, "cleaning items", "Oxi Dish Soap is a highly effective dishwashing solution designed to cut through tough grease and stubborn food residues with ease. Its powerful formula ensures sparkling clean dishes while being gentle on your hands. Infused with deep-cleaning agents, Oxi Dish Soap quickly removes oil, stains, and odors, leaving your plates, glasses, and cookware fresh and spotless.Whether tackling a small load or a sink full of dirty dishes, Oxi Dish Soap provides long-lasting foam and efficient cleaning with just a few drops. Its refreshing scent adds a pleasant touch to your dishwashing routine, making every wash a breeze. Trust Oxi for a superior clean that keeps your kitchenware shining.")
    insert_data(connection,5, "Elarosa", 55, "coffeetea","El Arosa Tea is one of Egypt’s most beloved and iconic tea brands, known for its rich flavor, deep aroma, and strong, satisfying taste. Made from carefully selected tea leaves, El Arosa Tea delivers a bold and refreshing experience, making it a staple in Egyptian households for generations.Whether enjoyed plain, with sugar, or infused with mint, this tea offers a perfect balance of strength and smoothness. Its deep red color and full-bodied flavor make it ideal for morning energy boosts or relaxing evening moments. Loved by tea enthusiasts across Egypt, El Arosa Tea represents tradition, quality, and the warmth of shared conversations over a steaming cup.")
    
    # # Fetch all data
    # print("\nAll data:")
    # all_data = fetch_all_data(connection)
    # for row in all_data:
    #     print(row)
    
    # # Fetch data by category
    # print("\nData in 'drinks' category:")
    # drinks_data = fetch_data_by_category(connection, "drinks")
    # for row in drinks_data:
    #     print(row)
    
    # # Update data
    # print("\nUpdating Coke price to 5...")
    # update_data(connection, code=1, price=5)

    # # Fetch updated data
    # print("\nUpdated data:")
    # updated_data = fetch_all_data(connection)
    # for row in updated_data:
    #     print(row)

    # # Delete data
    # print("\nDeleting 'Pepsi'...")
    # delete_data(connection, code=2)

    # # Fetch data after deletion
    # print("\nData after deletion:")
    # remaining_data = fetch_all_data(connection)
    # for row in remaining_data:
    #     print(row)

    # Close the connection
    connection.close()

if __name__ == "__main__":
    main()
