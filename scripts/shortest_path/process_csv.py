import csv 
def read_csv_to_matrix(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        matrix = list(reader)
    dist_matrix = []
    orient_matrix = []
    for row in matrix:
        dist_row = []
        orient_row = []
        for cell in row:
            if cell == '[]':
                dist_row.append(float('inf'))  # No path available
                orient_row.append(float('inf')) # No angle available
            else:
                # Convert the string "[100, 90]" to a list [100, 90]
                cell = cell[1:-1]
                data = cell.split(',')
                print(data)
                distance = float(data[0])
                orientation = float(data[1])
                dist_row.append(distance)
                orient_row.append(orientation)
        dist_matrix.append(dist_row)
        orient_row.append(orient_row)

    # Define the CSV file name
    dist_file = 'dist.csv'
    orient_file = 'orient.csv'

    # Write the 2D array to the CSV file
    with open(dist_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(dist_matrix)
    # Write the 2D array to the CSV file
    with open(orient_file, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(orient_matrix)

    print(f"2D array has been written")
    

read_csv_to_matrix('adjacency.csv')

