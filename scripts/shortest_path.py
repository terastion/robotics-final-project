import csv
import sys

def read_csv_to_matrix(filename):
    """Reads a CSV file and converts it into a 2D list (adjacency matrix)."""
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        matrix = list(reader)
    adj_matrix = []
    for row in matrix:
        adj_row = []
        for cell in row:
            if cell == '[]':
                adj_row.append(float('inf'))
            else:
                try:
                    data = ast.literal_eval(cell)
                    distance = data[0]
                    adj_row.append(distance)
                except:
                    adj_row.append(float('inf'))
        adj_matrix.append(adj_row)
    return adj_matrix

def dijkstra(adj_matrix, source):
    """Applies Dijkstra's algorithm to find the shortest path from the source to all other vertices."""
    n = len(adj_matrix)
    dist = [float('inf')] * n
    dist[source] = 0
    visited = [False] * n
    predecessor = [-1] * n

    for _ in range(n):
        # Find the vertex with the minimum distance from the set of vertices not yet processed.
        u = min((v for v in range(n) if not visited[v]), key=lambda v: dist[v])
        visited[u] = True
        
        # Update distance of the adjacent vertices of the picked vertex.
        for v in range(n):
            if adj_matrix[u][v] > 0 and not visited[v] and dist[v] > dist[u] + adj_matrix[u][v]:
                dist[v] = dist[u] + adj_matrix[u][v]
                predecessor[v] = u

    return dist, predecessor

def shortest_path(predecessor, source, dest):
    """Reconstructs the shortest path from source to dest using the predecessor array."""
    path = []
    step = dest
    while step != -1:
        path.append(step)
        step = predecessor[step]
        if step == source:
            path.append(step)
            break
    return path[::-1]

# Usage
filename = 'path_to_your_file.csv'
source_vertex = 0  # change as needed
destination_vertex = 4  # change as needed

adj_matrix = read_csv_to_matrix(filename)
distances, predecessors = dijkstra(adj_matrix, source_vertex)
path = shortest_path(predecessors, source_vertex, destination_vertex)

print("Shortest distances:", distances)
print("Shortest path from", source_vertex, "to", destination_vertex, ":", path)
