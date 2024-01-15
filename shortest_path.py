import networkx as nx
import heapq as hq

def load_graph_data(file_path):

    adjacency_matrix = []
    bandwidth_matrix = []
    delay_matrix = []
    reliability_matrix = []

    with open(file_path, 'r') as file:
        lines = file.readlines()

    current_matrix = None
    for line in lines:
        if not line.strip():
            continue
        if ':' in line:
            current_matrix = line.strip()[:-1].lower()
            continue
        elements = line.strip().split(':', 1)[1].split()

        if current_matrix == 'adjacency':
            adjacency_matrix.append(elements)
        elif current_matrix == 'bandwidth':
            bandwidth_matrix.append(elements)
        elif current_matrix == 'delay':
            delay_matrix.append(elements)
        elif current_matrix == 'reliability':
            reliability_matrix.append(elements)

    return adjacency_matrix, bandwidth_matrix, delay_matrix, reliability_matrix

def construct_graph(adjacency_matrix):
    graph = nx.Graph()
    num_nodes = len(adjacency_matrix)

    for i in range(num_nodes):
        graph.add_node(i)

    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            if adjacency_matrix[i][j] != 0:
                graph.add_edge(i, j, weight=adjacency_matrix[i][j])

    return graph

def simulated_annealing_algo(graph, source, destination, constraints):

    min_heap = [(0, source, [])]
    visited = set()

    while min_heap:
        cost, current, path = hq.heappop(min_heap)
        if current in visited:
            continue
        path = path + [current]
        if current == destination:
            return path
        visited.add(current)
        for neighbor in graph.neighbors(current):
            if neighbor not in visited and graph[current][neighbor]['weight'] >= constraints['bandwidth_demand']:
                heuristic_cost = graph.degree(neighbor)  # A simple heuristic
                hq.heappush(min_heap, (cost + graph[current][neighbor]['weight'] + heuristic_cost, neighbor, path))

    return []

def find_best_path(input_file_path, request, constraints):

    adjacency_matrix = load_graph_data(input_file_path)

    graph = construct_graph(adjacency_matrix)

    source_node, destination_node, bandwidth_requirement = request

    dijkstra_path = nx.shortest_path(graph, source=source_node, target=destination_node, weight='weight')
    bellman_ford_path = nx.bellman_ford_path(graph, source=source_node, target=destination_node, weight='weight')
    a_star_path = nx.astar_path(graph, source=source_node, target=destination_node, heuristic=None, weight='weight')

    distances, next_hop = nx.floyd_warshall_predecessor_and_distance(graph, weight='weight')
    floyd_warshall_path = nx.reconstruct_path(source=source_node, target=destination_node, predecessors=next_hop)

    simulated_annealing_path = simulated_annealing_algo(graph, source_node, destination_node, constraints)

    return dijkstra_path, bellman_ford_path, a_star_path, floyd_warshall_path, simulated_annealing_path

input_file_path = "test.txt"
request_details = (0, 3, 5)
constraints = {
    'bandwidth_demand': 5,
    'delay_threshold': 40,
    'reliability_threshold': 0.70
}

result_paths = find_best_path(input_file_path, request_details, constraints)

# Print the calculated paths.
print("Dijkstra's Path:", result_paths[0])
print("Bellman-Ford Path:", result_paths[1])
print("A* Path:", result_paths[2])
print("Floyd-Warshall Path:", result_paths[3])
print("Simulated Annealing Path:", result_paths[4])
