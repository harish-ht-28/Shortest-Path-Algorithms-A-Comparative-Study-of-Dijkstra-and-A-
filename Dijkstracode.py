import heapq
import math

def dijkstra(graph, start, end):
    pq = []
    heapq.heappush(pq, (0, start))
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        if current_node == end:
            break
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(pq, (distance, neighbor))
    path, node = [], end
    while previous_nodes[node] is not None:
        path.insert(0, node)
        node = previous_nodes[node]
    if path:
        path.insert(0, start)
    return path, distances[end]
def heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
def a_star(graph, start, end, coordinates):
    pq = []
    heapq.heappush(pq, (0, start))
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(coordinates[start], coordinates[end])
    previous_nodes = {node: None for node in graph}   
    while pq:
        _, current_node = heapq.heappop(pq)
        if current_node == end:
            break
        for neighbor, weight in graph[current_node].items():
            tentative_g_score = g_score[current_node] + weight
            if tentative_g_score < g_score[neighbor]:
                previous_nodes[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(coordinates[neighbor], coordinates[end])
                heapq.heappush(pq, (f_score[neighbor], neighbor))
    
    path, node = [], end
    while previous_nodes[node] is not None:
        path.insert(0, node)
        node = previous_nodes[node]
    if path:
        path.insert(0, start)
    return path, g_score[end]
