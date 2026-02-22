import heapq
import matplotlib.pyplot as plt
import numpy as np
import math


def heuristic(a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def astar(start, goal, grid):
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    
    came_from = {}
    g_score = {start: 0}
    
    directions = [(-1,0),(1,0),(0,-1),(0,1),
                  (-1,-1),(-1,1),(1,-1),(1,1)]
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor] == 1:
                    continue  
                
                tentative_g = g_score[current] + heuristic(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
    
    return None  




def build_full_route(start, pillars, gate, grid):
    full_path = []
    current = start
    
    targets = pillars + [gate]
    for target in targets:
        path_segment = astar(current, target, grid)
        if path_segment is None:
            print("Путь не найден!")
            return None
        if full_path:
            full_path.extend(path_segment[1:])
        else:
            full_path.extend(path_segment)
        current = target
        
    return full_path




def visualize(grid, path, pillars, gate):
    plt.figure()

    
  
    plt.imshow(grid.T, origin='lower', cmap='gray_r')
    
   
    x = [p[0] for p in path]
    y = [p[1] for p in path]
    plt.plot(x, y, color='red', linewidth=2)
    
   
    for i, p in enumerate(pillars):
        plt.scatter(p[0], p[1])
        plt.text(p[0], p[1], f' Pillar {i+1}')
    
    
    plt.scatter(gate[0], gate[1])
    plt.text(gate[0], gate[1], ' Gate')
    
    plt.title("Маршрут A*")
    plt.grid(True)
    plt.show()

grid_size = 10
grid = np.zeros((grid_size, grid_size))

start = (1, 1)

pillars = [
    (5, 9),
    (6,6),
    (9, 2)
]

gate = (9, 9)

#grid[5:15, 12] = 1



path = build_full_route(start, pillars, gate, grid)

if path:
    print("Длина пути:", len(path))
    visualize(grid, path, pillars, gate)