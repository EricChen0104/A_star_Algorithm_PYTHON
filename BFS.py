import numpy as np
import matplotlib.pyplot as plt
import queue

grid_size = 20
map_size = (grid_size, grid_size)
grid = np.zeros(map_size)

grid[5:10, 8] = 1
grid[15, 5:15] = 1

grid[12, 2:7] = 1

start = (2, 3)
goal = (17, 14)

# grid[start] = 0.5
# grid[goal] = 0.75

q = queue.Queue()
q.put(start)

level = [[-1 for x in range(grid_size)] for y in range(grid_size)] 
level[start[0]][start[1]] = 0

# BFS
while not q.empty():
    x, y = q.get()
    d = [[0, 1], [1, 0], [-1, 0], [0, -1]]
    for i in range(len(d)):
        dx = x + d[i][0]
        dy = y + d[i][1]
        if (0 <= dx < grid_size and 0 <= dy < grid_size and 
            level[dx][dy] == -1 and (grid[dx][dy] == 0 or (dx, dy) == goal)):
            q.put((dx, dy))
            level[dx][dy] = level[x][y] + 1

if level[goal[0]][goal[1]] == -1:
    print("Goal is not reachable from start!")
    print("Checking goal neighbors:")
    d = [[0, 1], [1, 0], [-1, 0], [0, -1]]
    for i in range(len(d)):
        dx = goal[0] + d[i][0]
        dy = goal[1] + d[i][1]
        if 0 <= dx < grid_size and 0 <= dy < grid_size:
            print(f"Neighbor ({dx}, {dy}): level={level[dx][dy]}, grid={grid[dx][dy]}")
else:
    path = []
    current = goal
    while current != start:
        path.append(current)
        x, y = current
        found = False
        d = [[0, 1], [1, 0], [-1, 0], [0, -1]]
        for i in range(len(d)):
            dx = x + d[i][0]
            dy = y + d[i][1]
            if 0 <= dx < grid_size and 0 <= dy < grid_size and level[dx][dy] == level[x][y] - 1:
                current = (dx, dy)
                found = True
                break
        if not found:
            print("Error: Cannot find path back to start!")
            break
    if current == start:
        path.append(start)
        path.reverse()

        grid_copy = grid.copy()
        grid_copy[start] = 0.5 
        grid_copy[goal] = 0.75 
        for x, y in path:
            if (x, y) != start and (x, y) != goal:
                grid_copy[x][y] = 0.3  

        # 繪圖
        plt.figure(figsize=(6, 6))
        plt.imshow(grid_copy, cmap='gray_r')
        plt.title("2D Grid Map with BFS Shortest Path")
        path_x, path_y = zip(*path)
        plt.plot(path_y, path_x, 'b-', linewidth=2)
        plt.xticks(np.arange(0, map_size[1], 1))
        plt.yticks(np.arange(0, map_size[0], 1))
        plt.gca().set_xticks(np.arange(-.5, map_size[1], 1), minor=True)
        plt.gca().set_yticks(np.arange(-.5, map_size[0], 1), minor=True)
        plt.grid(which='minor', color='black', linewidth=0.5)
        plt.gca().invert_yaxis()
        plt.show()