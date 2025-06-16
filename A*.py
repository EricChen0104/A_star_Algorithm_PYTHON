import matplotlib.pyplot as plt
import numpy as np
import heapq

def h(node, goal):
    return abs(goal[0] - node[0]) + abs(goal[1] - node[1])

def g(orig_node, x):
    return abs(orig_node[0] - x[0]) + abs(orig_node[1] - x[1])

def in_bound(node, grid_size):
    return 0 <= node[0] < grid_size and 0 <= node[1] < grid_size

def is_obstacle(node, grid):
    return grid[node[0]][node[1]] == 1

# 建立地圖
grid_size = 20
map_size = (grid_size, grid_size)
grid = np.zeros(map_size)

# 設置障礙物
grid[5:10, 8] = 1
grid[15, 5:15] = 1
grid[12, 2:7] = 1
grid[8, 12:16] = 1
grid[1:4, 10] = 1

start = (2, 3)
goal = (17, 14)

# A* 演算法
open_list = []
heapq.heappush(open_list, (h(start, goal), start))
close_list = set()
g_score = {start: 0}
came_from = {}

while open_list:
    f_current, current = heapq.heappop(open_list)

    if current == goal:
        break

    if current in close_list:
        continue

    close_list.add(current)

    d = [[0, 1], [1, 0], [-1, 0], [0, -1]]

    for dx, dy in d:
        neighbor = (current[0] + dx, current[1] + dy)

        if not in_bound(neighbor, grid_size):
            continue
        if is_obstacle(neighbor, grid):
            continue

        tentative_g = g_score[current] + 1

        if neighbor in g_score and tentative_g >= g_score[neighbor]:
            continue

        came_from[neighbor] = current
        g_score[neighbor] = tentative_g
        f = tentative_g + h(neighbor, goal)
        heapq.heappush(open_list, (f, neighbor))

# 印出所有試圖拜訪的節點
print("所有試圖拜訪的節點（包含 open_list 和 close_list 中的節點）：")
visited_nodes = list(g_score.keys())
for node in visited_nodes:
    print(f"節點: {node}, g 分數: {g_score[node]}")

# 繪製最終路徑
path = []
node = goal
while node != start:
    path.append(node)
    node = came_from[node]
path.append(start)

# 印出最終路徑
print("\n最終走過的路徑節點（從起點到終點）：")
for node in reversed(path):  # 反轉以從起點到終點順序顯示
    print(f"節點: {node}, g 分數: {g_score[node]}")

# 建立顯示地圖，根據 g 分數設定顏色
display_grid = np.zeros(map_size)
for node in visited_nodes:
    # 將 g 分數正規化到 0.4~0.8 的範圍（避免與最終路徑、起點、終點、障礙物混淆）
    normalized_g = 0.4 + 0.4 * (g_score[node] / max(g_score.values()))
    display_grid[node[0]][node[1]] = normalized_g

# 標記最終路徑、起點、終點和障礙物
for x, y in path:
    display_grid[x][y] = 0.3  # 最終路徑用固定值（較淺顏色）
display_grid[start] = 0.9  # 起點
display_grid[goal] = 0.9   # 
display_grid[grid == 1] = 1.0  

# 準備最終路徑的 x 和 y 座標以畫紅線
path_x = [node[1] for node in reversed(path)]  # x 座標 (列)
path_y = [node[0] for node in reversed(path)]

# 繪圖
plt.figure(figsize=(6, 6))
plt.imshow(display_grid, cmap='Greens')  
plt.colorbar(label='g_score (color depth')
plt.plot(path_x, path_y, 'r-', linewidth=2, label='最終路徑')  # 畫紅線
plt.scatter(path_x[0], path_y[0], c='red', s=100, marker='o', label='起點')  # 標記起點
plt.scatter(path_x[-1], path_y[-1], c='blue', s=100, marker='*', label='終點')  # 標記終點
plt.title("2D Grid Map with A* Final Path (including all visited nodes)")
plt.xticks(np.arange(0, map_size[1], 1))
plt.yticks(np.arange(0, map_size[0], 1))
plt.gca().set_xticks(np.arange(-.5, map_size[1], 1), minor=True)
plt.gca().set_yticks(np.arange(-.5, map_size[0], 1), minor=True)
plt.grid(which='minor', color='black', linewidth=0.5)
plt.gca().invert_yaxis()
plt.show()