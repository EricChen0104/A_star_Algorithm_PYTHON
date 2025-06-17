# A* Path Planning Alogrithm

This project is a simple and interactive visualization of the A* pathfinding algorithm. It demonstrates how the algorithm finds the shortest path from a start point to a goal while avoiding obstacles on a 2D grid. Built with Python and matplotlib, this tool is great for learning and teaching path planning concepts.

## DEMO
<img width="554" alt="截圖 2025-06-17 上午9 51 01" src="https://github.com/user-attachments/assets/c7b450df-b340-48ad-8bbf-e73f33a0db6d" />

### Installation 
clone the whole repository.
``` bash
 
git clone https://github.com/EricChen0104/A_star_Algorithm_PYTHON.git
 
```
source to the A*_env
```

cd A_star_Algorithm_PYTHON
source A*_env/bin/activate
 
```
RUN THE CODE
```
 
// A*
python A*.py

// A* with weight
python A*_weight.py

// BFS (Breath First Search)
python BFS.py
 
```

## HOW A* WORKS
A* is a kind of Heuristic Search, combining the benefit of Dijkstra alogrithm and Greedy Search. Maybe some beginners have already studied the BFS algorithm. The main idea of A* is to deal with the time complexity of BFS.

### Basic Concepts
Each node has its own esimate function:

*f(n) = g(n) + h(n)*

 - *g(n)*: the actual cost from the start node to the current node *n*.
 - *h(n)*: the estimate cost from the node *n* to the goal node.

A* will explore from the "minimum cost of *f(n)*".

### Popular Heuristic function *h(n)*
Manhattan Distance:

*h(n) = |Xgoal - Xn| + |Ygoal - Yn|*

## References
- Yan, Y. (2023). Research on the A Star algorithm for finding shortest path. Highlights in Science, Engineering and Technology, 46, 154-161.
- https://www.geeksforgeeks.org/a-search-algorithm/
- https://en.wikipedia.org/wiki/A*_search_algorithm
