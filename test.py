import numpy as np
h=np.zeros((5,6))
print(h)
grid = [[0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0], 
            [0, 1, 0, 0, 0, 0],
            [0, 1, 0, 0, 1, 0],
            [0, 0, 0, 0, 1, 0]]
print(len(grid[0]))
print(len(grid))
heuristic = [[0 for row in range(len(grid[0]))] for col in range(len(grid))] #0x6x5
print(heuristic)
print(h==heuristic)

