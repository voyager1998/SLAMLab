import numpy as np
import matplotlib.pyplot as plt
import sys
mapName = str(sys.argv[1])
map = np.loadtxt(mapName, dtype=int, skiprows=1)
disGrid = np.loadtxt("disGrid.txt", dtype=float)
print(map.shape)
with open('xy', 'r') as f:
  lines = f.readlines()
  print(len(lines))
  for i in range(len(lines)//2):
    x = int(lines[2*i])
    y = int(lines[2*i+1])
    map[y][x] = 100
    print(disGrid[y][x])
plt.imshow(map, cmap='gray_r', vmin = -127, vmax = 127)
plt.show()
plt.imshow(disGrid, cmap='hot')
plt.show()
