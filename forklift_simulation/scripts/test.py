import math
from math import pi
import numpy as np

cake = np.array([[[1,2,3]],[[4,5,5]],[[6,7,8]]])
x = np.array([1,2,3,4,5,6,7,8,9])
y = np.linalg.norm(cake - [1,2,3], axis=1)
print(cake.shape)

#change cake to 2d array
cake = cake.reshape(-1,3)
print((cake[1] == cake[1]).all())

# print(np.array(cake[-1:-4:-1])[:,0])
# print(np.array(cake[-1:-4:-1])[:,0] + [1,2,3])