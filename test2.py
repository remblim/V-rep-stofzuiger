import numpy as np
import matplotlib.pyplot as plt
import nudged

x = [[1,0],[0,1],[-1,0],[0,-1]]
y = np.add([[0,-2],[2,0],[0,2],[-2,0]],1)

trans = nudged.estimate(x, y);
y1 = trans.transform([1,0])
print(trans.get_rotation())
print(trans.get_scale())
print(trans.get_translation())
print(y1)