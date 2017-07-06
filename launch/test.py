import numpy as np

c = np.ones((3,1))
print c

if c[0]:
	c[1:3] = False

print c