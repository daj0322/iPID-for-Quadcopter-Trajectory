import numpy as np

var = np.array([9,8,7,6,5,4])
print(var)
print()
for i in var:
    print(i)
    
    
var2 = np.array([[1,2,3,4],[1,2,3,4]])
print(var2)
print()

for i,d in np.ndenumerate(var2):
    print(i,d)