import numpy as np
## Creating Arrays

a = np.array([1,2,3])
print(a)


## Creating Empty list

l = []

# Creating a for loop which goes from 1 to 4 and takes
# input from user, once it gets 4 inputs it going to append those 
# numbers to the "l"

for i in range(1,5):
    int_1 = int(input("Enter: "))
    np.append(int_1)
print(np.array(l))

## Dimemsions 
x = [[1,2,3,4],[5,6,7,8],[9,10,11,12]]
y = np.array(x)
# print(y)


## How to create Arrays using in-built functions

# creating arrays with zeros
b = np.zeros([5,5],dtype=int)
print(b)

# Creating arrays with ones
c = np.ones([2,1],dtype=int)
print(c)

# Creating a matrix with diagonal elements as zero
d = np.eye(2,dtype=int)
print(d)

# Creating a matrix with ones
e = np.ones([2,3],dtype=float)
print(e)

## Creating array with arange function

f = np.arange(1,20,2)
print(f)

## Creating array with linspace
g = np.linspace(1,20,19)
print(g)