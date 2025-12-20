import matplotlib.pyplot as plt
import numpy as np

## Making simple plots, trying to understand the syntax of plot 

x = np.array([2023,2024,2025,2026])
y1 = np.array([15,25,30,20])
y2 = np.array([17,23,38,5])
y3 = np.array([13,15,20,30])

## We made a dictionary so we dont have to write the line styles everytime

line_style = dict(marker = "o",   
                  markersize = 10,    #You can also use ms instead of marker size
                  markerfacecolor="cyan",
                  markeredgecolor = "red",
                  linestyle="--",
                  linewidth=2)


## to Unpack dictionary we have to write ** before the name of the dict

plt.plot(x,y1,**line_style,color="blue") 
plt.plot(x,y2,**line_style,color = "red")
plt.plot(x,y3,**line_style,color="black")


# Giving a title to the graph 

plt.title("Class size",fontsize=25,
                       family="Arial",
                       fontweight = "bold",
                       color = "Orange")

# Labelling the axes

plt.xlabel("Year",fontsize=25,
                  family="Arial",
                  fontweight = "bold",
                  color = "Orange")

plt.ylabel("Students",fontsize=25,
                  family="Arial",
                  fontweight = "bold",
                  color = "Blue")

## Adding Grid lines
plt.grid()

##
plt.tight_layout()

plt.show()



## Bar charts




