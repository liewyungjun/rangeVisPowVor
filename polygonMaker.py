
import matplotlib.pyplot as plt
import numpy as np

def onclick(event):
    if event.button == 1:  # Left click
        x, y = event.xdata, event.ydata
        if x is not None and y is not None:
            plt.plot(x, y, 'ro')  # Plot red dot
            with open('polygonPoints.txt', 'a') as f:
                f.write(f"{x},{y}\n")
            plt.draw()

fig, ax = plt.subplots()
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.grid(True)

# Clear the file before starting
open('polygonPoints.txt', 'w').close()

# Connect the click event to our handler
fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()
