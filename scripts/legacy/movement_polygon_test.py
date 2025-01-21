
import matplotlib.pyplot as plt
import numpy as np

fig, ax = plt.subplots()
point, = ax.plot([], [], 'bo')  # Blue point
circle = plt.Circle((0, 0), 2.0, fill=False, color='r')  # Red circle
ax.add_patch(circle)

# Set the plot limits
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.grid(True)
ax.set_aspect('equal')

def on_mouse_move(event):
    if event.inaxes and hasattr(event, 'button') and event.button == 1:  # Left click
        x, y = event.xdata, event.ydata
        point.set_data([x], [y])
        circle.center = (x, y)
        fig.canvas.draw_idle()

fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)
plt.show()

