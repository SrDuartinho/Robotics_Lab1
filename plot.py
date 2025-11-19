import matplotlib.pyplot as plt

class LiveSensorPlot:
    def __init__(self):
        plt.ion()  # interactive mode ON
        self.fig, self.ax = plt.subplots()

        # Data for 4 rays
        self.left1_data = []
        self.left2_data = []
        self.right1_data = []
        self.right2_data = []

        # Create 4 line objects
        (self.left1_line,) = self.ax.plot(self.left1_data, label="Left Ray 1")
        (self.left2_line,) = self.ax.plot(self.left2_data, label="Left Ray 2")
        (self.right1_line,) = self.ax.plot(self.right1_data, label="Right Ray 1")
        (self.right2_line,) = self.ax.plot(self.right2_data, label="Right Ray 2")

        self.ax.legend()
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Hit Distance")

    def update(self, distances):
        """distances = [L1, L2, R1, R2]"""

        l1, l2, r1, r2 = distances

        # Append new data
        self.left1_data.append(l1)
        self.left2_data.append(l2)
        self.right1_data.append(r1)
        self.right2_data.append(r2)

        # Update line data
        self.left1_line.set_ydata(self.left1_data)
        self.left2_line.set_ydata(self.left2_data)
        self.right1_line.set_ydata(self.right1_data)
        self.right2_line.set_ydata(self.right2_data)

        # X-axis is time steps
        x = range(len(self.left1_data))
        self.left1_line.set_xdata(x)
        self.left2_line.set_xdata(x)
        self.right1_line.set_xdata(x)
        self.right2_line.set_xdata(x)

        # Autoscale
        self.ax.relim()
        self.ax.autoscale_view()

        plt.draw()
        plt.pause(0.001)
