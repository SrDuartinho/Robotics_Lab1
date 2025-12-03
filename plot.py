import matplotlib.pyplot as plt

class LiveSensorPlot:
    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()

        self.min_distances = []
        (self.line,) = self.ax.plot(self.min_distances, label="Closest Lane Distance")

        self.ax.legend()
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Distance to Nearest Lane")

    def update(self, distance):
        if distance is None:
            return

        self.min_distances.append(distance)

        x = range(len(self.min_distances))
        self.line.set_xdata(x)
        self.line.set_ydata(self.min_distances)

        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001)
