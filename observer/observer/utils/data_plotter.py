import matplotlib.pyplot as plt
import os
import numpy as np


class DataPlotter:

    def __init__(self, save_path="figures"):

        self.save_path = save_path

        if not os.path.exists(save_path):
            os.makedirs(save_path)

        self.data = {}

 

    def plot(self,
         time,
         real,
         observed,
         ylabel,
         filename):

        plt.figure(figsize=(10, 6))

        plt.plot(time, real, label="Real")
        plt.plot(time, observed, label="Observed")

        plt.xlabel("time [s]")
        plt.ylabel(ylabel)

        plt.grid(True)
        plt.legend()

        save_file = os.path.join(
            self.save_path,
            filename
        )

        plt.savefig(
            save_file,
            dpi=300,
            bbox_inches="tight"
        )

        plt.close()

        print(f"Saved: {save_file}")