#!/usr/bin/env python3
"""
quick_plot_occ.py  – Plot occupied cells from map_grid.csv

Usage:
    python3 quick_plot_occ.py  path/to/map_grid.csv
"""
import sys
import pandas as pd
import matplotlib.pyplot as plt

def main(csv_path: str):
    # 1. load
    df = pd.read_csv(csv_path)

    # 2. select occupied cells
    occ = df[df["state"] == "occupied"]

    if occ.empty:
        print("No occupied cells found – check the CSV content.")
        return

    # 3. scatter plot
    plt.figure(figsize=(6, 6))
    plt.scatter(occ["x"], occ["y"], s=2, marker="s")   # tiny squares
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Occupied cells")
    plt.axis("equal")
    plt.grid(True, linewidth=0.3)
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 quick_plot_occ.py map_grid.csv")
        sys.exit(1)
    main(sys.argv[1])