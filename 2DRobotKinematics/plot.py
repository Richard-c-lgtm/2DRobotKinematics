import pandas as pd
import matplotlib.pyplot as plt
import os


csv_path = '/Users/chenhao/Desktop/C++/C++ Program/C+++/Project_1/2DRobotKinematics-epacaldzkbkcplcncqkobtwygijf/Build/Products/Debug/trajectory.csv'

cases = {
    3: [(5.0, 0.0), (5.0, 5.0), (0.0, 5.0)],
    4: [(2.0, 0.0), (2.0, 2.0), (-2.0, 2.0), (-2.0, -2.0)],
    5: [(2.0, 2.0), (4.0, 0.0), (6.0, 2.0), (8.0, 0.0)],
    6: [(10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)],
    7: [(0.0, 5.0), (-5.0, 0.0), (0.0, -5.0), (5.0, 0.0)],
    8: [(2.5, 5.0),(4.0, 4.5),(4.0, 3.5),(1.0, 1.0),(4.5, 1.0)]
}

target = cases[7]

def plot_robot_trajectory():
    if not os.path.exists(csv_path):
        print(f"Cannot find file：{csv_path}")
        return

    df = pd.read_csv(csv_path)

    plt.figure(figsize=(10, 10))

    plt.plot(df['x'], df['y'], color='#1f77b4', label='Actual Path', linewidth=1.5, zorder=2)

    tx, ty = zip(*target)
    plt.scatter(tx, ty, s=150, facecolors='none', edgecolors='red', linewidths=2, label='Target Waypoints', zorder=5)

    for i, (x, y) in enumerate(target):
        plt.text(x + 0.15, y + 0.15, f'P{i+1}', fontsize=12, color='red', fontweight='bold')

    plt.scatter(df['x'].iloc[0], df['y'].iloc[0], color='green', s=100, marker='D', label='Start', zorder=6)
    plt.scatter(df['x'].iloc[-1], df['y'].iloc[-1], color='black', s=150, marker='X', label='End', zorder=6)

    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(loc='upper right')
    plt.title('Robot Project: Test (Full Quadrant Navigation)', fontsize=14)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')

    plt.savefig('test_result_7.png', dpi=150)
    print(f"Final Position: x={df['x'].iloc[-1]:.4f}, y={df['y'].iloc[-1]:.4f}")
    plt.show()

if __name__ == "__main__":
    plot_robot_trajectory()
