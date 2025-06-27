#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

def plot_all_joints(csv_file: str):
    # Load the CSV into a DataFrame
    df = pd.read_csv(csv_file)

    # List of joint names we expect
    joint_names = [f"joint{i}" for i in range(1, 7)]

    # Create a figure with 2 rows: one for position, one for velocity
    fig, (ax_pos, ax_vel) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Plot each joint in turn
    for joint in joint_names:
        df_joint = df[df["name"] == joint]
        if df_joint.empty:
            print(f"  Warning: no data found for {joint}. Skipping.")
            continue

        # Position plot
        ax_pos.plot(
            df_joint["timestamp"],
            df_joint["position"],
            label=joint
        )

        # Velocity plot
        ax_vel.plot(
            df_joint["timestamp"],
            df_joint["velocity"],
            label=joint
        )

    # Format the position subplot
    ax_pos.set_ylabel("Position")
    ax_pos.set_title("Joint Positions vs. Time (joint1–joint6)")
    ax_pos.grid(True)
    ax_pos.legend(loc="upper right", ncol=3, fontsize="small")

    # Format the velocity subplot
    ax_vel.set_xlabel("Time [s]")
    ax_vel.set_ylabel("Velocity")
    ax_vel.set_title("Joint Velocities vs. Time (joint1–joint6)")
    ax_vel.grid(True)
    ax_vel.legend(loc="upper right", ncol=3, fontsize="small")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description="Plot position & velocity for joint1…joint6 from a CSV"
    )
    parser.add_argument(
        "--csv",
        type=str,
        default="joint_states_log.csv",
        help="Path to the joint_states CSV file"
    )
    args = parser.parse_args()
    plot_all_joints(args.csv)
