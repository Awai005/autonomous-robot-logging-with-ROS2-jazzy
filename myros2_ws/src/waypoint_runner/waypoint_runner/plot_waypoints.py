#!/usr/bin/env python3
import os, sqlite3, math, json
from pathlib import Path
import argparse
import pandas as pd
import matplotlib.pyplot as plt

def ns_to_sec(ns): return ns * 1e-9

import rosbag2_py
import rclpy.serialization
from nav_msgs.msg import Odometry

def read_bag_mcap(bag_path, topic_filter="/odom"):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    data = []
    while reader.has_next():
        topic, raw_data, t = reader.read_next()
        if topic == topic_filter:
            msg_type = rclpy.serialization.import_message_from_namespaced_type(type_map[topic])
            msg = rclpy.serialization.deserialize_message(raw_data, msg_type)
            data.append((t, msg.pose.pose.position.x, msg.pose.pose.position.y))
    return data


def parse_pose(msg_bytes):
    # rosbag2 stores CDR-serialized ROS 2 messages; easiest: use JSON if you used rosbag2 JSON storage plugin.
    # If not, fall back to odometry only OR use rosbag2_py/rosbags for full deserialization.
    # Here we handle odometry (nav_msgs/Odometry) via JSON (if you recorded with --storage sqlite3 and --serialization-format cdr, JSON isn't available).
    # To keep this script portable, we'll rely on /diff_drive_controller/odom for numeric plots and /checkpoint_log for events.
    return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="path to rosbag2 (directory)")
    ap.add_argument("--out", default="bag_export.csv")
    args = ap.parse_args()

    rows, topics, con = read_bag_sqlite(args.bag)

    # Try odom (nav_msgs/msg/Odometry) for trajectory & velocities
    odom_rows = rows("/diff_drive_controller/odom")
    # checkpoint messages
    log_rows  = rows("/checkpoint_log")

    if not odom_rows:
        print("No /diff_drive_controller/odom found. Did you record it?")
        return

    # We’ll decode CDR with 'rosbags' if present, else do a minimal JSON fallback
    # Attempt 'rosbags' first:
    have_rosbags = False
    try:
        from rosbags.highlevel import AnyReader
        have_rosbags = True
    except Exception:
        pass

    data = []
    if have_rosbags:
        with AnyReader([Path(args.bag)]) as reader:
            conns = {c.topic: c for c in reader.connections}
            # Odom
            for conn, ts, raw in reader.messages(conns["/diff_drive_controller/odom"]):
                msg = reader.deserialize(raw, conn.msgtype)
                t = ns_to_sec(ts)
                px = msg.pose.pose.position.x
                py = msg.pose.pose.position.y
                vx = msg.twist.twist.linear.x
                wz = msg.twist.twist.angular.z
                data.append({"t": t, "x": px, "y": py, "vx": vx, "wz": wz})
            # Checkpoint logs (std_msgs/String)
            log_data = []
            if "/checkpoint_log" in conns:
                for conn, ts, raw in reader.messages(conns["/checkpoint_log"]):
                    msg = reader.deserialize(raw, conn.msgtype)
                    log_data.append({"t": ns_to_sec(ts), "msg": msg.data})
    else:
        print("Tip: install 'rosbags' for full decoding:  pip install rosbags --break-system-packages")
        print("Without it, plotting is limited.")

    if not data:
        print("No decoded odom data; cannot plot.")
        return

    df = pd.DataFrame(data)
    df.to_csv(args.out, index=False)
    print(f"Wrote CSV: {args.out}")

    # Plots
    plt.figure()
    plt.plot(df["x"], df["y"])
    plt.xlabel("x [m]"); plt.ylabel("y [m]"); plt.title("Robot trajectory (odom)")
    plt.axis("equal"); plt.grid(True)
    plt.savefig("trajectory_xy.png", dpi=150)
    print("Saved: trajectory_xy.png")

    plt.figure()
    plt.plot(df["t"] - df["t"].iloc[0], df["vx"], label="linear x [m/s]")
    plt.plot(df["t"] - df["t"].iloc[0], df["wz"], label="angular z [rad/s]")
    plt.xlabel("time [s]"); plt.title("Velocities vs time"); plt.grid(True); plt.legend()
    plt.savefig("velocities.png", dpi=150)
    print("Saved: velocities.png")

if __name__ == "__main__":
    main()
