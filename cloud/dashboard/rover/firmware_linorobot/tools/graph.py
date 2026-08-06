"""Ask rclpy directly what publishers exist, bypassing the ros2 daemon.

`ros2 node list` returned empty while nodes were provably running, so the
daemon's graph cache is not trustworthy here. count_publishers() queries the
live DDS discovery data from inside this process instead.
"""
import time

import rclpy
from rclpy.node import Node

rclpy.init()
node = Node("fpms_graph")

# Discovery needs a moment; polling shows whether it arrives late rather than
# never, which distinguishes a slow-discovery problem from an absent publisher.
for i in range(6):
    time.sleep(2.0)
    rclpy.spin_once(node, timeout_sec=0.2)
    print(f"t={2 * (i + 1):2d}s  "
          f"odom_raw pubs={node.count_publishers('/odom_raw')} "
          f"imu pubs={node.count_publishers('/imu')} "
          f"battery pubs={node.count_publishers('/battery')} "
          f"cmd_vel subs={node.count_subscribers('/cmd_vel')}")

print("\n--- full topic list as this process sees it ---")
for name, types in sorted(node.get_topic_names_and_types()):
    n_pub = node.count_publishers(name)
    n_sub = node.count_subscribers(name)
    print(f"  {name:24s} {str(types):46s} pub={n_pub} sub={n_sub}")

print("\n--- nodes as this process sees them ---")
for n, ns in sorted(node.get_node_names_and_namespaces()):
    print(f"  {ns}{n}")

node.destroy_node()
rclpy.shutdown()
