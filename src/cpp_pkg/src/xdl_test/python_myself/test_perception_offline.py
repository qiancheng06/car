#!/usr/bin/env python3
"""ROS-free synthetic checks for pure_lidar_b_node perception helpers."""
import importlib.util
import math
import os
import sys
import types


def install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")
    class Node: pass
    class Q: pass
    rclpy_node.Node = Node
    rclpy_qos.HistoryPolicy = Q
    rclpy_qos.QoSProfile = Q
    rclpy_qos.ReliabilityPolicy = Q
    rclpy.node = rclpy_node
    for name, classes in {
        "geometry_msgs.msg": ["Twist"],
        "nav_msgs.msg": ["Odometry"],
        "sensor_msgs.msg": ["Imu", "LaserScan"],
        "std_msgs.msg": ["Bool", "Float32", "Float32MultiArray", "String"],
    }.items():
        package, _ = name.split(".")
        package_module = sys.modules.setdefault(package, types.ModuleType(package))
        module = types.ModuleType(name)
        for class_name in classes:
            setattr(module, class_name, type(class_name, (), {}))
        package_module.msg = module
        sys.modules[name] = module
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos


def load_module():
    install_ros_stubs()
    spec = importlib.util.spec_from_file_location(
        "pure_lidar", os.path.join(os.path.dirname(__file__), "pure_lidar_b_node.py")
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_node(module):
    node = object.__new__(module.PureLidarBNode)
    values = {
        "min_range": 0.15, "max_range": 8.0, "max_lateral": 2.6,
        "boundary_max_lateral": 1.65,
        "max_forward": 5.0, "front_width": 0.38,
        "front_obstacle_min_points": 3, "cluster_gap": 0.28,
        "max_index_gap": 2, "min_cluster_points": 2,
        "max_cluster_points": 60, "max_cluster_radius": 0.60,
        "wall_segment_points": 12, "use_bucket_edges": True,
        "bucket_window": 10, "bucket_min_support": 6,
        "bucket_range_tolerance": 0.20, "bucket_edge_jump": 0.55,
        "candidate_merge_distance": 0.24, "side_deadband": 0.12,
        "side_min_y": 0.30, "side_max_y": 3.80,
        "max_boundary_points_per_side": 5, "boundary_fit_residual": 0.36,
        "pair_y_tolerance": 0.85, "min_corridor_width": 0.65,
        "max_corridor_width": 2.35, "track_width": 1.50,
        "track_width_tolerance": 0.85, "near_lookahead": 0.90,
        "far_lookahead": 2.30, "perception_smoothing": 0.0,
        "filtered_center_error": None, "filtered_heading_error": None,
        "latest_boundary_points": [],
    }
    node.__dict__.update(values)
    return node


class Scan:
    pass


def circle_scan(circles, samples=1080):
    scan = Scan()
    scan.angle_min = -math.pi
    scan.angle_increment = 2.0 * math.pi / samples
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = []
    for index in range(samples):
        angle = scan.angle_min + index * scan.angle_increment
        dx, dy = math.sin(angle), math.cos(angle)
        closest = float("inf")
        for cx, cy, radius in circles:
            projection = cx * dx + cy * dy
            discriminant = projection * projection - (cx * cx + cy * cy - radius * radius)
            if projection <= 0.0 or discriminant < 0.0:
                continue
            distance = projection - math.sqrt(discriminant)
            if distance > scan.range_min:
                closest = min(closest, distance)
        scan.ranges.append(closest)
    return scan


def main():
    module = load_module()
    node = make_node(module)

    separated = [
        (-0.70, 1.00, 1.0, 10), (-0.69, 1.02, 1.0, 11),
        (-0.68, 1.04, 1.0, 20), (-0.67, 1.06, 1.0, 21),
    ]
    clusters = node._cluster(separated)
    assert len(clusters) == 2, f"invalid scan-index break merged objects: {len(clusters)}"

    straight = [
        (-0.75, 0.75, 0.11), (0.75, 0.75, 0.11),
        (-0.75, 1.55, 0.11), (0.75, 1.55, 0.11),
        (-0.75, 2.45, 0.11), (0.75, 2.45, 0.11),
        (-2.10, 1.20, 0.10),  # left-side outlier
    ]
    corridor, front = node._estimate_corridor(circle_scan(straight))
    assert corridor.valid, corridor
    assert abs(corridor.near_center_x) < 0.18, corridor
    assert abs(corridor.heading_error) < 0.12, corridor
    assert front > 0.42, front

    node.filtered_center_error = None
    node.filtered_heading_error = None
    right_curve = [
        (-0.75, 0.75, 0.11), (0.75, 0.75, 0.11),
        (-0.62, 1.55, 0.11), (0.88, 1.55, 0.11),
        (-0.42, 2.45, 0.11), (1.08, 2.45, 0.11),
    ]
    curve, _ = node._estimate_corridor(circle_scan(right_curve))
    assert curve.valid, curve
    assert curve.heading_error > 0.03, curve
    print("PASS: index breaks, centered corridor, outlier rejection, right-curve sign")
    print("straight:", corridor)
    print("curve:", curve)


if __name__ == "__main__":
    main()
