#!/usr/bin/env python3
"""
ROS2 节点：订阅当前 domain 中所有话题，统计 Hz 与消息类型，并导出为 CSV。
"""

import csv
import importlib
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def get_message_class(type_str: str):
    """
    根据 ROS2 类型字符串（如 'std_msgs/msg/String'）动态加载并返回消息类。
    """
    parts = type_str.split('/')
    if len(parts) != 3:
        return None
    pkg, kind, type_name = parts[0], parts[1], parts[2]
    if kind != 'msg':
        return None
    try:
        module = importlib.import_module(f'{pkg}.{kind}')
        return getattr(module, type_name)
    except (ImportError, AttributeError):
        return None


class TopicGatherNode(Node):
    def __init__(self, measure_sec: float = 3.0, output_path: str = None):
        super().__init__('topic_gather_node')
        self.measure_sec = measure_sec
        if output_path is None:
            output_path = Path.home() / 'topic_gather_report.csv'
        self.output_path = Path(output_path)
        self.msg_count = 0
        self.sub = None

    def _cb(self, msg):
        self.msg_count += 1

    def get_all_topics_and_types(self):
        """获取当前 ROS domain 中所有话题名与类型。"""
        names_and_types = self.get_topic_names_and_types()
        out = []
        for name, type_list in names_and_types:
            for t in type_list:
                if '/msg/' in t:
                    out.append((name, t))
                    break
        return out

    def measure_hz(self, topic_name: str, type_str: str) -> float:
        """对单个话题订阅并测量 Hz，返回频率（无数据时返回 0.0）。"""
        msg_class = get_message_class(type_str)
        if msg_class is None:
            self.get_logger().warning(f'无法加载类型: {type_str}, 跳过话题 {topic_name}')
            return 0.0

        self.msg_count = 0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        try:
            self.sub = self.create_subscription(
                msg_class,
                topic_name,
                self._cb,
                qos,
            )
        except Exception as e:
            self.get_logger().warning(f'订阅失败 {topic_name}: {e}')
            return 0.0

        # 短暂等待建立连接
        time.sleep(0.5)
        deadline = time.monotonic() + self.measure_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(self.sub)
        self.sub = None

        elapsed = self.measure_sec
        return self.msg_count / elapsed if elapsed > 0 else 0.0

    def run(self):
        self.get_logger().info('正在发现当前 domain 中的话题...')
        topics = self.get_all_topics_and_types()
        if not topics:
            self.get_logger().warn('未发现任何消息类型话题。')
            self.write_csv([])
            return

        self.get_logger().info(f'发现 {len(topics)} 个话题，开始依次测量 Hz（每个约 {self.measure_sec}s）...')
        rows = []
        for i, (name, type_str) in enumerate(topics):
            hz = self.measure_hz(name, type_str)
            rows.append({
                'topic': name,
                'type': type_str,
                'hz': round(hz, 2),
            })
            self.get_logger().info(f'[{i+1}/{len(topics)}] {name} | {type_str} | {hz:.2f} Hz')

        self.write_csv(rows)
        self.get_logger().info(f'结果已写入: {self.output_path}')

    def write_csv(self, rows):
        """将结果写入 CSV。"""
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_path, 'w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=['topic', 'type', 'hz'])
            w.writeheader()
            w.writerows(rows)


def main(args=None):
    rclpy.init(args=args)
    import argparse
    parser = argparse.ArgumentParser(description='订阅所有话题并统计 Hz，导出 CSV')
    parser.add_argument('--duration', '-d', type=float, default=3.0,
                        help='每个话题测量时长（秒），默认 3.0')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='输出 CSV 路径，默认 ~/topic_gather_report.csv')
    parsed, _ = parser.parse_known_args()

    node = TopicGatherNode(measure_sec=parsed.duration, output_path=parsed.output)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
