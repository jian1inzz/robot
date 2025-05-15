#!/usr/bin/env python3
# yolo_cmd_listener_v2.py  ──離散 + 連續角度 + 雷達安全停車
# ----------------------------------------------------------
import math, rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ----------- 參數 ----------
FORWARD_V   = 0.05     # m/s 最大直行速度
ANG_KP      = 0.4      # 角度誤差 → 角速度比例增益
SAFE_DIST   = 0.15     # m  前 ±15° 小於此距離就停
ANGLE_GATE  = math.radians(3)   # 誤差小於 3° 視為已對準

class YoloCmdListener(Node):
    def __init__(self):
        super().__init__('yolo_cmd_listener')

        self.cmd_discrete = "stop"   # 最新 L/R/F 指令
        self.angle_deg    = None     # 連續角度 (deg)
        self.front_dist   = float('inf')

        self.create_subscription(String,  '/yolo_cmd',            self.cb_cmd,   10)
        self.create_subscription(Float32, '/yolo_angle',          self.cb_angle, 10)
        self.create_subscription(LaserScan, '/filtered_scan',     self.cb_scan,  10)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("✅ yolo_cmd_listener v2 啟動")

    def cb_cmd(self, msg: String):
        self.cmd_discrete = msg.data.strip().lower()

        # ✅ stop 指令 → 清除角度資料
        if self.cmd_discrete == "stop":
            self.angle_deg = None

    def cb_angle(self, msg: Float32):
        self.angle_deg = msg.data     # -30 ° ~ +30 °

    def cb_scan(self, scan: LaserScan):
        inc = scan.angle_increment
        n15 = int(math.radians(15) / inc)
        self.front_dist = min(scan.ranges[:n15] + scan.ranges[-n15:])

    def control_loop(self):
        twist = Twist()

        if self.front_dist < SAFE_DIST:
            self.get_logger().debug("⚠️ 前方障礙過近，停車")
            self.pub_vel.publish(twist)
            return

        if self.angle_deg is not None:
            ang_err = math.radians(self.angle_deg)
            ang_abs = abs(ang_err)

            lin_v = FORWARD_V * max(0.3, 1.0 - ang_abs * 2.0)
            ang_v = ANG_KP * ang_err
            if ang_abs < ANGLE_GATE:
                ang_v = 0.0

            twist.linear.x  = lin_v
            twist.angular.z = ang_v
            self.get_logger().info(f"🎯 角度: {self.angle_deg:+.1f}° → 線速: {lin_v:.2f}, 角速: {ang_v:.2f}")
            self.pub_vel.publish(twist)
            return

        if self.cmd_discrete == "left":
            twist.linear.x  = FORWARD_V * 0.5
            twist.angular.z =  0.10
        elif self.cmd_discrete == "right":
            twist.linear.x  = FORWARD_V * 0.5
            twist.angular.z = -0.10
        elif self.cmd_discrete == "left_fast":
            twist.linear.x  = FORWARD_V * 0.5
            twist.angular.z =  0.08
        elif self.cmd_discrete == "right_fast":
            twist.linear.x  = FORWARD_V * 0.5
            twist.angular.z = -0.08
        elif self.cmd_discrete == "center":
            twist.linear.x  = FORWARD_V
        elif self.cmd_discrete == "forward":
            twist.linear.x  = FORWARD_V
        else:
            pass  # stop 或未知

        self.pub_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = YoloCmdListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手動中斷")
    finally:
        node.pub_vel.publish(Twist())  # 離開前停車
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
