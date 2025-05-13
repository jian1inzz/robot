#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32          # ★★ 新增 Float32
from cv_bridge import CvBridge

import asyncio, websockets, base64, cv2, threading, json   # ★★ 新增 json

class CameraWebSocketNode(Node):
    def __init__(self):
        super().__init__('camera_ws_node')

        # -------- ROS publishers --------
        self.img_pub  = self.create_publisher(Image,   '/image_raw', 10)
        self.cmd_pub  = self.create_publisher(String,  '/yolo_cmd',  10)
        self.ang_pub  = self.create_publisher(Float32, '/yolo_angle', 10)   # ★★ 角度 topic

        self.bridge = CvBridge()

        # -------- 打開相機 --------
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("❌ 相機開啟失敗")
            return
        self.get_logger().info("✅ 相機成功打開")

        # 10 Hz 定時拍照 + 發影像 + WebSocket 傳輸
        self.create_timer(0.1, self.timer_cb)

        # -------- WebSocket --------
        self.ws_uri = "ws://172.20.10.2:8765"   # ⚠️ 換成桌機 IP
        self.ws   = None
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        asyncio.run_coroutine_threadsafe(self.connect_ws(), self.loop)

    # ================== Timer ==================
    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("⚠️ 無法讀取影像")
            return

        # 1) 發 ROS 影像
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        # 2) 傳 WebSocket
        if self.ws:
            asyncio.run_coroutine_threadsafe(self.ws_send(frame), self.loop)

    # ================== WebSocket 連線 ==================
    async def connect_ws(self):
        try:
            self.ws = await websockets.connect(self.ws_uri)
            self.get_logger().info("✅ 已連上 WebSocket Server")
            asyncio.ensure_future(self.ws_receive())     # 啟動接收
        except Exception as e:
            self.get_logger().error(f"❌ WebSocket 連線失敗: {e}")
            self.ws = None

    async def ws_send(self, frame):
        try:
            _, buf = cv2.imencode('.jpg', frame)
            await self.ws.send(base64.b64encode(buf).decode())
        except Exception as e:
            self.get_logger().warning(f"⚠️ 傳送影像失敗: {e}")
            self.ws = None

    async def ws_receive(self):
        try:
            async for message in self.ws:
                # -------- ★★ 同時支援 JSON / 純字串 --------
                try:
                    data = json.loads(message)           # 嘗試解析 JSON
                    cmd   = data.get("cmd")
                    angle = data.get("angle_deg")
                except json.JSONDecodeError:
                    cmd   = message.strip().lower()
                    angle = None
                # ------------ 指令處理 ------------
                if cmd in ("left", "left_fast", "center",
                           "right", "right_fast", "forward", "stop"):
                    str_msg = String(); str_msg.data = cmd
                    self.cmd_pub.publish(str_msg)
                    self.get_logger().info(f"📨 cmd={cmd}")

                    # 若有角度，一併發布
                    if angle is not None:
                        ang_msg = Float32(); ang_msg.data = float(angle)
                        self.ang_pub.publish(ang_msg)
                        self.get_logger().info(f"   angle={angle:+.1f}°")
                else:
                    self.get_logger().warning(f"⚠️ 未知指令: {message}")
        except Exception as e:
            self.get_logger().error(f"❌ 接收失敗: {e}")
            self.ws = None

def main(args=None):
    rclpy.init(args=args)
    node = CameraWebSocketNode()
    rclpy.spin(node)
    if node.cap.isOpened():
        node.cap.release()
    rclpy.shutdown()

if __name__ == '__main__':
    main()