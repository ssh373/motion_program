import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from booster_msgs.msg import RpcReqMsg
import json


def f_or_none(v):
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None
    

class LocoBridge(Node):

    def __init__(self):
        super().__init__('loco_bridge')

        self.sub = self.create_subscription(RpcReqMsg, 'LocoApiTopicReq', self.callback, 10)
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_head = self.create_publisher(Twist, 'head_cmd', 10)


        self.kMoveApiId = 2001
        self.kRotateHead = 2004
        self.kGetUp = 2008 # 추후 모션 학습이 완료되면 ... 

        # 마지막 정상 head 값 유지용
        self._last_pitch = 0.0
        self._last_yaw = 0.0
        self._have_head = False

    def callback(self, msg):
        try:
            header = json.loads(msg.header)
            api_id = header.get("api_id", None)
            if api_id is None:
                return

            body = json.loads(msg.body) if msg.body else {}

            if api_id == self.kMoveApiId:
                twist = Twist()
                twist.linear.x  = float(body.get("vx", 0.0))
                twist.linear.y  = float(body.get("vy", 0.0))
                twist.angular.z = float(body.get("vyaw", 0.0))
                self.pub.publish(twist)
                return
            
            if api_id == self.kRotateHead:
                pitch = f_or_none(body.get("pitch"))
                yaw   = f_or_none(body.get("yaw"))

                # 둘 다 None이면 아무것도 안 보냄
                if pitch is None and yaw is None:
                    return

                # None인 값은 유지
                if pitch is None:
                    pitch = self._last_pitch
                if yaw is None:
                    yaw = self._last_yaw

                self._last_pitch = pitch
                self._last_yaw = yaw
                self._have_head = True

                head = Twist()
                head.angular.y = pitch
                head.angular.z = yaw
                self.pub_head.publish(head)
                return

        except Exception as e:
            self.get_logger().warn(f"JSON parse error: {e}")

def main():
    rclpy.init()
    node = LocoBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
