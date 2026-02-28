import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from booster_msgs.msg import RpcReqMsg
from sensor_msgs.msg import Joy
import json
import time


def f_or_none(v):
    if v is None:
        return None
    try:
        return float(v)
    except (TypeError, ValueError):
        return None
    
def apply_deadzone(x: float, dz: float) -> float:
    return 0.0 if abs(x) < dz else x
    

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

        # -- joystick
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.joy_hold_sec = 0.3

        # params 
        self.deadzone = 0.05
        self.max_vx = 1.1         # m/s
        self.max_vy = 1.0          # m/s 
        self.max_wz = 1.2          # rad/s
        self.max_head_pitch = 0.75 # rad
        self.min_head_pitch = -0.30 # rad
        self.max_head_yaw = 1.0    # rad
        self.joy_timeout_sec = 0.25

        self.axis_lx = 0   # left stick left/right
        self.axis_ly = 1   # left stick up/down
        self.axis_rx = 3   # right stick left/right
        self.axis_ry = 4   # right stick up/down

        self.axis_dpad_lr = 6  # head yaw
        self.axis_dpad_ud = 7  # head pitch
        
        self._joy_last_time = 0.0
        self._joy_seen = False

        # timeout watchdog (takeover 중 조이스틱 끊기면 정지)
        self._joy_prev_time = None
        self.timer = self.create_timer(0.05, self._watchdog)

    def callback(self, msg):
        if self._joy_seen and (time.time() - self._joy_last_time) < self.joy_hold_sec:
            return
        
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
    
    # --- Joy callback 
    def joy_callback(self, msg: Joy):
        self._joy_seen = True
        self._joy_last_time = time.time()

        # axes 읽기 (범위 -1~1)
        def ax(i):
            return float(msg.axes[i]) if 0 <= i < len(msg.axes) else 0.0

        a0 = apply_deadzone(ax(0), self.deadzone)
        a1 = apply_deadzone(ax(1), self.deadzone)
        a4 = apply_deadzone(ax(4), self.deadzone)
        a3 = apply_deadzone(ax(3), self.deadzone)

        a_fwd = a1 if abs(a1) > abs(a4) else a4

        vx = (a_fwd) * self.max_vx
        vy = (a0) * self.max_vy
        wz = (a3) * self.max_wz

        # head
        d6 = ax(self.axis_dpad_lr)
        d7 = ax(self.axis_dpad_ud)
        d_lr = 0.0 if abs(d6) < 0.5 else (1.0 if d6 > 0 else -1.0) # -1/0/1 형태니까!
        d_ud = 0.0 if abs(d7) < 0.5 else (1.0 if d7 > 0 else -1.0)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.pub.publish(twist)

        # dt 계산
        now = time.time()
        if not hasattr(self, "_joy_prev_time") or self._joy_prev_time is None:
            dt = 0.02
        else:
            dt = max(1e-3, now - self._joy_prev_time)
        self._joy_prev_time = now

        # 고개 돌리는 속도
        head_yaw_speed   = 1.0   # rad/s
        head_pitch_speed = 1.0   # rad/s

        # 누르는 동안 목표각 계속 이동
        self._last_yaw   += d_lr * head_yaw_speed * dt
        self._last_pitch += d_ud * head_pitch_speed * dt

        # max 범위 제한
        self._last_yaw   = max(-self.max_head_yaw,   min(self.max_head_yaw,   self._last_yaw))
        self._last_pitch = max(self.min_head_pitch, min(self.max_head_pitch, self._last_pitch))

        head = Twist()
        head.angular.y = self._last_pitch  # pitch 목표각
        head.angular.z = self._last_yaw    # yaw 목표각
        self.pub_head.publish(head)

    def _watchdog(self):
        # joy 없으면 암것두 안하게
        if not self._joy_seen:
            return

        if (time.time() - self._joy_last_time) > self.joy_timeout_sec:
            stop = Twist()
            self.pub.publish(stop)
            self.get_logger().warn("Joy timeout -> stop cmd_vel")

def main():
    rclpy.init()
    node = LocoBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
