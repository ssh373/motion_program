#!/usr/bin/env python3
import argparse
import numpy as np
import torch

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from booster_interface.msg import LowState


DEFAULT_JOINT_NAMES = [
    "AAHead_Yaw", "Head_Pitch",
    "ALeft_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
    "ARight_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
    "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw",
    "Left_Knee_Pitch", "Left_Ankle_Pitch", "Left_Ankle_Roll",
    "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw",
    "Right_Knee_Pitch", "Right_Ankle_Pitch", "Right_Ankle_Roll",
]

LEG_JOINT_ORDER = [
    "Left_Hip_Pitch", "Right_Hip_Pitch",
    "Left_Hip_Roll",  "Right_Hip_Roll",
    "Left_Hip_Yaw",   "Right_Hip_Yaw",
    "Left_Knee_Pitch","Right_Knee_Pitch",
    "Left_Ankle_Pitch","Right_Ankle_Pitch",
    "Left_Ankle_Roll","Right_Ankle_Roll",
]

SCALE_BY_TYPE = {
    "Hip_Pitch": 0.09375,
    "Hip_Roll":  0.109375,
    "Hip_Yaw":   0.0625,
    "Knee_Pitch":0.125,
    "Ankle_Pitch": 0.166666,
    "Ankle_Roll":  0.166666,
}

class WalkingPolicyNode(Node):
    def __init__(self, model_path: str, device: str, rate_hz: float,
                 out_topic: str, joint_state_topic: str, imu_topic: str, cmd_vel_topic: str,
                 clip_action: float, max_delta_per_step: float):

        super().__init__("walking_policy_node")

        self.device = torch.device(device)
        self.rate_hz = float(rate_hz)
        self.dt = 1.0 / self.rate_hz

        self.clip_action = float(clip_action)
        self.max_delta_per_step = float(max_delta_per_step)

        self.model = torch.jit.load(model_path, map_location=self.device).to(self.device)
        self.model.eval()
        self.get_logger().info(f"Loaded model: {model_path}")

        self.sub_cmd = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.sub_js  = self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)
        self.sub_low = self.create_subscription(LowState, imu_topic, self._on_low_state, 10)
        self.pub_cmd = self.create_publisher(JointState, out_topic, 10)

        self.name_to_idx = {n: i for i, n in enumerate(DEFAULT_JOINT_NAMES)}
        self.leg_idx = [self.name_to_idx[n] for n in LEG_JOINT_ORDER]

        self.head_yaw_idx = self.name_to_idx["AAHead_Yaw"]
        self.head_pitch_idx = self.name_to_idx["Head_Pitch"]
        self._head_target = None

        self.sub_head = self.create_subscription(Twist, "head_cmd", self._on_head_cmd, 10)

        self.q  = np.zeros(22, dtype=np.float32)
        self.qd = np.zeros(22, dtype=np.float32)
        self.latest_cmd = np.zeros(3, dtype=np.float32)

        self.gyro = np.zeros(3, dtype=np.float32)
        self.proj_g = np.array([0, 0, -1], dtype=np.float32)

        self.have_js = False
        self.have_imu = False

        self.q_default = np.zeros(22, dtype=np.float32)
        for i, n in enumerate(DEFAULT_JOINT_NAMES):
            if "Hip_Pitch" in n:
                self.q_default[i] = -0.2
            elif "Knee_Pitch" in n:
                self.q_default[i] = 0.4
            elif "Ankle_Pitch" in n:
                self.q_default[i] = -0.2

        self.q_leg_default = self.q_default[self.leg_idx].copy()
        self.upper_hold = None
        self.last_action = np.zeros(len(self.leg_idx), dtype=np.float32)

        self.timer = self.create_timer(self.dt, self._tick)

    def _on_cmd_vel(self, msg: Twist):
        self.latest_cmd[0] = float(msg.linear.x)
        self.latest_cmd[1] = float(msg.linear.y)
        self.latest_cmd[2] = float(msg.angular.z)

    def _on_head_cmd(self, msg: Twist):
        self._head_target = (float(msg.angular.z), float(msg.angular.y))

    def _on_joint_state(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position)) if msg.name and msg.position else {}
        vel = dict(zip(msg.name, msg.velocity)) if msg.name and msg.velocity else {}

        for n, i in self.name_to_idx.items():
            if n in pos:
                self.q[i] = float(pos[n])
            if n in vel:
                self.qd[i] = float(vel[n])

        self.have_js = True

        if self.upper_hold is None:
            self.upper_hold = self.q.copy()

    def _on_low_state(self, msg: LowState):
        # gyro
        self.gyro[:] = [
            float(msg.imu_state.gyro[0]),
            float(msg.imu_state.gyro[1]),
            float(msg.imu_state.gyro[2]),
        ]

        # rpy -> proj_g
        roll  = float(msg.imu_state.rpy[0])
        pitch = float(msg.imu_state.rpy[1])
        yaw   = float(msg.imu_state.rpy[2])

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        # R = Rz(yaw)*Ry(pitch)*Rx(roll)
        R_wb = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr],
        ], dtype=np.float32)

        g_w = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.proj_g = (R_wb.T @ g_w).astype(np.float32)

        self.have_imu = True

    def _build_obs(self) -> np.ndarray:
        q_leg = self.q[self.leg_idx]
        qd_leg = self.qd[self.leg_idx]

        joint_pos_rel = (q_leg - self.q_leg_default).astype(np.float32)
        gyro = np.clip(self.gyro.astype(np.float32), -5.0, 5.0)
        joint_vel_rel = np.clip(qd_leg.astype(np.float32), -10.0, 10.0)

        return np.concatenate([
            gyro,
            self.proj_g.astype(np.float32),
            self.latest_cmd,
            joint_pos_rel,
            joint_vel_rel,
            self.last_action.astype(np.float32),
        ], axis=0).astype(np.float32)

    @torch.no_grad()
    def _infer_action(self, obs: np.ndarray) -> np.ndarray:
        x = torch.from_numpy(obs).to(self.device).unsqueeze(0)
        y = self.model(x)
        if isinstance(y, (tuple, list)):
            y = y[0]
        a = y.squeeze(0).cpu().numpy().astype(np.float32)

        if self.clip_action > 0:
            a = np.clip(a, -self.clip_action, self.clip_action)

        return a

    def _action_to_targets(self, action: np.ndarray) -> np.ndarray:
        scales = np.array([SCALE_BY_TYPE[k.split("_",1)[1]] for k in LEG_JOINT_ORDER], dtype=np.float32)
        q_leg_target = self.q_leg_default + scales * action

        if self.max_delta_per_step > 0:
            cur = self.q[self.leg_idx]
            delta = np.clip(q_leg_target - cur, -self.max_delta_per_step, self.max_delta_per_step)
            q_leg_target = cur + delta

        return q_leg_target.astype(np.float32)

    def _publish(self, q_leg_target: np.ndarray):
        q_target_22 = self.upper_hold.copy() if self.upper_hold is not None else self.q.copy()
        q_target_22[self.leg_idx] = q_leg_target

        if self._head_target is not None:
            yaw, pitch = self._head_target
            q_target_22[self.head_yaw_idx] = yaw
            q_target_22[self.head_pitch_idx] = pitch

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = DEFAULT_JOINT_NAMES
        msg.position = [float(v) for v in q_target_22.tolist()]
        self.pub_cmd.publish(msg)

    def _tick(self):
        if not (self.have_js and self.have_imu):
            return

        obs = self._build_obs()
        action = self._infer_action(obs)
        q_leg_target = self._action_to_targets(action)
        self._publish(q_leg_target)
        self.last_action[:] = action


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--device", default="cpu", choices=["cpu", "cuda"])
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--clip_action", type=float, default=1.0)
    ap.add_argument("--max_delta_per_step", type=float, default=0.05)

    ap.add_argument("--out_topic", default="joint_cmd")
    ap.add_argument("--joint_states", default="joint_states")
    ap.add_argument("--imu", default="low_state")
    ap.add_argument("--cmd_vel", default="cmd_vel")

    args, unknown = ap.parse_known_args()
    
    rclpy.init()
    node = WalkingPolicyNode(
        model_path=args.model,
        device=args.device,
        rate_hz=args.rate,
        out_topic=args.out_topic,
        joint_state_topic=args.joint_states,
        imu_topic=args.imu,
        cmd_vel_topic=args.cmd_vel,
        clip_action=args.clip_action,
        max_delta_per_step=args.max_delta_per_step,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
