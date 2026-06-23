import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class MotionGenerator(Node):

    def __init__(self):
        super().__init__('motion_generator')
        
        # 1. 建立一個 Publisher，負責把關節角度發送到 /joint_states 畫面上
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 2. 設定定時器：每 0.05 秒 (50毫秒) 執行一次控制迴圈 (符合講義要求)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 3. 定義 5 個在關節空間中的目標姿態點 (Waypoints)
        # 每組數值代表: [arm_0, arm_1, arm_2, gripper_l, gripper_r]
        self.waypoints = [
            [0.0,   0.0,   0.0,   0.0,   0.0],   # 姿態 1: 直立原點
            [1.57,  0.5,   0.5,  -0.03,  0.03],  # 姿態 2: 轉向一側並張開夾爪
            [-1.57, -0.5,  0.8,   0.0,   0.0],   # 姿態 3: 甩到另一側並閉合
            [0.0,   0.8,  -0.5,  -0.05,  0.05],  # 姿態 4: 往前伸低點
            [0.0,  -0.3,   0.3,   0.0,   0.0]    # 姿態 5: 稍微收回
        ]
        
        self.current_waypoint_index = 0
        self.next_waypoint_index = 1
        
        # 內插計步器 (用來在兩個姿態之間平滑移動)
        self.interpolation_steps = 40  # 兩個點之間花 40 步 (約 2 秒) 慢慢走
        self.current_step = 0
        
        # 目前機器人的實際關節角度
        self.current_positions = [0.0, 0.0, 0.0, 0.0, 0.0]

    def timer_callback(self):
        # 取得當前點與下一個點
        start_p = self.waypoints[self.current_waypoint_index]
        end_p = self.waypoints[self.next_waypoint_index]
        
        # 計算線性內插比例 (0.0 到 1.0)
        alpha = self.current_step / self.interpolation_steps
        
        # 平滑計算當前這一刻的關節角度 (講義要求的 Smoothly interpolate)
        for i in range(5):
            self.current_positions[i] = start_p[i] + alpha * (end_p[i] - start_p[i])
            
        # 前進一步
        self.current_step += 1
        
        # 如果走到目標點了，就換下一組目標點
        if self.current_step > self.interpolation_steps:
            self.current_step = 0
            self.current_waypoint_index = self.next_waypoint_index
            # 循環前進：如果到最後一個點，就回到第一個點 (符合講義 interpolate to the first point)
            self.next_waypoint_index = (self.next_waypoint_index + 1) % len(self.waypoints)

        # 4. 打包 ROS 2 的 JointState 訊息並發布出去
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 🚨 這裡的名稱一定要跟你在 Assignment 3 寫的 URDF 關節名稱完全一致！
        msg.name = ['arm_0_joint', 'arm_1_joint', 'arm_2_joint', 'gripper_l_joint', 'gripper_r_joint']
        msg.position = self.current_positions

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    motion_generator = MotionGenerator()
    rclpy.spin(motion_generator)
    motion_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
