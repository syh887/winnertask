#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # 1. PID参数（针对红色圆环赛道优化）
        self.Kp = 20.0    # 比例系数（提高，让转向更灵敏）
        self.Ki = 0.0    # 积分系数（先关掉，避免干扰）
        self.Kd = 0.3   # 微分系数（大幅降低，防止突变）
        
        # 2. 状态变量
        self.last_error = 0.0
        self.integral = 0.0
        self.sample_time = 0.01
        self.current_error = 0.0
        
        # 3. 订阅横向偏差话题
        self.error_sub = self.create_subscription(
            Float32,
            '/racecar/track_error',
            self.error_callback,
            10)
        
        # 4. 发布速度控制指令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 5. 定时器
        self.timer = self.create_timer(self.sample_time, self.pid_control_loop)
        
        # 6. 速度设置
        self.base_speed = 0.06 # 适中速度
        self.get_logger().info("PID控制器节点已启动！")

    def error_callback(self, msg):
        self.current_error = msg.data

    def pid_control_loop(self):
        try:
            error = self.current_error
            
            # 弯道检测和速度调整
            if abs(error) > 0.05:  # 大弯
                current_speed = 0.05
                kp_used = self.Kp * 1.5  # 弯道增强转向
            else:  # 直道
                current_speed = self.base_speed
                kp_used = self.Kp
            
            # PID计算（简化版，只使用P控制）
            P = kp_used * error
            
            # 微分项（加低通滤波，防止突变）
            D = self.Kd * (error - self.last_error)
            self.last_error = error
            
            # 计算转向
            steering = P + D
            # 转向角限幅
            steering = max(min(steering, 1.0), -1.0)
            
            # 发布控制指令
            cmd_vel = Twist()
            cmd_vel.linear.x = current_speed
            cmd_vel.angular.z = steering
            self.cmd_vel_pub.publish(cmd_vel)
            
            # 调试输出
            self.get_logger().info(f"偏差:{error:.2f} 转向:{steering:.2f} 速度:{current_speed:.2f}")
            
        except AttributeError:
            # 还没收到偏差，发零速度
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    
    # 使用try-finally确保正确关闭
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('正在停止小车...')
        # 发送停止命令
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        for _ in range(5):  # 多发几次确保收到
            controller.cmd_vel_pub.publish(stop_msg)
            rclpy.spin_once(controller, timeout_sec=0.1)
        controller.get_logger().info('小车已停止')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
