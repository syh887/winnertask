#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32  # 发布横向偏差

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        
        # 1. 创建CV桥接器
        self.bridge = CvBridge()
        
        # 2. 订阅摄像头图像话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.image_callback,
            10)
        
        # 3. 发布处理后的图像（可视化用）
        self.image_pub = self.create_publisher(Image, '/racecar/camera/image_processed', 10)
        
        # 4. 发布横向偏差（给PID控制器用）
        self.error_pub = self.create_publisher(Float32, '/racecar/track_error', 10)
        
        # 5. 白色赛道的HSV范围（亮蓝色底板上检测白色）
        # 白色的HSV范围（亮度高，饱和度低）
        self.lower_white = np.array([0, 0, 180])    # 降低亮度阈值，适应不同光照
        self.upper_white = np.array([180, 100, 255]) # 饱和度要低，亮度要高
        
        # 也可以尝试这个更宽松的范围（如果上面的检测不到）
        # self.lower_white = np.array([0, 0, 150])
        # self.upper_white = np.array([180, 80, 255])
        
        # 裁剪参数（只处理下半部分）
        self.crop_height = 200  # 从底部向上取200像素
        
        self.get_logger().info("白色赛道检测节点已启动！")

    def image_callback(self, msg):
        try:
            # 步骤1：ROS图像转OpenCV图像（BGR格式）
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width = cv_image.shape[:2]

            # 步骤2：裁剪图像（只保留下半部分，聚焦赛道）
            crop_img = cv_image[height-self.crop_height:height, 0:width]

            # 步骤3：HSV颜色分割，检测白色赛道
            # 转HSV颜色空间
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            
            # 生成白色掩码
            mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
            
            # 形态学操作去噪
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 开运算去噪点
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算填充空洞

            # 可选：对亮蓝色底板做反向掩码，增强白色检测
            # 蓝色底板的HSV范围（如果需要增强效果）
            # lower_blue = np.array([100, 100, 50])
            # upper_blue = np.array([130, 255, 255])
            # blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
            # mask = cv2.bitwise_and(mask, cv2.bitwise_not(blue_mask))

            # 步骤4：提取赛道轮廓，拟合中线
            contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # 找到最大轮廓（赛道区域）
                c = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(c)
                
                if area > 500:  # 只处理足够大的轮廓（排除小噪点）
                    # 计算轮廓的矩
                    M = cv2.moments(c)
                    
                    if M["m00"] != 0:
                        # 计算轮廓中心（赛道中线x坐标）
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # 步骤5：计算横向偏差（图像中心 - 赛道中线x）
                        # 偏差为正：小车偏左；偏差为负：小车偏右
                        image_center = width / 2
                        error = (image_center - cx) / (width / 2)  # 归一化偏差（-1 ~ +1）
                        
                        # 发布偏差值
                        error_msg = Float32()
                        error_msg.data = error
                        self.error_pub.publish(error_msg)
                        
                        # 步骤6：可视化
                        # 画赛道轮廓（绿色）
                        cv2.drawContours(crop_img, [c], -1, (0,255,0), 2)
                        # 画赛道中线（红色）
                        cv2.line(crop_img, (cx, cy-20), (cx, cy+20), (0,0,255), 3)
                        # 画图像中心线（蓝色）
                        cv2.line(crop_img, (int(image_center), 0), (int(image_center), self.crop_height), (255,0,0), 2)
                        # 显示面积和偏差
                        cv2.putText(crop_img, f"Error: {error:.2f}", (10,30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)  # 黄色文字
                        cv2.putText(crop_img, f"Area: {int(area)}", (10,60), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
                        
                        self.get_logger().info(f"检测到白色赛道，面积: {int(area)}, 偏差: {error:.2f}")
                    else:
                        self._publish_no_track(crop_img, "Invalid contour")
                else:
                    self._publish_no_track(crop_img, f"Area too small: {int(area)}")
            else:
                self._publish_no_track(crop_img, "No white track detected")

            # 发布处理后的图像（供可视化）
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(crop_img, "bgr8"))

        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败：{e}")
    
    def _publish_no_track(self, img, message):
        """没检测到赛道时发布0偏差和提示信息"""
        error_msg = Float32()
        error_msg.data = 0.0
        self.error_pub.publish(error_msg)
        
        cv2.putText(img, message, (10,30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        self.get_logger().warn(message)

def main(args=None):
    rclpy.init(args=args)
    processor = ImageProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
