import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # -------- 横向引导线检测节点 (Horizontal Line Detector) --------
        Node(
            package='horizontal_line_control',
            executable='horizontal_line_detector_node',
            name='horizontal_line_detector_node',
            output='screen',
            parameters=[{
                # 1. 基础话题配置 (与垂直节点解耦，使用 horizontal_line 命名空间)
                'binary_topic': '/line/binary_image',           # 订阅同一个二值化图像
                'line_topic': '/horizontal_line/line',          # 发布的横线三维点坐标
                'angle_topic': '/horizontal_line/angle_deg',    # 发布的横线角度
                'x_topic': '/horizontal_line/y_at_x_half',      # 旋转后的X，相当于原图的Y坐标
                'debug_topic': '/horizontal_line/debug_image',  # rqt 调试画面 (旋转了90度)

                # ==========================================
                # 2. 【核心控制】：旋转方向
                # True = 顺时针旋转90度 (看右边的横线)
                # False = 逆时针旋转90度 (看左边的横线)
                # ==========================================
                'rotate_clockwise': True,     

                # 3. 核心动态调参区 (继承巅峰版参数)
                'output_ema_alpha': 0.85,     # 平滑阻尼
                'max_abs_angle_deg': 40.0,    # 最大允许拟合偏角
                'border_margin_px': 15,       # 画面边缘裁剪像素

                # 4. 天际线角点 NaN 触发窗口
                'skyline_nan_min_y_ratio': 0.45,
                'skyline_nan_max_y_ratio': 0.85,

                # 5. 视觉与显示
                'publish_debug': True,
                'show_fps_overlay': True,
                'fps_ema_alpha': 0.2,

                # 6. 底层兼容参数
                'morph_open_ksize': 3,
                'morph_close_ksize': 5,
                'hough_threshold': 15,
                'hough_min_length': 30,
                'hough_max_gap': 40,
                'angle_penalty': 2.0
            }]
        )
    ])