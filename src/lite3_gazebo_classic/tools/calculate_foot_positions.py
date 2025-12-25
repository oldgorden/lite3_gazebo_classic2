#!/usr/bin/env python3
"""
计算基于URDF几何结构和关节角度的脚部位置
"""

import numpy as np
import math

def calculate_foot_position(leg_offset_x, leg_offset_y, thigh_offset, 
                           hip_angle_x, hip_angle_y, knee_angle, mirror=1):
    """
    计算脚部相对于躯干的位置
    
    参数:
    - leg_offset_x, leg_offset_y: 髋关节相对于躯干的位置
    - thigh_offset: 髋关节Y关节的偏移
    - hip_angle_x, hip_angle_y, knee_angle: 关节角度(弧度)
    - mirror: 1 表示左侧，-1 表示右侧
    
    返回:
    - foot_position: 脚部相对于躯干的位置 (x, y, z)
    """
    
    # 从URDF中获取的几何参数
    thigh_length = 0.20  # 大腿长度 (HipY到Knee)
    shank_length = 0.21  # 小腿长度 (Knee到Ankle)
    
    # 髋关节位置相对于躯干
    hip_position = np.array([leg_offset_x, leg_offset_y * mirror, 0.0])
    
    # 计算大腿向量 (HipY关节)
    # 注意：HipY关节绕y轴旋转，偏移在y方向
    thigh_vector_local = np.array([0.0, thigh_offset * mirror, 0.0])
    
    # 应用HipX旋转 (绕x轴)
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(hip_angle_x), -math.sin(hip_angle_x)],
                   [0, math.sin(hip_angle_x), math.cos(hip_angle_x)]])
    
    # 应用HipY旋转 (绕y轴)
    Ry = np.array([[math.cos(hip_angle_y), 0, math.sin(hip_angle_y)],
                   [0, 1, 0],
                   [-math.sin(hip_angle_y), 0, math.cos(hip_angle_y)]])
    
    # 大腿向量在HipX和HipY旋转后的位置
    thigh_vector = Rx @ Ry @ thigh_vector_local
    
    # 大腿段向量 (从HipY到Knee)
    thigh_segment_local = np.array([0.0, 0.0, -thigh_length])
    thigh_segment = Rx @ Ry @ thigh_segment_local
    
    # 小腿段向量 (从Knee到Ankle)
    # 应用Knee旋转 (绕y轴)
    Rk = np.array([[math.cos(knee_angle), 0, math.sin(knee_angle)],
                   [0, 1, 0],
                   [-math.sin(knee_angle), 0, math.cos(knee_angle)]])
    
    shank_segment_local = np.array([0.0, 0.0, -shank_length])
    shank_segment = Rx @ Ry @ Rk @ shank_segment_local
    
    # 总脚部位置
    foot_position = hip_position + thigh_vector + thigh_segment + shank_segment
    
    return foot_position

def main():
    # 从URDF/xacro文件中获取的几何参数
    leg_offset_x = 0.1745
    leg_offset_y = 0.062
    thigh_offset = 0.0955
    
    # 从robot_control.yaml中获取的stand_pos关节角度
    # stand_pos: [0.0, -0.732, 1.361] 对于每个腿 (HipX, HipY, Knee)
    hip_angle_x = 0.0        # rad
    hip_angle_y = -0.732     # rad
    knee_angle = 1.361       # rad
    
    print("基于URDF几何结构和stand_pos关节角度计算脚部位置")
    print("=" * 60)
    print(f"几何参数:")
    print(f"  leg_offset_x: {leg_offset_x} m")
    print(f"  leg_offset_y: {leg_offset_y} m")
    print(f"  thigh_offset: {thigh_offset} m")
    print(f"  thigh_length: 0.20 m")
    print(f"  shank_length: 0.21 m")
    print(f"\n关节角度 (stand_pos):")
    print(f"  HipX: {hip_angle_x:.3f} rad ({hip_angle_x*180/math.pi:.1f} deg)")
    print(f"  HipY: {hip_angle_y:.3f} rad ({hip_angle_y*180/math.pi:.1f} deg)")
    print(f"  Knee: {knee_angle:.3f} rad ({knee_angle*180/math.pi:.1f} deg)")
    
    print("\n" + "=" * 60)
    print("计算脚部位置 (相对于躯干):")
    
    # 计算四个腿的位置
    legs = [
        ("FR", -1),  # 前右，mirror=-1
        ("FL", 1),   # 前左，mirror=1
        ("HR", -1),  # 后右，mirror=-1
        ("HL", 1)    # 后左，mirror=1
    ]
    
    # 注意：前后腿的leg_offset_x符号不同
    positions = []
    for leg_name, mirror in legs:
        # 前后腿的leg_offset_x符号不同
        if leg_name.startswith('F'):  # 前腿
            x_offset = leg_offset_x
        else:  # 后腿
            x_offset = -leg_offset_x
            
        pos = calculate_foot_position(
            x_offset, leg_offset_y, thigh_offset,
            hip_angle_x, hip_angle_y, knee_angle,
            mirror
        )
        positions.append((leg_name, pos))
        print(f"\n{leg_name}:")
        print(f"  x: {pos[0]:.6f} m")
        print(f"  y: {pos[1]:.6f} m")
        print(f"  z: {pos[2]:.6f} m")
    
    print("\n" + "=" * 60)
    print("控制器当前硬编码值 (feet_pos_normal_stand_):")
    print("x: [0.1881, 0.1881, -0.1881, -0.1881]")
    print("y: [-0.1300, 0.1300, -0.1300, 0.1300]")
    print("z: [-0.3200, -0.3200, -0.3200, -0.3200]")
    
    print("\n" + "=" * 60)
    print("建议更新值 (基于计算):")
    
    # 格式化输出为控制器代码格式
    x_vals = [pos[0] for _, pos in positions]
    y_vals = [pos[1] for _, pos in positions]
    z_vals = [pos[2] for _, pos in positions]
    
    print("feet_pos_normal_stand_ << ", end="")
    print(f"{x_vals[0]:.6f}, {x_vals[1]:.6f}, {x_vals[2]:.6f}, {x_vals[3]:.6f}, ", end="")
    print(f"{y_vals[0]:.6f}, {y_vals[1]:.6f}, {y_vals[2]:.6f}, {y_vals[3]:.6f}, ", end="")
    print(f"{z_vals[0]:.6f}, {z_vals[1]:.6f}, {z_vals[2]:.6f}, {z_vals[3]:.6f};")

if __name__ == "__main__":
    main()
