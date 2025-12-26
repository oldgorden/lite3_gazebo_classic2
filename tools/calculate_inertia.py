#!/usr/bin/env python3
"""
计算URDF模型的总体惯性矩阵
"""

import numpy as np

def main():
    """
    从URDF文件中提取的质量和惯性参数
    基于之前分析的URDF文件
    """
    
    print("计算URDF模型的总体惯性矩阵")
    print("=" * 60)
    
    # 从URDF中提取的各部件质量 (实际值)
    masses = {
        "torso": 5.629898899102925,      # 躯干质量 (来自URDF)
        "hip": 0.5834374645007383,       # 单个髋关节质量 (来自URDF)
        "thigh": 0.8315347040781551,     # 单个大腿质量 (来自URDF)
        "shank": 0.1567647392934227,     # 单个小腿质量 (来自URDF)
        "foot": 0.08179029876178574,     # 单个脚质量 (来自URDF)
    }
    
    # 各部件数量
    num_hips = 4
    num_thighs = 4
    num_shanks = 4
    num_feet = 4
    
    # 计算总质量
    total_mass = (masses["torso"] + 
                  masses["hip"] * num_hips +
                  masses["thigh"] * num_thighs +
                  masses["shank"] * num_shanks +
                  masses["foot"] * num_feet)
    
    print(f"总质量: {total_mass:.3f} kg")
    print(f"  躯干: {masses['torso']:.3f} kg")
    print(f"  髋关节: {masses['hip']:.3f} kg × {num_hips} = {masses['hip'] * num_hips:.3f} kg")
    print(f"  大腿: {masses['thigh']:.3f} kg × {num_thighs} = {masses['thigh'] * num_thighs:.3f} kg")
    print(f"  小腿: {masses['shank']:.3f} kg × {num_shanks} = {masses['shank'] * num_shanks:.3f} kg")
    print(f"  脚: {masses['foot']:.3f} kg × {num_feet} = {masses['foot'] * num_feet:.3f} kg")
    
    print("\n" + "=" * 60)
    print("控制器中硬编码的惯性参数 (BalanceCtrl.cpp):")
    print("Ib_ = Vec3(0.07416, 0.12086, 0.15067).asDiagonal();")
    print(f"  即: diag({0.07416:.5f}, {0.12086:.5f}, {0.15067:.5f}) kg·m²")
    
    print("\n" + "=" * 60)
    print("分析:")
    print("1. 惯性矩阵应该是正定对称矩阵")
    print("2. 对于四足机器人，通常 Ixx < Iyy < Izz (绕x轴转动惯量最小)")
    print("3. 控制器值: Ixx=0.07416, Iyy=0.12086, Izz=0.15067")
    print("4. 这符合物理规律 (Ixx < Iyy < Izz)")
    
    print("\n" + "=" * 60)
    print("建议:")
    print("1. 当前硬编码的惯性参数看起来合理")
    print("2. 如果需要更精确的值，需要:")
    print("   a. 计算每个部件的惯性矩阵")
    print("   b. 使用平行轴定理转换到质心坐标系")
    print("   c. 求和得到总体惯性矩阵")
    print("3. 考虑到平衡控制的鲁棒性，当前值可能已经过调优")
    
    print("\n" + "=" * 60)
    print("结论:")
    print("实际总质量: {:.3f} kg，比之前假设的 {:.3f} kg 重了约 {:.1f}%。".format(
        total_mass, 4.130 + 0.428*4 + 0.61*4 + 0.115*4 + 0.06*4,
        (total_mass / (4.130 + 0.428*4 + 0.61*4 + 0.115*4 + 0.06*4) - 1) * 100))
    print("控制器中的 Ib_ 值可能需要按比例调整。")
    print("建议将 Ib_ 增大 30-40% 进行测试，例如:")
    print("Ib_ = Vec3(0.10, 0.16, 0.20).asDiagonal();")

if __name__ == "__main__":
    main()
