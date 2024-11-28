from math import *
import numpy as np

# cam_fx:     387.229
# cam_fy:     387.229
# cam_cx:     320.000
# cam_cy:     243.449
# # cam_cx:     321.046

# proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
# proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;

left = atan2(321.046, 387.229)
right = atan2(640 - 321.046, 387.229)
up = atan2(243.449, 387.229)

print(left)
print(right)
print(up)

# 使用 180 / π 将弧度转换为角度
print(degrees(left))
print(degrees(up))

# 展示 exp 运算
print(exp(0.5) ** 2)
print(exp(0.2) ** 5)

# Numpy 数组操作
a1 = np.array([29.08, 31.51, 32.22, 28.65, 28.95, 28.7, 30.84, 34.29])
a2 = np.array([31.50, 27.20, 30.90, 32.74, 29.65, 30.6, 29.93])

print(a1.mean())
print(a1.var())
print(a1.max())
print(a2.mean())
print(a2.var())
print(a2.max())
