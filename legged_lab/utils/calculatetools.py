import math

# 弧度列表
radians = [0.2, 0.42, 0.23]

# 将弧度转换为度数
degrees = [round(math.degrees(r),4) for r in radians]

# 打印结果
print(degrees)

# 角度列表
degrees = [2, 12, -24, 13]

# 将角度转换为弧度，并保留四位小数
radians = [round(math.radians(d), 4) for d in degrees]

# 打印结果
print(radians)