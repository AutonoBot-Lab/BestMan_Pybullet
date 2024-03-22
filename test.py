import matplotlib.pyplot as plt

# 定义要涂黑的方形区域的顶点坐标
x = [1, 1, 2, 2]  # 方形的 x 坐标
y = [1, 2, 2, 1]  # 方形的 y 坐标

# 涂黑方形区域
plt.fill(x, y, color='black')

# 设置图形属性
plt.grid(True)
plt.axis("equal")
plt.show()
