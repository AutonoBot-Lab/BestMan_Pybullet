from poissonDiskSampling import bridsonVariableRadius

nParticle, particleCoordinates = bridsonVariableRadius.poissonDiskSampling(rad, k=30, radiusType='default')

# ... 进行之前的步骤，如转换采样点为摄像头的旋转角度

# 旋转摄像头到每个角度并采集图像（伪代码部分）
# for angle in angles:
#     rotate_camera_to(angle)
#     capture_image()