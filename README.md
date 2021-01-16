# Hand-Eye-with-matlab
Use matlab for eye-in-hand calibration

# 1.尽量将相机在不同的位置拍摄棋盘格，前提是使棋盘格完整出现在视场中。

# 2.left文件夹中存放对应采集的图片

# 3.坐标6.txt文件夹里存放对应的机械臂位置和姿态，每一行7个元素，分别是位置和四元数的值（通过机械臂sdk反馈）

# 4.如何使用matlab进行标定？？
 4.1 使用matlab APP中的Camera Calibrator进行图像内外参标定，并导出数据到工作空间。
 4.2 运行toolbox_stereo_eye_in_hand.m获得标定的旋转矩阵和平移矩阵
