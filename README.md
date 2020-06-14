QRCode-Localization
===
代码关于如何更具摆放好的二维码来获取无人机的运动高精度运动轨迹，方法是在无人机上固连一个拍摄二维码的辅助相机，保存拍摄到的图片以及图片的时间，然后进行离线识别，获得运动真值。

1.依赖的条件
---
OpenCV4.x
[Opencv4.x 下载](https://github.com/opencv/opencv/archive/4.3.0.zip)
[安装OpenCV4.x教程](https://blog.csdn.net/learning_tortosie/article/details/80594399)

2.ubuntu 下编译
---
下载到本地，然后解压
```bash
cd QRCode-Localization
mkdir build && cd build
cmake ..
make
```


3.示例
---
##单张图片进行多二维码检测，可获得可视化结果
```bash
cd build
./Test ../5.jpg
```
![单张图片运行结果](https://github.com/jlm345/QRCode-Localization/blob/master/result.png)
##连续采集的图片输出轨迹信息 数据集下载[百度网盘](www.baidu.com)(等待上传)
```bash
./QRPositions path_to_dataset
```
然后就能在trajectory.txt获得估计出的position信息
