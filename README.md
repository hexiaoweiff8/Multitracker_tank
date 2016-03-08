# Multitracker_tank
车辆输转多目标
需要更改输入的视频，在代码开始的位置
使用了背景差和KCF的方法（当出现连通域连在一起的情况）

由于标志点的比例问题，导致长宽无法被正确识别
实际使用中需要进行更改id_mark中的代码,tracker.cpp 中的distrube

TODO:
angle: -0 <0??? 是否需要角度变化是否需要判定？
Done:STC的加入