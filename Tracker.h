#pragma once   //之前没加这个导致在类继承上面出了问题

#include <iostream>
#include <opencv2\opencv.hpp>

using namespace std;
using namespace cv;

namespace mycv {

	// 目标跟踪器
	class Tracker
	{
	public:
		Tracker();
		virtual ~Tracker();
		// 初始化跟踪器
		virtual bool init(const Mat& initFrame, const Rect& initBoundingBox);   //要把这些函数定义成virtual，才会在调用时调用子类的函数
		// 跟踪目标
		virtual bool track(const Mat& currentFrame, Rect& currentBoundingBox);
		// 更新目标模型
		virtual bool update(Rect& NextSearchBox, double& fps);
	};

}

