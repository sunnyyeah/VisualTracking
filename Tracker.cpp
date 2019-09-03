#include "Tracker.h"

namespace mycv {
	// 构造函数
	Tracker::Tracker()
	{
		cout << "运行：Tracker::Tracker()" << endl;
	}

	// 析构函数
	Tracker::~Tracker()
	{
	}

	// 初始化跟踪器
	bool Tracker::init(const Mat& initFrame, const Rect& initBoundingBox) {
		cout << "运行：Tracker::init()" << endl;
		return false;
	}

	// 跟踪目标
	bool Tracker::track(const Mat& currentFrame, Rect& currentBoundingBox) {
		cout << "运行：Tracker::track()" << endl;
		return false;
	}

	// 跟新目标模型
	bool Tracker::update(Rect& NextSearchBox, double& fps) {
		cout << "运行：Tracker::update()" << endl;
		return false;
	}
}
