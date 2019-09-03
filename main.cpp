///<summary>
///多尺度目标跟踪
///</summary>

#include <iostream>
#include <string>
#include <vector>
#include <opencv2\opencv.hpp>
#include "Tracker.h"
#include "SingleTemplateTracker.h"
#include "MultipleTemplateTracker.h"
#include "datasets.h"

using namespace std;
using namespace cv;

// 绘制鼠标选择区域
namespace global {
	bool paused = true;			//单击鼠标右键，暂停标志
	Mat displayImg;				//绘制选择目标时鼠标的拖动痕迹
	bool selectObject = false;	//selectObject的初始值为false
	bool isRoiReady = false;	//ROI区域是否已经选择好
	Point origin;				//ROI区域的左上角起始位置
	Rect SelectedRoi;			//最终通过鼠标选择的ROI区域

	// 添加鼠标触发事件
	static void onMouse(int event, int x, int y, int, void*) {
		if (selectObject) {		//鼠标左键被按下后，该段语句开始执行
								//按住左键拖动鼠标的时候，该鼠标响应函数会被不断的触发，不断计算目标矩形的窗口
			SelectedRoi.x = MIN(x, origin.x);			// x为鼠标点击在图像上x方向的像素位置
			SelectedRoi.y = MIN(y, origin.y);
			SelectedRoi.width = std::abs(x - origin.x);	// 模板图像的宽度
			SelectedRoi.height = std::abs(y - origin.y);
			SelectedRoi &= Rect(0, 0, displayImg.cols, displayImg.rows);	//不能越界

			//画出鼠标的选择框
			rectangle(displayImg, SelectedRoi, Scalar(0, 0, 255), 1);
		}

		switch (event)
		{
			//当在第一帧按下鼠标左键后，select0bject被置为true
		case  EVENT_LBUTTONDOWN:
			origin = Point(x, y);
			SelectedRoi = Rect(x, y, 0, 0);
			selectObject = true;
			isRoiReady = false;
			break;
			//直到鼠标左键抬起，标志着目标区域选择完毕，selectObject被置为false
		case EVENT_LBUTTONUP:
			selectObject = false;
			if (SelectedRoi.width > 0 && SelectedRoi.height > 0)
				isRoiReady = true;
			cout << "目标区域已经选择完毕" << endl;
			cout << "选中的矩形区域为：" << SelectedRoi << endl;
			break;
			// 单击右键，暂停/开始
		case EVENT_RBUTTONDOWN:
			paused = !paused;
			break;
		}
	}
}

//视频数据集
//namespace datsets {
//	const string datasets_dir = "D:\\0_Study\\Opencv\\Workspace\\Tracking\\video\\";
//	const string video1 = datasets_dir + "VID_20181121_183353.mp4";
//	int video1_start_frame = 30;
//
//	const string video = video1;
//	int start_frame = video1_start_frame;
//}



int main(int argc, char* argv[]) {
	// 计算帧率
	double fps;
	//用于存放帧率的字符串
	char stringFps[10];		
	//用于存放帧数的字符串
	char stringframe[10];

	// 指定数据集，起始帧，其实目标
	mycv::DataSet dataset = mycv::dataset11;

	VideoCapture capture;

	// 打开视频
	capture.open(dataset.video_name);
	assert(capture.isOpened());

	// 获取视频信息
	const int FrameCount = (int)capture.get(VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
	const int FrameWidth = (int)capture.get(VideoCaptureProperties::CAP_PROP_FRAME_WIDTH);
	const int FrameHeight = (int)capture.get(VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT);
	const Rect FramArea(0, 0, FrameWidth, FrameHeight);

	// 设置从第几帧开始读取
	int frameIndex = dataset.start_frame;
	capture.set(VideoCaptureProperties::CAP_PROP_POS_FRAMES, double(frameIndex));

	// 创建显示跟踪过程的窗口
	const string winName = "Tracking Window";
	namedWindow(winName, 1);
	// 设置回调函数
	setMouseCallback(winName, global::onMouse, 0);		

	// 读取指定的起始帧
	Mat CurrentFrame, WorkFrame;
	capture >> CurrentFrame;
	assert(!CurrentFrame.empty());
	cout << "当前帧索引：" << frameIndex << endl;
	frameIndex++;

	// 在起始帧上选择目标区域
	while (!global::isRoiReady)
	{
		// 将起始帧拷贝到displayImg中
		CurrentFrame.copyTo(global::displayImg);

		// 在按下鼠标左键和抬起鼠标左键之间的这段时间，selectObject为true，
		// selectedRoi会随着鼠标的移动不断变化，直到抬起鼠标左键后，
		//selectObject为false，selectedRoi就是选中的目标矩形框
		if (global::selectObject && global::SelectedRoi.width > 0 && global::SelectedRoi.height > 0) {
			
			Mat roi_img(global::displayImg, global::SelectedRoi);			
			bitwise_not(roi_img, roi_img);		//把选中的区域图像反转显示		
		}

		// 显示鼠标选择过程
		imshow(winName, global::displayImg);
		waitKey(10);
	}

	// 如果lock_roi==true，就表示鼠标选择的区域无效
	if (dataset.lock_roi)
		global::SelectedRoi = dataset.start_roi;
	cout << "初始帧上的目标位置：" << global::SelectedRoi << endl;

	cout << "声明跟踪器对象实例，初始化目标跟踪器..." << endl;
	//mycv::STTracker::Params params = mycv::STTracker::Params();
	//params.numPoints = 1000;			// 修改默认参数
	//Ptr<mycv::Tracker> tracker = new mycv::SingleTemplateTracker(params);
	
	mycv::MTTracker::Params mtparams = mycv::MTTracker::Params();
	mtparams.alpha = 0.7;				// 修改默认参数
	mtparams.numPoints = 1000;			
	mtparams.sigma = Point2d(0.4, 0.4);
	mtparams.expandWidth = 50;
	Ptr<mycv::Tracker> tracker = new mycv::MultipleTemplateTracker(mtparams);

	cvtColor(CurrentFrame, WorkFrame, COLOR_BGR2GRAY);
	tracker->init(WorkFrame, global::SelectedRoi);
	
	cout << "单击鼠标右键开启跟踪...." << endl;

	// 进入循环，处理视频序列，跟踪目标
	for (; frameIndex < FrameCount;) {
		// 如果没有暂停，则继续读入下一帧图像
		if (!global::paused) {
			capture >> CurrentFrame;
			assert(!CurrentFrame.empty());
			cout << "当前帧索引：" << frameIndex << endl;
			frameIndex++;

			// 将读入的图像拷贝到displayImg中
			CurrentFrame.copyTo(global::displayImg);
			// 转换为灰度图像
			cvtColor(CurrentFrame, WorkFrame, COLOR_BGR2GRAY);

			// 开始跟踪
			Rect currentBoundingBox;
			tracker->track(WorkFrame, currentBoundingBox);

			// 更新目标模型
			Rect NextSearchBox;
			tracker->update(NextSearchBox, fps);

			// 显示帧率
			sprintf(stringFps, "%.2f", fps);		//帧率保留两位小数
			std::string fpsString("FPS:");
			fpsString += stringFps;
			putText(global::displayImg, fpsString, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

			// 显示帧数
			sprintf(stringframe, "%d", frameIndex);		//帧率保留两位小数
			std::string frameString("frameIndex:");
			frameString += stringframe;
			putText(global::displayImg, frameString, Point(5, 35), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));

			// 显示当前帧跟踪结果图像
			rectangle(global::displayImg, NextSearchBox, Scalar(255, 0, 0), 2);
			rectangle(global::displayImg, currentBoundingBox, Scalar(0, 0, 255), 2);
			imshow(winName, global::displayImg);

			// 保存每帧图片
			string imageFileName;   
			stringstream StrStm;
			StrStm.clear();
			imageFileName.clear();
			StrStm << "./Image/" << frameIndex-10;
			StrStm >> imageFileName;
			imageFileName += ".jpg";
			imwrite(imageFileName, global::displayImg);

			waitKey(30);
		}
		else
		{
			// 显示当前帧跟踪结果图像
			imshow(winName, global::displayImg);
			waitKey(300);
		}
	}
	return 0;
}