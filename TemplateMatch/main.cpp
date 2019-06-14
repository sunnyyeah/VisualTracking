#include <iostream>
#include <string>
#include <vector>
#include <opencv2\opencv.hpp>

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
			SelectedRoi.x = MIN(x, origin.x);
			SelectedRoi.y = MIN(y, origin.y);
			SelectedRoi.width = std::abs(x - origin.x);
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

// 模板匹配函数
///函数说明
/// 四个参数依次是：原始函数，模板函数，返回的匹配点位置，匹配方法
float MatchTemplate(const Mat& src, const Mat& templ, Point2i& match_location, 
	int match_method, Vec2i& xy_step, Vec2i& xy_stride) {
	assert((src.type() == CV_8UC1) && (templ.type() == CV_8UC1));
	// 原图像与模板的尺寸
	int src_width = src.cols;
	int src_height = src.rows;
	int templ_cols = templ.cols;
	int templ_rows = templ.rows;
	int y_end = src_height - templ_rows + 1;
	int x_end = src_width - templ_cols + 1;

	// 在匹配过程中，记录最匹配的位置和匹配度
	float match_degree = FLT_MAX;
	int y_match = -1, x_match = -1;

	// 从上到下扫描原图像
	for (int y = 0; y < y_end; y+=xy_stride[1]) {
		// 从左到右扫描原图像
		for (int x = 0; x < x_end; x+= xy_stride[0]) {
			// src(y,x)位置上与模板的匹配度
			float match_yx = 0.0f;

			// 将模板左上角templ(0,0)对其到src(y,x)位置，在模板内累加每个采样像素点上的差异
			for (int r = 0; r < templ_rows; r+=xy_step[1]) {
				for (int c = 0; c < templ_cols; c+=xy_step[0]) {
					uchar src_val = src.ptr<uchar>(y + r)[x + c];
					uchar templ_val = templ.ptr<uchar>(r)[c];
					//cout << "src_val:" << src_val << endl;
					//cout << "templ_val:" << templ_val << endl;
					if (match_method == 0)			//SQDIFF
						match_yx += (float)(std::abs(src_val - templ_val) * std::abs(src_val - templ_val));
					if (match_method == 1)			//SADIFF
						match_yx += (float)(abs(src_val - templ_val));
					//cout << "match_yx:" << match_yx << endl;
				}
			}
			// 与历史最好的差异度进行比较，找出差异最小的点
			if (match_degree > match_yx) {
				match_degree = match_yx;
				x_match = x;
				y_match = y;
			}
		}
	}

	match_location = Point2i(x_match, y_match);
	return match_degree;
}

int main(int argc, char** argv) {
	const string image_file = "08.jpeg";   

	Mat srcImg = imread(image_file, IMREAD_GRAYSCALE);

	// 用来显示结果
	Mat displayImg;
	srcImg.copyTo(displayImg);

	// 设置全局变量
	global::isRoiReady = false;
	global::selectObject = false;
	global::displayImg = displayImg;

	const string winName = "Result Image";
	namedWindow(winName, WINDOW_AUTOSIZE);
	setMouseCallback(winName, global::onMouse, 0);

	// 循环显示图像， 等待鼠标选择ROI区域
	for (;;) {
		imshow(winName, displayImg);

		// 一旦选择好ROI区域，就进入处理
		if (global::isRoiReady) {
			// 设置为false，处理完此次就接着等待鼠标选择
			global::isRoiReady = false;

			// 提取鼠标选中的图像块
			Rect roiRect = global::SelectedRoi;
			Mat roiImg = srcImg(roiRect).clone();
			imshow("ROI Image", roiImg);				//显示roi图像块

			// 为原始图像添加高斯噪声
			Mat noiseImg(srcImg.size(), srcImg.type());
			randn(noiseImg, Scalar(0), Scalar(30));		//均值：0，标准差：30

			Mat workImg = noiseImg + srcImg;			//噪声图像
			// 显示噪声污染的图像和选择的ROI区域
			workImg.copyTo(displayImg);
			rectangle(displayImg, global::SelectedRoi, Scalar::all(0), 4);
			imshow(winName, displayImg);
			waitKey(15);

			// 选择的模板在被噪声污染的图像上匹配
			Point2i match_location;
			int match_method = 1;
			Vec2i xy_step(2, 2);		//模板与原图像素点匹配的步长
			Vec2i xy_stride(8, 8);		//模板在原图上移动的步长，对于大的roi区域而言，步长大点误差并不会太大，但是对于小的roi区域而言，大步长会让误差变大
			float matchDegree = MatchTemplate(workImg, roiImg, match_location, match_method, xy_step, xy_stride);
			Rect matchedRoi(match_location.x, match_location.y, roiImg.cols, roiImg.rows);
			cout << "匹配度：" << matchDegree << endl;
			cout << "匹配位置：" << matchedRoi << endl;
			// 显示匹配结果
			rectangle(displayImg, matchedRoi, Scalar::all(255), 2);
			imshow(winName, displayImg);
			waitKey(15);
		}

		waitKey(15);
	}

	waitKey(0);
	return 0;
}