#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

namespace mycv {
	struct  DataSet
	{
		string video_name;
		int start_frame;
		Rect2i start_roi;
		bool lock_roi;
		DataSet(const string _videofile, int _start_frame,
			Rect2i _start_roi, bool _lock_roi) {
			video_name = _videofile;
			start_frame = _start_frame;
			start_roi = _start_roi;
			lock_roi = _lock_roi;
		}
	};

	const string datasets_dir = "D:\\0_Study\\Opencv\\Workspace\\Tracking\\video\\";
	const string video1 = datasets_dir + "VID_20181121_183353.mp4";

	DataSet dataset11(video1, 30, Rect2i(), false);
	DataSet dataset12(video1, 30, Rect2i(336,561,63,127), true);
	DataSet dataset13(video1, 30, Rect2i(337,152,21,47), true);
	DataSet dataset14(video1, 80, Rect2i(), false);

	const string video2 = datasets_dir + "smuglanka.mp4";
	DataSet dataset21(video2, 296, Rect2i(), false);
}