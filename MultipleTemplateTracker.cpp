#include "MultipleTemplateTracker.h"

namespace mycv {
	MultipleTemplateTracker::MultipleTemplateTracker(Params _params)
	{
		cout << "运行：MultipleTemplateTracker::MultipleTemplateTracker()" << endl;
		this->params = _params;
	}


	MultipleTemplateTracker::~MultipleTemplateTracker()
	{
	}

	// 初始化跟踪器
	bool MultipleTemplateTracker::init(const Mat& initFrame, const Rect& initBoundingBox) {
		cout << "运行：MultipleTemplateTracker::init()" << endl;
		this->FrameArea = Rect(0, 0, initFrame.cols, initFrame.rows);
		
		
		// 提取初始帧上的初始多尺度目标模板
		this->GenerateMultiScaleTargetTemplates(initFrame(initBoundingBox),
			this->MultiScaleTargetsTemplates);
		this->ShowMultiScaleTemplates(this->MultiScaleTargetsTemplates);

		// 估计下一帧的搜索范围
		this->EstimateSearchArea(initBoundingBox, this->NextSearchArea,
			this->params.expandWidth, this->params.expandWidth);

		// 初始化标准区间随机采样点集
		this->GenerateRandomSamplePoints(this->SamplePoints, this->params.numPoints, this->params.sigma);
		return false;
	}

	// 跟踪目标
	bool MultipleTemplateTracker::track(const Mat& currentFrame, Rect& currentBoundingBox) {
		cout << "运行：MultipleTemplateTracker::track()" << endl;

		// 在ROI子区域内搜索目标
		Rect2i match_location(-1, -1, 0, 0);

		// 调用多尺度模板在当前帧匹配
		this->MatchMultiScaleTemplates(currentFrame(this->NextSearchArea), this->MultiScaleTargetsTemplates,
			match_location, this->params.matchMethod, this->params.matchStrategy, this->SamplePoints,
			this->params.xyStep, this->params.xyStride);
		
		// 重新调整匹配点坐标，使其返回原始图像的坐标系
		match_location.x += this->NextSearchArea.x;
		match_location.y += this->NextSearchArea.y;

		// 计算当前帧上的目标位置
		this->currentBoundingBox = match_location;

		// 抓取当前帧的目标图像块
		this->CurrentTargetPatch = currentFrame(this->currentBoundingBox).clone();

		// 输出跟踪结果
		currentBoundingBox = this->currentBoundingBox;
		return false;
	}

	// 更新目标模型
	bool MultipleTemplateTracker::update(Rect& NextSearchBox, double& fps) {
		cout << "运行：MultipleTemplateTracker::update()" << endl;

		t = (double)getTickCount();		//返回从操作系统启动到当前所经的计时周期数

		// 更新多尺度目标模板库
		this->UpdateMultiScaleTargetTemplates(this->CurrentTargetPatch);
		// 显示多尺度目标模板
		this->ShowMultiScaleTemplates(this->MultiScaleTargetsTemplates);


		// 更新下一帧上的局部搜索范围
		this->EstimateSearchArea(this->currentBoundingBox, this->NextSearchArea,
			this->params.expandWidth, this->params.expandWidth);

		// 输出局部搜索范围
		NextSearchBox = this->NextSearchArea;

		// 计算帧率
		t = ((double)getTickCount() - t) / getTickFrequency();
		fps = 1.0 / t;

		return false;
	}

	// 模板匹配函数
	///函数说明
	/// 四个参数依次是：原始函数，模板函数，返回的匹配点位置，匹配方法
	float MultipleTemplateTracker::MatchTemplate(const Mat& src, const Mat& templ, Rect2i& match_location,
		MatchMethod match_method, Vec2i& xy_step, Vec2i& xy_stride) {
		assert((src.type() == CV_8UC1) && (templ.type() == CV_8UC1));
		// 原图像与模板的尺寸
		int src_width = src.cols;
		int src_height = src.rows;
		int templ_cols = templ.cols;
		int templ_rows = templ.rows;
		int y_end = src_height - templ_rows + 1;
		int x_end = src_width - templ_cols + 1;

		cout << "x_end: " << x_end << endl;
		cout << "y_end: " << y_end << endl;

		// 在匹配过程中，记录最匹配的位置和匹配度
		float match_degree = FLT_MAX;
		int y_match = -1, x_match = -1;

		// 从上到下扫描原图像
		for (int y = 0; y < y_end; y += xy_stride[1]) {
			// 从左到右扫描原图像
			for (int x = 0; x < x_end; x += xy_stride[0]) {
				// src(y,x)位置上与模板的匹配度
				float match_yx = 0.0f;

				// 将模板左上角templ(0,0)对其到src(y,x)位置，在模板内累加每个采样像素点上的差异
				for (int r = 0; r < templ_rows; r += xy_step[1]) {
					for (int c = 0; c < templ_cols; c += xy_step[0]) {
						uchar src_val = src.ptr<uchar>(y + r)[x + c];
						uchar templ_val = templ.ptr<uchar>(r)[c];
						//cout << "src_val:" << src_val << endl;
						//cout << "templ_val:" << templ_val << endl;
						if (match_method == MatchMethod::SQDIFF)			//SQDIFF
							match_yx += (float)(std::abs(src_val - templ_val) * std::abs(src_val - templ_val));
						if (match_method == MatchMethod::SADIFF)			//SADIFF
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

		match_location = Rect2i(x_match, y_match, templ_cols, templ_rows);
		return match_degree;
	}

	// 估计下一帧的搜索范围
	void MultipleTemplateTracker::EstimateSearchArea(const Rect& target_location, Rect& search_area, int expand_x, int expand_y) {
		float center_x = target_location.x + 0.5f*target_location.width;
		float center_y = target_location.y + 0.5f*target_location.height;
		search_area.width = target_location.width + expand_x;
		search_area.height = target_location.height + expand_y;
		search_area.x = int(center_x - 0.5f*search_area.width);
		search_area.y = int(center_y - 0.5f*search_area.height);
		search_area &= this->FrameArea;
	}

	// 在指定的采样范围内产生截断正太分布点集
	void MultipleTemplateTracker::GenerateRandomSamplePoints(vector<Point2d>& sample_points,
		int num_points /*= 1000*/, Point2d& sigma /*= Point2d(0.3, 0.3)*/) {
		RNG rng = theRNG();
		Rect2d sample_area(0.0, 0.0, 1.0, 1.0);
		for (int k = 0; k < num_points;) {
			Point2d pt;
			pt.x = sample_area.width / 2.0 + rng.gaussian(sigma.x);
			pt.y = sample_area.height / 2.0 + rng.gaussian(sigma.y);
			if (sample_area.contains(pt)) {
				sample_points.push_back(pt);
				k++;
			}
		}
	}

	// 使用随机采样点集进行模板匹配
	float MultipleTemplateTracker::MatchTemplate(const Mat& src, const Mat& templ, Rect2i& match_location,
		MatchMethod match_method, const vector<Point2d>& sample_points) {
		CV_Assert((src.type() == CV_8UC1) && (templ.type() == CV_8UC1));

		// 原图像和模板的尺寸
		int src_width = src.cols;
		int src_height = src.rows;
		int templ_cols = templ.cols;
		int templ_rows = templ.rows;
		int y_end = src_height - templ_rows + 1;
		int x_end = src_width - templ_cols + 1;

		/*cout << "x_end: " << x_end << endl;
		cout << "y_end: " << y_end << endl;*/

		// 针对具体的模板大小，将采样点的坐标进行缩放
		vector<Point2i> SamplePoints(sample_points.size());
		for (size_t k = 0; k < sample_points.size(); k++) {
			const Point2d& ptd = sample_points[k];
			Point2i& pti = SamplePoints[k];				// 保存缩放以后的点集，为int类型
			pti.x = cvFloor(ptd.x * templ_cols);
			pti.y = cvFloor(ptd.y * templ_rows);
		}

		// 在匹配过程中，记录最匹配的位置和匹配度
		float match_dgree = FLT_MAX;
		int y_match = -1, x_match = -1;

		// 从上到下扫描原图
		for (int y = 0; y < y_end; y++) {
			// 从左到右扫描原图
			for (int x = 0; x < x_end; x++) {
				// src(y,x)位置上有模板的匹配度
				float match_yx = 0.0f;
				// 按照采样点数组计算模板与原始图像的匹配度
				for (size_t k = 0; k < SamplePoints.size(); k++) {
					Point2i& pt = SamplePoints[k];
					//cout << "pt.x,pt.y:" << pt.x << "," << pt.y << endl;
					uchar src_val = src.ptr<uchar>(y + pt.y)[x + pt.x];
					uchar templ_val = templ.ptr<uchar>(pt.y)[pt.x];
					//cout << "第" << k << "个点的src_val和templ_Val-----" << src_val << "    ----" << templ_val << endl;
					if (match_method == MatchMethod::SQDIFF)
						match_yx += float(abs(src_val - templ_val)*abs(src_val - templ_val));
					if (match_method == MatchMethod::SADIFF)
						match_yx += float(abs(src_val - templ_val));
				}
				// 与历史最好的差异度进行比较，找出差异最小的点
				if (match_dgree > match_yx) {
					match_dgree = match_yx;
					x_match = x;
					y_match = y;
				}
			}
		}
		match_location = Rect2i(x_match, y_match, templ_cols, templ_rows);
		return match_dgree;
	}

	// 产生多尺度目标模板
	void MultipleTemplateTracker::GenerateMultiScaleTargetTemplates(const Mat& origin_target, vector<Mat>& multiscale_target) {
		vector<double> resize_scales = { 1.5, 1.4, 1.3, 1.2, 1.1, 1.0,
			0.9, 0.8, 0.7, 0.6, 0.5 };
		multiscale_target.resize(resize_scales.size(), Mat());
		for (size_t scidx = 0; scidx < resize_scales.size(); scidx++) {
			cv::resize(origin_target, multiscale_target[scidx], Size(), resize_scales[scidx],
				resize_scales[scidx], InterpolationFlags::INTER_AREA);
		}
		return;
	}

	// 显示多尺度模板匹配
	void MultipleTemplateTracker::ShowMultiScaleTemplates(const vector<Mat>& multiscale_targets) {
		int total_cols = 0, total_rows = 0;
		vector<Rect2i> target_rois(multiscale_targets.size());

		for (size_t k = 0; k < multiscale_targets.size(); k++) {
			target_rois[k] = Rect2i(total_cols, 0, multiscale_targets[k].cols, multiscale_targets[k].rows);
			total_cols += multiscale_targets[k].cols;
			total_rows = max(multiscale_targets[k].rows, total_rows);
		}
		Mat targetsImg = Mat::zeros(total_rows, total_cols, CV_8UC1);
		for (size_t k = 0; k < multiscale_targets.size(); k++) {
			multiscale_targets[k].copyTo(targetsImg(target_rois[k]));
		}
		imshow("Target Image", targetsImg);			// 显示roi图像块
		waitKey(100);
	}

	// 使用多尺度模板匹配
	float MultipleTemplateTracker::MatchMultiScaleTemplates(
		const Mat& src, const vector<Mat>& multiscale_templs, Rect2i& best_match_location,
		MatchMethod match_method, MatchStrategy match_strategy,
		const vector<Point2d>& sample_points, Vec2i& xy_step, Vec2i& xy_stride) {
		
		if (match_strategy == MatchStrategy::NORMAL) {
			CV_Assert(!sample_points.empty());
		}

		// 记录最佳匹配度和最佳匹配位置
		float bestMatchDgree = FLT_MAX;
		Rect bestMatchLocation;
		// 记录每次尺度匹配的位置和匹配度
		Rect matchLocation;
		float matchDgree;
		// 拿着多尺度模板在目标图像上匹配
		for (size_t scaleIdx = 0; scaleIdx < multiscale_templs.size(); scaleIdx++) {
			const Mat& templ = multiscale_templs[scaleIdx];
			if (match_strategy == MatchStrategy::UNIFORM) {
				matchDgree = this->MatchTemplate(src, templ, matchLocation,
					match_method, xy_step, xy_stride);
			}
			if (match_strategy == MatchStrategy::NORMAL) {
				matchDgree = this->MatchTemplate(src, templ, matchLocation,
					match_method, sample_points);
			}
			// 记录最佳匹配度和匹配位置
			if (matchDgree < bestMatchDgree) {
				bestMatchDgree = matchDgree;
				bestMatchLocation = matchLocation;
			}
		}
		best_match_location = bestMatchLocation;
		return bestMatchDgree;
	}


	// 更新多尺度目标模板
	void MultipleTemplateTracker::UpdateMultiScaleTargetTemplates(const Mat& currentTargetPatch) {
		for (size_t idx = 0; idx < this->MultiScaleTargetsTemplates.size(); idx++) {
			
			if (this->MultiScaleTargetsTemplates[idx].size() == currentTargetPatch.size()) {
			
				cv::addWeighted(this->MultiScaleTargetsTemplates[idx],
					this->params.alpha, currentTargetPatch, 
					1.0 - this->params.alpha, 0.0, 
					this->MultiScaleTargetsTemplates[idx]);
			}
		}
	}
}