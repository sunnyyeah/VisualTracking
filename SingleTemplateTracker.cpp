#include "SingleTemplateTracker.h"


namespace mycv {
	SingleTemplateTracker::SingleTemplateTracker(Params _params)
	{
		cout << "运行：SingleTemplateTracker::SingleTemplateTracker()" << endl;
		this->params = _params;
	}


	SingleTemplateTracker::~SingleTemplateTracker()
	{
	}

	// 初始化跟踪器
	bool SingleTemplateTracker::init(const Mat& initFrame, const Rect& initBoundingBox) {
		cout << "运行：SingleTemplateTracker::init()" << endl;
		this->FrameArea = Rect(0,0,initFrame.cols, initFrame.rows);
		// 提取初始帧上的目标模板
		this->TargetTemplate = initFrame(initBoundingBox).clone();
		// 估计下一帧的搜索范围
		this->EstimateSearchArea(initBoundingBox, this->NextSearchArea, 
			this->params.expandWidth, this->params.expandWidth);

		// 初始化标准区间随机采样点集
		this->GenerateRandomSamplePoints(this->SamplePoints, this->params.numPoints, this->params.sigma);
		return false;
	}

	// 跟踪目标
	bool SingleTemplateTracker::track(const Mat& currentFrame, Rect& currentBoundingBox) {
		cout << "运行：SingleTemplateTracker::track()" << endl;

		// 在ROI子区域内搜索目标
		Rect2i match_location(-1, -1, 0, 0);

		// 计算匹配度时采样点在模板内部均匀分布
		if(this->params.matchStrategy == MatchStrategy::UNIFORM)
			this->MatchTemplate(currentFrame(this->NextSearchArea), this->TargetTemplate, match_location, this->params.matchMethod, this->params.xyStep, this->params.xyStride);
		// 计算匹配度时采样点在模板内部正态分布
		if (this->params.matchStrategy == MatchStrategy::NORMAL)
			this->MatchTemplate(currentFrame(this->NextSearchArea), this->TargetTemplate, match_location, this->params.matchMethod, this->SamplePoints);
		
		// 重新调整匹配点坐标，使其返回原始图像的坐标系
		match_location.x += this->NextSearchArea.x;
		match_location.y += this->NextSearchArea.y;

		// 计算当前帧上的目标位置
		this->currentBoundingBox = Rect(match_location.x, match_location.y, this->TargetTemplate.cols, this->TargetTemplate.rows);

		// 抓取当前帧的目标图像块
		this->CurrentTargetPatch = currentFrame(this->currentBoundingBox).clone();

		// 输出跟踪结果
		currentBoundingBox = this->currentBoundingBox;
		return false;
	}

	// 更新目标模型
	bool SingleTemplateTracker::update(Rect& NextSearchBox, double& fps) {
		cout << "运行：SingleTemplateTracker::update()" << endl;
		
		// 更新目标表面特征模型
		double alpha = 0.7;
		addWeighted(this->TargetTemplate, alpha, this->CurrentTargetPatch, 
			1.0 - alpha, 0.0, this->TargetTemplate);
		
		// 更新下一帧上的局部搜索范围
		this->EstimateSearchArea(this->currentBoundingBox, this->NextSearchArea, 
			this->params.expandWidth, this->params.expandWidth);

		NextSearchBox = this->NextSearchArea;
		return false;
	}

	// 模板匹配函数
	///函数说明
	/// 四个参数依次是：原始函数，模板函数，返回的匹配点位置，匹配方法
	float SingleTemplateTracker::MatchTemplate(const Mat& src, const Mat& templ, Rect2i& match_location,
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
	void SingleTemplateTracker::EstimateSearchArea(const Rect& target_location, Rect& search_area, int expand_x, int expand_y) {
		float center_x = target_location.x + 0.5f*target_location.width;
		float center_y = target_location.y + 0.5f*target_location.height;
		search_area.width = target_location.width + expand_x;
		search_area.height = target_location.height + expand_y;
		search_area.x = int(center_x - 0.5f*search_area.width);
		search_area.y = int(center_y - 0.5f*search_area.height);
		search_area &= this->FrameArea;
	}

	// 在指定的采样范围内产生截断正太分布点集
	void SingleTemplateTracker::GenerateRandomSamplePoints(vector<Point2d>& sample_points,
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
	float SingleTemplateTracker::MatchTemplate(const Mat& src, const Mat& templ, Rect2i& match_location,
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

}