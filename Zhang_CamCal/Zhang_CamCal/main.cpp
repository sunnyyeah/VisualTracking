#include <iostream>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <fstream>
#include <iomanip>

using namespace std;
using namespace cv;

int main() {
	ifstream fin("calibdata9.txt"); /* 标定所用图像文件的路径 */
	ofstream fout("caliberation_result9.txt");  /* 保存标定结果的文件 */

	//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化   
	cout << "【开始提取角点】\n";
	int image_count = 0;  /* 图像数量 */
	Size image_size;  /* 图像的尺寸 */
	//Size board_size = Size(7, 6);    /* 标定板上每行、列的角点数 */
	Size board_size = Size(13, 8);
	vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点位置（焦点位置的每个数字都是以像素坐标保存） */
	vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
	string filename;
	int count = 0;//用于存储角点个数。  

	///
	/// -------------------------------------------提取角点-------------------------------------------------
	///
	while (getline(fin, filename))
	{
		image_count++;
		// 用于观察检验输出  
		cout << "image_count = " << image_count << endl;
		//cout << "-->count = " << count<<endl;
		Mat imageInput = imread(filename);   //读取当前图片
		if (image_count == 1)  //读入第一张图片时获取图像宽高信息  
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}

		//imshow("image", imageInput);
		//waitKey(20);

		/* 提取角点 */
		if (!findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"; //找不到角点  
			//exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imageInput, view_gray, COLOR_RGB2GRAY);   //将图像转换为灰度图

			/* 亚像素精确化 */
			cornerSubPix(view_gray, image_points_buf, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
			//亚像素精确化方法二
			//Size(5,5)是搜索窗口的大小,Size(-1,-1)表示没有死区
			//第四个参数定义求角点的迭代过程的终止条件，可以为迭代次数和角点精度两者的组合

			count += image_points_buf.size();
			image_points_seq.push_back(image_points_buf);//保存亚像素角点

			/*绘制棋盘角点*/
			drawChessboardCorners(view_gray, board_size, image_points_buf, false);
			//用于绘制被成功标定的角点，输入8位灰度或者彩色图像
			//第四个参数是标志位，用来指示定义的棋盘内角点是否被完整的探测到
			//false表示有未被探测到的内角点，这时候函数会以圆圈标记出检测到的内角点

			namedWindow("Camera Calibration", WINDOW_AUTOSIZE);
			imshow("Camera Calibration", view_gray);//显示图片
			waitKey(500);
		}
	}


	// 输出所有角点
	int total = image_points_seq.size();					 //总共有多上张图片成功检测出了角点
	cout << "total = " << total << endl << endl;
	int CornerNum = board_size.width*board_size.height;		//每张图片上总的角点数  
	for (int ii = 0; ii<total; ii++)
	{
		int k = -1;
		k = ii % total;										 // 13 是每幅图片个数。此判断语句是为了输出 图片号，便于控制台观看
			
		int j = k + 1;
		cout << "【第" << j << "图片的数据】 : " << endl;

		//输出所有的角点  第ii张图片的第i个点
		for (int i = 0; i < CornerNum; i++) {
			cout << "(" << image_points_seq[ii][i].x;
			cout << ", " << image_points_seq[ii][i].y << ")";
			if (0 == (i+1)% 6)								 // 此判断语句，格式化输出，便于控制台查看  
			{
				cout << endl;
			}
			else
			{
				cout.width(10);
			}
		}
		cout << endl;
	}
	cout << "角点提取完成！\n" << endl;

	/// 
	/// ----------------------------------------------------以下是摄像机标定  ------------------------------------------
	/// 
	cout << "【开始标定】" << endl;
	/*棋盘三维信息*/
	//Size square_size = Size(200, 200);
	Size square_size = Size(150, 150);						 // 实际测量得到的标定板上每个棋盘格的大小 
	vector<vector<Point3f>> object_points;					 // 保存所有图片的标定板上角点的三维坐标（这里是真实坐标） 
	
	/*内外参数*/
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); // 摄像机内参数矩阵 
	vector<int> point_counts;								// 每幅图像中角点的数量  
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));	// 摄像机的5个畸变系数：k1,k2,p1,p2,k3 
	vector<Mat> tvecsMat;									// 每幅图像的平移向量 
	vector<Mat> rvecsMat;									// 每幅图像的旋转向量 
						  
	/* 初始化标定板上角点真实的三维坐标 */
	int i, j, t;
	for (t = 0; t<image_count; t++)
	{
		vector<Point3f> tempPointSet;
		for (i = 0; i<board_size.height; i++)
		{
			for (j = 0; j<board_size.width; j++)
			{
				Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}


	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (i = 0; i<image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}

	/* 开始标定 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	cout << "【标定完成！】\n\n";

	//对标定结果进行评价  
	cout << "【开始评价标定结果】\n";
	double total_err = 0.0;								// 所有图像的平均误差的总和 
	double err = 0.0;									// 每幅图像的平均误差 
	vector<Point2f> image_points2;						// 保存重新计算得到的新投影点 
	cout << "\t*每幅图像的标定误差*\n";
	fout << "每幅图像的标定误差：\n";
	for (i = 0; i<image_count; i++)
	{
		vector<Point3f> tempPointSet = object_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
		vector<Point2f> tempImagePoint = image_points_seq[i];
		Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
		Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<Vec2f>(0, j) = Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<Vec2f>(0, j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		//求解误差
		err = norm(image_points2Mat, tempImagePointMat, NORM_L2);
		total_err += err /= point_counts[i];
		cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
	}
	cout << "总体平均误差：" << total_err / image_count << "像素" << endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << endl << endl;
	cout << "【评价完成！】\n" << endl;
	//保存定标结果      
	cout << "【开始保存定标结果】" << endl;
	Mat rotation_matrix = Mat(3, 3, CV_32FC1, Scalar::all(0));				// 保存每幅图像的旋转矩阵
	fout << "相机内参数矩阵：" << endl;
	fout << cameraMatrix << endl << endl;
	fout << "畸变系数：\n";
	fout << distCoeffs << endl << endl << endl;
	for (int i = 0; i<image_count; i++)
	{
		fout << "第" << i + 1 << "幅图像的旋转向量：" << endl;
		fout << rvecsMat[i] << endl;
		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(rvecsMat[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << endl;
		fout << rotation_matrix << endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << endl;
		fout << tvecsMat[i] << endl << endl;
	}
	cout << "【完成保存】\n" << endl;
	fout << endl;

	/************************************************************************
	显示定标结果
	*************************************************************************/
	Mat mapx = Mat(image_size, CV_32FC1);
	Mat mapy = Mat(image_size, CV_32FC1);
	Mat R = Mat::eye(3, 3, CV_32F);
	cout << "【保存矫正图像】" << endl;
	string imageFileName;   //读入图像
	string imageFileName1;   //输出图像
	stringstream StrStm;
	stringstream StrStm1;

	for (int i = 1; i <= image_count; i++)
	{
		cout << "Frame # " << i << "....." << endl;
		cout << cameraMatrix << endl;
		cout << distCoeffs << endl;
		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix,
								image_size, CV_32FC1, mapx, mapy);				//用来计算畸变映射

		StrStm.clear();//清除缓存
		StrStm1.clear();
		imageFileName.clear();
		imageFileName1.clear();
		//string filePath = "left";
		string filePath = "./Self11/pic";
		//StrStm << setw(2) << setfill('0') << i;
		StrStm << i;     //读入编号i
		StrStm >> imageFileName;   //将读入的数据传给imageFileName
		filePath += imageFileName;
		filePath += ".jpg";
		//获取图片路径
		Mat imageSource = imread(filePath);//读取图像
		Mat newimage = imageSource.clone();//拷贝图像

		
		remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);//把求得的映射应用到图像上
															   //与initUndistortRectifyMap结合使用，为矫正方法之一

															   //undistort(imageSource,newimage,cameraMatrix,distCoeffs);//矫正方法二
															   //第五个参数newCameraMatrix=noArray()，默认跟cameraMatrix保持一致,故可省

		StrStm1 << "./Result/result" << i;
		StrStm1 >> imageFileName1;
		imageFileName1 += "_d.jpg";								//矫正后图片命名
		imwrite(imageFileName1, newimage);						//保存矫正后的图片
		imshow("Original Image", imageSource);
		//waitKey(500);//暂停0.5s
		imshow("Undistorted Image", newimage);
		waitKey(500);

	}
	fin.close();
	fout.close();
	getchar();//等待输入以退出
	return 0;
}