// solvepnp测试.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include "pch.h"
#include<opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
	
	Size image_size;
	Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));        // 摄像机内参数矩阵
	Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));          // 摄像机的5个畸变系数：k1,k2,p1,p2,k3
	vector<Mat> rvecsMat;                                          // 存放所有图像的旋转向量，每一副图像的旋转向量为一个mat
	vector<Mat> tvecsMat;                                          // 存放所有图像的平移向量，每一副图像的平移向量为一个mat

	vector<Mat> images;
	for (int i = 0; i < 8; i++)
	{
		Mat image = imread("IMG_20181102_0" + format("%d", i+1) + ".jpg");
		if (i==1)
		{
			image_size = Size(image.cols, image.rows);
		}
		images.push_back(image);
	}
	vector<Point2f> image_points;
	vector<vector<Point2f>> image_points_seq;
	for (int i = 0; i < 8; i++)
	{
		Mat image = images[i];
		bool b = findChessboardCorners(image, Size(7, 7), image_points);			//获得棋盘图像的角点
		if (b)
		{
			image_points_seq.push_back(image_points);
		}
		else
		{
			continue;
		}
	}
	//获得棋盘真实点的坐标
	Size square_size = Size(20, 20);						//棋盘方格的物理尺寸     
	vector<vector<Point3f>> object_points_seq;				//棋盘方格的实际点在世界坐标系下的物理尺寸坐标
	for (int t = 0; t < image_points_seq.size(); t++)
	{
		vector<Point3f> object_points;
		for (int i = 0; i < 7; i++)
		{
			for (int j = 0; j < 7; j++)
			{
				Point3f realPoint;
				realPoint.x = (float)i * square_size.width;
				realPoint.y = (float)j * square_size.height;
				realPoint.z = 0;
				object_points.push_back(realPoint);
			}
		}
		object_points_seq.push_back(object_points);
	}

	double result = calibrateCamera(object_points_seq, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
	cout << cameraMatrix << endl;
	vector<Mat> rvecsMatPnp;
	vector<Mat> tvecsMatPnp;
	bool pnp = solvePnP(object_points_seq, image_points_seq, cameraMatrix, distCoeffs, rvecsMatPnp, tvecsMatPnp);
	//Mat src1 = imread("IMG_20181102_132719.jpg");
	//vector<cv::Point2f> corners;
	//Mat dst1 = Mat::zeros(Size(src1.cols, src1.rows), CV_8UC3);
	//findChessboardCorners(src1, Size(7, 7), corners);
	//drawChessboardCorners(src1, Size(7, 7), corners, 1);
	//imshow("相机标定图像", src1);
	//imshow("棋盘角点绘制", dst1);
	waitKey(0);
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
