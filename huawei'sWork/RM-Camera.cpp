// RM-Camera.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <string>

using namespace cv;
using namespace std;
void on_mouse(int event, int x, int y, int flag, void *param);
int get_fea();
int pnp_sol(); 
vector<Point2f>points;
vector<Point2f>goodfeature;
Mat frame;
Size subPixWinSize(10, 10), winSize(31, 31);
TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.01);

void codeRotateByZ(double x, double y, double thetaz, double* outx, double* outy)
{
	double x1 = x;//将变量拷贝一次，保证&x == &outx这种情况下也能计算正确
	double y1 = y;
	double rz = thetaz * CV_PI / 180;
	*outx = cos(rz) * x1 - sin(rz) * y1;
	*outy = sin(rz) * x1 + cos(rz) * y1;
}

void codeRotateByY(double x, double z, double thetay, double* outx, double* outz)
{
	double x1 = x;
	double z1 = z;
	double ry = thetay * CV_PI / 180;
	*outx = cos(ry) * x1 + sin(ry) * z1;
	*outz = cos(ry) * z1 - sin(ry) * x1;
}

void codeRotateByX(double y, double z, double thetax, double* outy, double* outz)
{
	double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
	double z1 = z;
	double rx = thetax * CV_PI / 180;
	*outy = cos(rx) * y1 - sin(rx) * z1;
	*outz = cos(rx) * z1 + sin(rx) * y1;
}


int pnp_sol()
{
	double camD[9] = { 9.000517011369869e+02,0.274666410547990, 3.190051580550110e+02, 0,8.987165220399067e+02,2.396192084412702e+02, 0, 0, 1 };	//通过Matlab获得的内参和畸变
	double distCoeffD[5] = { 0.0882,-0.1506, -8.694265304966329e-04,0.001558602637161,0.00000 };
	Mat cam_matrix = Mat(3, 3, CV_64FC1, camD);
	Mat distortion_coefficients = Mat(5, 1, CV_64FC1, distCoeffD);
	Mat obj_m;
	Mat_<float> tvec;
	Mat raux, taux, rvec;
	Mat_<float> rotMat(3, 3);
	vector<Point3f> obj_p;
	obj_p.push_back(Point3f(0, 0, 0));//目标的世界坐标
	obj_p.push_back(Point3f(0,5, 0));
	obj_p.push_back(Point3f(0,0,-5));
	obj_p.push_back(Point3f(0, 5,-5));
	Mat(obj_p).convertTo(obj_m, CV_32F);
	namedWindow("test", 1);
	imshow("test", frame);
	get_fea();
	cvSetMouseCallback("test", on_mouse, NULL);
	if (points.size() == 4) {
		solvePnP(obj_m, Mat(points), cam_matrix, distortion_coefficients, raux, taux);
		raux.convertTo(rvec, CV_32F);
		taux.convertTo(tvec, CV_32F);
		Rodrigues(rvec, rotMat);
		//获取旋转角
		float theta_z = atan2(rotMat[1][0], rotMat[0][0])*57.2958;
		float theta_y = atan2(-rotMat[2][0], sqrt(rotMat[2][1] * rotMat[2][1] + rotMat[2][2] * rotMat[2][2]))*57.2958;
		float theta_x = atan2(rotMat[2][1], rotMat[2][2])*57.2958;
		double o_x, o_y, o_z;
		o_x = tvec(0);
		o_y = tvec(1);
		o_z = tvec(2);
		codeRotateByZ(o_x, o_y, -1 * theta_z, &o_x, &o_y);
		codeRotateByY(o_x, o_z, -1*theta_y, &o_x, &o_z);
		codeRotateByX(o_y, o_z, -1 * theta_x, &o_y, &o_z);
		cout << -1*o_x << " " << -1*o_y << " " << -1*o_z << endl;
		points.clear();//清空画面上标记点
	}
	return 1;
}


void on_mouse(int event, int x, int y, int flag, void *param)//获取用户标定的点并于角点进行比较，取最近的角点压入points
{
	if (event == CV_EVENT_LBUTTONDOWN)
		if (points.size() < 4) 
			for (size_t i = 0; i<goodfeature.size(); i++)
			{
				if (abs(goodfeature[i].x - x) + abs(goodfeature[i].y - y)<3)//检测用户标定点附近角点
				{
					points.push_back(goodfeature[i]);
					cout << "done" << endl;//检测成功给出提示
					break;
				}
			}										
}

int get_fea()		//获取当前画面的角点
{
	Ptr<ORB> orb = ORB::create();
	vector<KeyPoint>keypoints;
	Mat discript;
	Mat image = frame.clone();
	Mat gray;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	// automatic initialization
	orb->detectAndCompute(image, Mat(), keypoints, discript);//ORB角点检测
	goodfeature.clear();
	for (size_t i = 0; i < keypoints.size(); i++) {
		goodfeature.push_back(keypoints[i].pt);
	}
	cornerSubPix(gray, goodfeature, subPixWinSize, Size(-1, -1), termcrit);//亚像素化
	for (size_t i = 0; i < goodfeature.size(); i++)
	{
		circle(image, goodfeature[i], 3, Scalar(0, 255, 0), -1, 8);
	}
	imshow("test1", image);
	return 1;
}

int main()
{
	VideoCapture cap(0);
	if (!cap.isOpened()) {
		cout << "Unable to open the cam" << endl;
		return 0;
	}
	while(waitKey(0)!=27)	//键入ESC退出
	{ 
		cap >> frame;
		pnp_sol();
	}
	return 0;
}