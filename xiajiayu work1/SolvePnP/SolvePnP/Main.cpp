#include"head.h"
#include<iostream>
using namespace cv;
using namespace std;



int main() {
	Mat srcImg = imread("4.jpg");
	namedWindow("srcImg", 0);
	imshow("srcImg", srcImg);
	Mat imgThresholded = segmentImg(srcImg);
	vector<vector<Point> > contours;
	vector<Point2f> point_img;
	contours_filter(imgThresholded, contours);
	point_img = pick_point(contours);
	visual_point(srcImg, point_img);
	solvepnp(point_img);
	waitKey(0);
	return 0;
}