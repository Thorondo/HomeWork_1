#pragma once
#ifndef SEGMENT_H_
#define SEGMENT_H_
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;




Mat segmentImg(Mat &src);
void contours_filter(Mat &imgThresholded, vector<vector<Point> > &contours);
vector<Point2f>pick_point(vector<vector<Point>>&contours);
void visual_point(Mat src, vector<Point2f> point_image);


void solvepnp(vector<Point2f>&point_img);
void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
void CodeRotateByY(double x, double y, double thetaz, double& outx, double& outz);
void CodeRotateByZ(double x, double y, double thetaz, double& outy, double& outz);
#endif // SEGMENT_H_
