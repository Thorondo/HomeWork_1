#include"head.h"

//分割图像
Mat segmentImg(Mat &src) {
	Mat srcImg = src.clone();
	Mat HsvImg, ImgThreshold;
	//BGR转HSV
	cvtColor(srcImg, HsvImg,COLOR_BGR2HSV);
	vector<Mat>splitHsv;
	split(HsvImg, splitHsv);
	equalizeHist(splitHsv[2], splitHsv[2]);
	merge(splitHsv, HsvImg);

	int lowH = 0, highH = 180, lowS = 0, highS = 43, lowV = 50, highV = 255;
	inRange(HsvImg, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV),ImgThreshold);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));

	morphologyEx(ImgThreshold, ImgThreshold, MORPH_OPEN, element);//开操作
	morphologyEx(ImgThreshold, ImgThreshold, MORPH_CLOSE, element);//闭操作
	namedWindow("thresholdImage", 0);
	imshow("thresholdImage", ImgThreshold);
	return ImgThreshold;

}


//检测轮廓
void contours_filter(Mat &imgThresholded, vector<vector<Point> > &contours)
{
	vector<Vec4i> hierarchy;
	findContours(imgThresholded, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);//求轮廓　
	//初步筛选符合要求的矩形
		Mat contour = Mat::zeros(imgThresholded.rows, imgThresholded.cols, CV_8SC3);
		drawContours(contour, contours, -1, Scalar(200, 200, 200), 2);
		imshow("contours", contour);

}

vector<Point2f>pick_point(vector<vector<Point>>&contours) {
	vector<Point>hull;
	vector<Point2f> point_img;
	convexHull(contours[0], hull);
	//cout << hull.size();
	vector<Point> squar;
	size_t num = hull.size();
    if (num >= 4) {
		float max_area;
		for (int m = 0;m < num - 3;m++) {
			for (int n = m + 1;n < num - 2;n++) {
				for (int j = n + 1;j < num - 1;j++) {
					for (int k = j + 1;k < num;k++) {
						vector<Point> squar_tmp;
						squar_tmp.push_back(hull[m]);
						squar_tmp.push_back(hull[n]);
						squar_tmp.push_back(hull[j]);
						squar_tmp.push_back(hull[k]);
						if (m == 0 && n == 1 && j == 2 && k == 3) {
							max_area = fabs(contourArea(Mat(squar_tmp)));
							squar.clear();
							squar = squar_tmp;
						}
						else {
							float area = fabs(contourArea(Mat(squar_tmp)));
							if (area > max_area) {
								max_area = area;
								squar.clear();
								squar = squar_tmp;
							}
						}
					}
				}
			}
		}
	}
	//给四点排序
	vector<Point> squar_sort = squar;
	for (int i = 0;i < squar_sort.size();i++) {
		point_img.clear();
		for (size_t num_p = 0;num_p < squar_sort.size();num_p++) {
			// point_img.push_back(squar_sort[num_p] * (1 / minifactor));
			point_img.push_back(squar_sort[num_p]);
		}
	}

	vector<Point2f> point_temp = point_img;
	point_img.clear();
	point_img.push_back(point_temp[1]);
	point_img.push_back(point_temp[2]);
	point_img.push_back(point_temp[3]);
	point_img.push_back(point_temp[0]);
	cout << point_img;
	return point_img;
}

// 可视化角点
void visual_point(Mat src, vector<Point2f> point_image)
{
	for (int i = 0;i < point_image.size();i++)
	{
		circle(src, point_image[i], 5, Scalar(255, 200, 0), 3);
	}
	imshow("point", src);
	imwrite("point.png", src);
}


