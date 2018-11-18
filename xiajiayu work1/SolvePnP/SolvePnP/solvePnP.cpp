#include"head.h"

void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
	double x1 = x;
	double y1 = y;
	double rz = thetaz * CV_PI / 180;
	outx = cos(rz) * x1 - sin(rz) * y1;
	outy = sin(rz) * x1 + cos(rz) * y1;
}

//将空间点绕Y轴旋转
//输入参数 x z为空间点原始x z坐标
//thetay为空间点绕Y轴旋转多少度，角度制范围在-180到180
//outx outz为旋转后的结果坐标
void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz)
{
	double x1 = x;
	double z1 = z;
	double ry = thetay * CV_PI / 180;
	outx = cos(ry) * x1 + sin(ry) * z1;
	outz = cos(ry) * z1 - sin(ry) * x1;
}

//将空间点绕X轴旋转
//输入参数 y z为空间点原始y z坐标
//thetax为空间点绕X轴旋转多少度，角度制，范围在-180到180
//outy outz为旋转后的结果坐标
void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
	double y1 = y;//将变量拷贝一次，保证&y == &y这种情况下也能计算正确
	double z1 = z;
	double rx = thetax * CV_PI / 180;
	outy = cos(rx) * y1 - sin(rx) * z1;
	outz = cos(rx) * z1 + sin(rx) * y1;
}

void solvepnp(vector<Point2f>&point_img) {
	vector<Point3f>point3D;
	//MATLAB标定得到相机内参以及畸变参数
	double cameraMatrix[9] = {
						877.5879   ,  0 , 590.3458 ,
						0,  879.4849   ,403.7733,
						0, 0,   1.0000
	};
	Mat CameraMatrix = Mat(3, 3, CV_64FC1, cameraMatrix);
	double distCoeffD[5] = {0.0208 ,-0.0826,0,0,0};
	Mat distortion_coefficients = Mat(5, 1, CV_64FC1, distCoeffD);
	Mat rvec = Mat::zeros(3, 1, CV_64FC1);
	Mat tvec = Mat::zeros(3, 1, CV_64FC1);
	//输入世界坐标系
	point3D.push_back(Point3f(0,210, 0));
	point3D.push_back(Point3f(0, 0, 0));
	point3D.push_back(Point3f(300,0, 0));
	point3D.push_back(Point3f(300, 210, 0));
	//使用solvepnp得到旋转向量和平移向量
	solvePnP(point3D, point_img, CameraMatrix, distortion_coefficients, rvec, tvec);
	//旋转向量变旋转矩阵
	//提取旋转矩阵
	double rm[9];
	Mat rotM(3, 3, CV_64FC1, rm);
	Rodrigues(rvec, rotM);
	double r11 = rotM.ptr<double>(0)[0];
	double r12 = rotM.ptr<double>(0)[1];
	double r13 = rotM.ptr<double>(0)[2];
	double r21 = rotM.ptr<double>(1)[0];
	double r22 = rotM.ptr<double>(1)[1];
	double r23 = rotM.ptr<double>(1)[2];
	double r31 = rotM.ptr<double>(2)[0];
	double r32 = rotM.ptr<double>(2)[1];
	double r33 = rotM.ptr<double>(2)[2];

	//计算出相机坐标系的三轴旋转欧拉角，旋转后可以转出世界坐标系。
	double thetaz = atan2(r21, r11) / CV_PI * 180;
	double thetay = atan2(-1 * r31, sqrt(r32*r32 + r33 * r33)) / CV_PI * 180;
	double thetax = atan2(r32, r33) / CV_PI * 180;
	cout << "相机的三轴旋转角：" << -1 * thetax << ", " << -1 * thetay << ", " << -1 * thetaz << endl;
	

	//提出平移矩阵，表示从相机坐标系原点，跟着向量(x,y,z)走，就到了世界坐标系原点
	double tx = tvec.ptr<double>(0)[0];
	double ty = tvec.ptr<double>(0)[1];
	double tz = tvec.ptr<double>(0)[2];

	//x y z 为唯一向量在相机原始坐标系下的向量值
	//也就是向量OcOw在相机坐标系下的值
	double x = tx, y = ty, z = tz;

	//进行三次反向旋转
	CodeRotateByZ(x, y, -1 * thetaz, x, y);
	CodeRotateByY(x, z, -1 * thetay, x, z);
	CodeRotateByX(y, z, -1 * thetax, y, z);


	//获得相机在世界坐标系下的位置坐标
	//即向量OcOw在世界坐标系下的值
	double Cx = x * -1;
	double Cy = y * -1;
	double Cz = z * -1;
	cout << "相机的世界坐标：" << Cx << ", " << Cy << ", " << Cz << endl;

}