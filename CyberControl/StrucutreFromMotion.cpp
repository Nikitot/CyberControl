#include "stdafx.h"
#include "StrucutreFromMotion.h"
#include "SimpleViewer.h"

GLfloat yRotated;
vector<Point3f> points_3d;
Point3f camera_position;
Mat out;
int width = 640, height = 480;

Mat cam_matrix = (Mat_<double>(3, 3) <<
	6624.070862, 0, 1008.853968,
	0, 6624.995786, 1132.158299,
	0, 0, 1);

Mat dist_coeff = (Mat_<double>(1, 5) << -0.159685, 0.037437, -0.000708, -0.000551, 0.000000);

//void StrucutreFromMotion::init()
//{
//	glClearColor(0, 0, 0, 0);
//}

void StrucutreFromMotion::draw_point_cloud()
{
	vector<Point3f> this_points_3d = points_3d;
	glMatrixMode(GL_MODELVIEW);
	// clear the drawing buffer.
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(camera_position.x, camera_position.y, camera_position.z);
	// rotation about X axis
	glRotatef(0, 1.0, 0.0, 0.0);
	// rotation about Y axis
	glRotatef(yRotated, 0.0, 1.0, 0.0);
	// rotation about Z axis
	glRotatef(0, 0.0, 0.0, 1.0);
	if (points_3d.size() > 0) {
		glBegin(GL_POINTS);


		for (unsigned int i = 0; i < this_points_3d.size(); i++) {
			if (this_points_3d.at(i).x != 0 && this_points_3d.at(i).y != 0 && this_points_3d.at(i).z)
				glVertex3f(this_points_3d.at(i).x, this_points_3d.at(i).y, this_points_3d.at(i).z);
		}

		glEnd();
		glFlush();
	}
}
//
//void StrucutreFromMotion::animation()
//{
//	yRotated += 0.01;
//	draw_point_cloud();
//}
//
//void StrucutreFromMotion::reshape(int x, int y)
//{
//	if (y == 0 || x == 0) return;  //Nothing is visible then, so return
//								   //Set a new projection matrix
//	glMatrixMode(GL_PROJECTION);
//	glLoadIdentity();
//	//Angle of view:40 degrees
//	//Near clipping plane distance: 0.5
//	//Far clipping plane distance: 20.0
//
//	gluPerspective(40.0, (GLdouble)x / (GLdouble)y, 0.5, 20.0);
//	glMatrixMode(GL_MODELVIEW);
//	glViewport(0, 0, x, y);  //Use the whole window for rendering
//}

void StrucutreFromMotion::calculation_SFM_SVD(Mat &image1, Mat &image2, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points) {
	if (found_opfl_points.size() != found_opfl_points.size()) {
		return;
	}

	undistortPoints(prev_opfl_points, prev_opfl_points, cam_matrix, dist_coeff);
	undistortPoints(found_opfl_points, found_opfl_points, cam_matrix, dist_coeff);

	Mat fundamental = findFundamentalMat(prev_opfl_points, found_opfl_points, FM_RANSAC, 3.0, 0.99);
	Mat essential = cam_matrix.t() * fundamental * cam_matrix;

	/* Find the projection matrix between those two images */
	SVD svd(essential);
	static const Mat W = (Mat_<double>(3, 3) <<
		0, -1, 0,
		1, 0, 0,
		0, 0, 1);

	static const Mat W_inv = W.inv();

	Mat_<double> R1 = svd.u * W * svd.vt;
	Mat_<double> T1 = svd.u.col(2);

	Mat_<double> R2 = svd.u * W_inv * svd.vt;
	Mat_<double> T2 = -svd.u.col(2);

	static const Mat P1 = Mat::eye(3, 4, CV_64FC1);
	Mat P2 = (Mat_<double>(3, 4) <<
		R1(0, 0), R1(0, 1), R1(0, 2), T1(0),
		R1(1, 0), R1(1, 1), R1(1, 2), T1(1),
		R1(2, 0), R1(2, 1), R1(2, 2), T1(2));

	/*  Triangulate the points to find the 3D homogenous points in the world space
	Note that each column of the 'out' matrix corresponds to the 3d homogenous point
	*/
	triangulatePoints(P1, P2, prev_opfl_points, found_opfl_points, out);

	/* Since it's homogenous (x, y, z, w) coord, divide by w to get (x, y, z, 1) */
	//vector<Mat> splitted = {
	//	out.row(0) / out.row(3),
	//	out.row(1) / out.row(3),
	//	out.row(2) / out.row(3)
	//};

	//merge(splitted, out);

	vector<glm::vec3> points;
	for (int i = 0; i < out.cols; i++) {
		Mat m = out.col(i);
		float x = m.at<float>(0)/ m.at<float>(3);
		float y = m.at<float>(1)/ m.at<float>(3);
		float z = m.at<float>(2)/ m.at<float>(3);

		points.push_back(glm::vec3(x, y, z));
	}

	/*  This is a silly hack to shift the image to the origin coord (0, 0, 0)
	by applying K-mean cluster (in this case, 1 cluster), to get the cluster center ...
	*/
	Mat labels, center;
	kmeans(out.t(), 1, labels, TermCriteria(CV_TERMCRIT_ITER, 1000, 1e-5), 1, KMEANS_RANDOM_CENTERS, center);

	/*  ... and shift all the points based on the cluster center */
	for (glm::vec3& point : points) {
		point.x -= center.at<float>(0, 0);
		point.y -= center.at<float>(0, 1);
		point.z -= center.at<float>(0, 2);
	}


	/* Briefly see how our original images look like */

	Mat appended(image1.rows, image1.cols + image1.cols, image1.type());
	image1.copyTo(Mat(appended, Rect(0, 0, image1.cols, image1.rows)));
	image2.copyTo(Mat(appended, Rect(image1.cols, 0, image2.cols, image2.rows)));
	//resize(appended, appended, Size(), 0.3f, 0.3f);

	imshow("frames", appended);

	SimpleViewer viewer = SimpleViewer();
	if (viewer.init("Simple Point Cloud Viewer", 1066, 600)) {
		/* Pass the coordinates to our simple viewer */
		viewer.setVertexData(points);
		viewer.run();
	}
}

void StrucutreFromMotion::calculation_SFM_SVD_old(vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points) {
	if (found_opfl_points.size() != found_opfl_points.size()) {
		return;
	}
	double focal_length = 30;		//mm
	double pixel_length = 0.2646;	//mm

	const int size = 300;
	Matx<float, 4, size> W;
	Matx <float, 4, 1> U;
	Matx <float, 4, 4>	D;
	Matx <float, 4, size> Vt;	

	Point2f prev_mass_center = Point2f(0, 0);
	Point2f found_mass_center = Point2f(0, 0);
	for (unsigned int i = 0; i < found_opfl_points.size(); i++) {
		prev_mass_center = Point2f(prev_opfl_points.at(i).x + prev_mass_center.x
			, prev_opfl_points.at(i).y + prev_mass_center.y);

		found_mass_center = Point2f(found_opfl_points.at(i).x + found_mass_center.x
			, found_opfl_points.at(i).y + found_mass_center.y);
	}
	prev_mass_center = Point2f(prev_mass_center.x / prev_opfl_points.size(), prev_mass_center.y / prev_opfl_points.size());
	found_mass_center = Point2f(found_mass_center.x / found_opfl_points.size(), found_mass_center.y / found_opfl_points.size());

	for (unsigned int i = 0; i < found_opfl_points.size(); i++) {
		found_opfl_points.at(i) = Point2f(found_opfl_points.at(i).x - found_mass_center.x, found_opfl_points.at(i).y - found_mass_center.y);
		prev_opfl_points.at(i) = Point2f(prev_opfl_points.at(i).x - prev_mass_center.x, prev_opfl_points.at(i).y - prev_mass_center.y);
	}

	for (int i = 0; i < size; i++)
	{
		if (i < prev_opfl_points.size()) {
			W.val[i] = prev_opfl_points.at(i).x;
			W.val[i + size] = prev_opfl_points.at(i).y;
			W.val[i + size * 2] = found_opfl_points.at(i).x;
			W.val[i + size * 3] = found_opfl_points.at(i).y;
		}
		else
			break;
	}

	SVD::compute(W, U, D, Vt);

	camera_position = Point3f(U.val[0] / D.val[0], U.val[1] / D.val[5], U.val[2] / D.val[10]);

	vector<Point3f> this_points_3d;

	
	for (int i = 0; i < size; i++)
	{
		this_points_3d.push_back(Point3f(Vt.val[i], Vt.val[i + size], Vt.val[i + size * 2]));
	}
	points_3d = this_points_3d;
}

void StrucutreFromMotion::calculation_simple_Z(Mat &img1, Mat &img2, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points)
{

	//double focal_length = 30;		//mm
	//double pixel_length = 0.2646;	//mm

	points_3d = vector<Point3f>();

	double f = 300;
	double B = 1;
	int w = img1.cols;
	int h = img1.rows;

	float X = 0;
	float Y = 0;
	float Z = 0;

	for (unsigned int i = 0; i < found_opfl_points.size(); i++)
	{
		float delta_x = abs(found_opfl_points.at(i).x - prev_opfl_points.at(i).x);
		float delta_y = abs(found_opfl_points.at(i).y - prev_opfl_points.at(i).y);
		double delta = pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5);

		Z = (B * f) / delta;
		X = (prev_opfl_points.at(i).x + prev_opfl_points.at(i).y) / 2;
		Y = (prev_opfl_points.at(i).y + prev_opfl_points.at(i).y) / 2;
		points_3d.push_back(Point3f(X, Y, Z));
	}
}

void sfm_svd(vector<Point2f> left_points, vector<Point2f> right_points) {
	Mat fundamental = findFundamentalMat(left_points, right_points, FM_RANSAC, 3.0, 0.99);
	Mat essential = cam_matrix.t() * fundamental * cam_matrix;

	SVD svd(essential);
	static const Mat W = (Mat_<float>(3, 3) <<
		0, -1, 0,
		1, 0, 0,
		0, 0, 1);

	static const Mat W_inv = W.inv();

	Mat_<float> R1 = svd.u * W * svd.vt;
	Mat_<float> T1 = svd.u.col(2);

	Mat_<float> R2 = svd.u * W_inv * svd.vt;
	Mat_<float> T2 = -svd.u.col(2);

	static const Mat P1 = Mat::eye(3, 4, CV_64FC1);
	Mat P2 = (Mat_<float>(3, 4) <<
		R1(0, 0), R1(0, 1), R1(0, 2), T1(0),
		R1(1, 0), R1(1, 1), R1(1, 2), T1(1),
		R1(2, 0), R1(2, 1), R1(2, 2), T1(2));

	/*  Triangulate the points to find the 3D homogenous points in the world space
	Note that each column of the 'out' matrix corresponds to the 3d homogenous point
	*/
	Mat out;
	triangulatePoints(P1, P2, left_points, right_points, out);

	/* Since it's homogenous (x, y, z, w) coord, divide by w to get (x, y, z, 1) */
	//vector splitted = {
	//	out.row(0) / out.row(3),
	//	out.row(1) / out.row(3),
	//	out.row(2) / out.row(3)
	//};

	//merge(splitted, out);
}