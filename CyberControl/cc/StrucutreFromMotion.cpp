#include "stdafx.h"
#include "StrucutreFromMotion.h"
#include "SimpleViewer.h"
#include "Common.h"
#include "GL/freeglut.h"

float yRotated = 0.;

vector<cv::Point3f> mesh;
vector<cv::Point3f> color_mesh;



void init()
{
	glClearColor(0, 0, 0, 0);
	glPointSize(3);
}

void draw_point_cloud()
{

	glMatrixMode(GL_MODELVIEW);
	// clear the drawing buffer.
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0, 0, -10);
	// rotation about X axis
	glRotatef(0, 1.0, 0.0, 0.0);
	// rotation about Y axis
	glRotatef(yRotated, 0.0, 1.0, 0.0);
	// rotation about Z axis
	glRotatef(0, 0.0, 0.0, 1.0);

	glBegin(GL_POINTS);

	for (unsigned int i = 0; i < mesh.size(); i++){
		if (mesh.at(i).x != 0 && mesh.at(i).y != 0 && mesh.at(i).z)
			glVertex3f(mesh.at(i).x, -mesh.at(i).y, mesh.at(i).z);
			glColor3f(color_mesh.at(i).x, color_mesh.at(i).y, color_mesh.at(i).z);
	}

	glEnd();
	glFlush();
}

void reshape(int x, int y)
{
	if (y == 0 || x == 0) return;  //Nothing is visible then, so return
	//Set a new projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//Angle of view:40 degrees
	//Near clipping plane distance: 0.5
	//Far clipping plane distance: 20.0

	gluPerspective(40.0, (GLdouble)x / (GLdouble)y, 0.5, 20.0);
	glMatrixMode(GL_MODELVIEW);
	glViewport(0, 0, x, y);  //Use the whole window for rendering
}

void animation()
{
	yRotated += 0.02;
	draw_point_cloud();
}

void spinDisplay(void)
{
	yRotated += 2.0;
	if (yRotated > 360.0)
		yRotated -= 360.0;
	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y)
{
	switch (button) {
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
			glutIdleFunc(spinDisplay);
		break;
	case GLUT_MIDDLE_BUTTON:
		if (state == GLUT_DOWN)
			glutIdleFunc(NULL);
		break;
	default:
		break;
	}
}

void StrucutreFromMotion::calculation_SFM_SVD(cv::Mat &image1, cv::Mat &image2, vector <cv::Point2f> found_opfl_points, vector <cv::Point2f> prev_opfl_points) {
	if (found_opfl_points.size() != found_opfl_points.size()) {
		return;
	}
	cv::Mat out;

	cv::Mat cam_Matrix = (cv::Mat_<double>(3, 3) <<
		6624.070862, 0, 1008.853968,
		0, 6624.995786, 1132.158299,
		0, 0, 1);

	//cv::Mat dist_coeff = (cv::Mat_<double>(1, 5) << -0.159685, 0.037437, -0.000708, -0.000551, 0.000000);
	//cam_cv::Matrix << 300., 0., 200., 0, 300., 100., 0., 0., 1.;

	//double cx = image1.cols / 2.;
	//double cy = image1.rows / 2.;
	//double fxy = cx / tan(60. / 2. * 3.14159265 / 180.);

	//cv::Mat_ <double>cam_cv::Matrix(3, 3);

	//cam_cv::Matrix << fxy, 0., cx, 0, fxy, cy, 0., 0., 1.;
	cv::Mat_ <double>dist_coeff(1, 5);

	undistortPoints(prev_opfl_points, prev_opfl_points, cam_Matrix, dist_coeff);
	undistortPoints(found_opfl_points, found_opfl_points, cam_Matrix, dist_coeff);

	cv::Mat fundamental = findFundamentalMat(prev_opfl_points, found_opfl_points, cv::FM_RANSAC, 3.0, 0.99);
	cv::Mat essential = cam_Matrix.t() * fundamental * cam_Matrix;

	/* Find the projection cv::Matrix between those two images */
	cv::SVD svd(essential);
	cv::Mat W = (cv::Mat_<double>(3, 3) <<
		0, -1, 0,
		1, 0, 0,
		0, 0, 1);

	cv::Mat W_inv = W.inv();

	cv::Mat_<double> R1 = svd.u * W * svd.vt;
	cv::Mat_<double> T1 = svd.u.col(2);

	cv::Mat_<double> R2 = svd.u * W_inv * svd.vt;
	cv::Mat_<double> T2 = -svd.u.col(2);

	cv::Mat P1 = cv::Mat::eye(3, 4, CV_64FC1);
	cv::Mat P2 = (cv::Mat_<double>(3, 4) <<
		R1(0, 0), R1(0, 1), R1(0, 2), T1(0),
		R1(1, 0), R1(1, 1), R1(1, 2), T1(1),
		R1(2, 0), R1(2, 1), R1(2, 2), T1(2));

	/*  Triangulate the points to find the 3D homogenous points in the world space
	Note that each column of the 'out' cv::Matrix corresponds to the 3d homogenous point
	*/
	triangulatePoints(P1, P2, prev_opfl_points, found_opfl_points, out);

	//merge(splitted, out);

	vector<glm::vec3> points;
	for (int i = 0; i < out.cols; i++) {
		cv::Mat m = out.col(i);
		float x = m.at<float>(0)/ m.at<float>(3);
		float y = m.at<float>(1)/ m.at<float>(3);
		float z = m.at<float>(2)/ m.at<float>(3);

		points.push_back(glm::vec3(x, y, z));
	}

	/*  This is a silly hack to shift the image to the origin coord (0, 0, 0)
	by applying K-mean cluster (in this case, 1 cluster), to get the cluster center ...
	*/
	cv::Mat labels, center;
	kmeans(out.t(), 1, labels, cv::TermCriteria(CV_TERMCRIT_ITER, 1000, 1e-5), 1, cv::KMEANS_RANDOM_CENTERS, center);

	/*  ... and shift all the points based on the cluster center */
	for (glm::vec3& point : points) {
		point.x -= center.at<float>(0, 0);
		point.y -= center.at<float>(0, 1);
		point.z -= center.at<float>(0, 2);
	}
	
	/* Briefly see how our original images look like */
	cv::Mat appended;
	Common::mergeImages(appended, image1, image2);

	imshow("input SFM", appended);

	//SimpleViewer viewer = SimpleViewer();
	//if (viewer.init("Simple Point Cloud Viewer", 1066, 600)) {
	//	/* Pass the coordinates to our simple viewer */
	//	viewer.setVertexData(points);
	//	viewer.run();
	//}
}


void StrucutreFromMotion::initDrawGL(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitWindowSize(800, 600);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Point Cloud");
	init();
}

void StrucutreFromMotion::simpleDrawGL()
{
	glutDisplayFunc(draw_point_cloud);
	glutReshapeFunc(reshape);
	//Set the function for the animation.
	//glutMouseFunc(mouse);
	glutIdleFunc(animation);
	glutMainLoop();
}

void StrucutreFromMotion::calculation_simple_Z(cv::Mat img1, cv::Mat img2, vector <cv::Point2f> found_opfl_points, vector <cv::Point2f> prev_opfl_points)
{	

	int f = 300;
	int B = 1;
	float X = 0;
	float Y = 0;
	float Z = 0;

	// количество вершин в нашей геометрии, у нас простой треугольник
	const int vertexCount = found_opfl_points.size();

	// подготовим данные для вывода треугольника, всего 3 вершины
	//static float *pointsMesh = (float *)malloc(sizeof(int)* found_opfl_points.size() * 6);



	for (unsigned int i = 0; i < vertexCount; i++)
	{
		double delta_x = abs(found_opfl_points.at(i).x - prev_opfl_points.at(i).x);
		double delta_y = abs(found_opfl_points.at(i).y - prev_opfl_points.at(i).y);
		double delta = pow(pow(delta_x, 2) + pow(delta_y, 2), 0.5);

		if (delta != 0) 
			Z = (B * f) / (float)delta;
			

		X = found_opfl_points.at(i).x - img1.cols/2;
		Y = found_opfl_points.at(i).y - img1.rows/2;
		
		//point.push_back(X / 300.);
		//point.push_back(Y / 300.);
		//point.push_back(-Z);
		mesh.push_back(cv::Point3f(X / 200., Y / 200., Z));
		
		cv::Vec3b color = img2.at<cv::Vec3b>(found_opfl_points.at(i).y, found_opfl_points.at(i).x);
		color_mesh.push_back(cv::Point3f(color.val[2] / 255., color.val[1] / 255., color.val[0] / 255.));
	}


	cv::Mat appended;
	Common::mergeImages(appended, img1, img2);
	imwrite("2images.jpg", appended);


	//// создадим и настроим камеру
	//setPointsMesh(mesh);

	//LoggerCreate("visual.log");
	//if (!GLWindowCreate("Visual stereo image", 800, 600, false));
	//GLWindowMainLoop();

	//GLWindowDestroy();
	//LoggerDestroy();

	//we initizlilze the glut. functions



}
