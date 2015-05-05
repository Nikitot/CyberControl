#include "stdafx.h"
#include <gl/glew.h>
#include <gl/freeglut.h>
#include "StrucutreFromMotion.h"

GLfloat yRotated;
vector<Point3f> points_3d;
Point3f camera_position;

void draw_point_cloud()
{
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

	glBegin(GL_POINTS);

	for (unsigned int i = 0; i < points_3d.size(); i++) {
		if (points_3d.at(i).x != 0 && points_3d.at(i).y != 0 && points_3d.at(i).z)
			glVertex3f(points_3d.at(i).x, points_3d.at(i).y, points_3d.at(i).z);
	}
	glEnd();
	glFlush();
}

void animation()
{
	yRotated += 0.01;
	draw_point_cloud();
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

void StrucutreFromMotion::calculation_SFM_SVD(vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points) {
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
	points_3d = vector<Point3f>();

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

	camera_position = Point3f(U.val[0], U.val[1], U.val[2]);

	for (int i = 0; i < size; i++)
	{
		points_3d.push_back(Point3f(Vt.val[i], Vt.val[i + size], Vt.val[i + size * 2]));
	}

	//glutDisplayFunc(draw_point_cloud);
	//glutReshapeFunc(reshape);
	//Set the function for the animation.
	//glutIdleFunc(animation);
	//glutMainLoop();

}