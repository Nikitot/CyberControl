#include "stdafx.h"
#include "StrucutrFromMotion.h"



StrucutrFromMotion::StrucutrFromMotion()
{
}


StrucutrFromMotion::~StrucutrFromMotion()
{
}

void StrucutrFromMotion::calculation_SFM_SVD(Mat &depth_map, vector <Point2f> found_opfl_points, vector <Point2f> prev_opfl_points) {
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

	glutDisplayFunc(draw_point_cloud);
	glutReshapeFunc(reshape);
	//Set the function for the animation.
	glutIdleFunc(animation);
	glutMainLoop();

}