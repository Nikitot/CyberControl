#include "stdafx.h"
#include "Common.h"

#include <iostream>

void Common::rotateImage(Mat *frame, int angle){
	Point2f center(frame->cols / 2.0F, frame->rows / 2.0F);
	angle = 180;												// на 60 градусов по часовой стрелке
	double scale = 1;											// масштаб
	Mat rot_matrix = getRotationMatrix2D(center, angle, 1.0);
	Mat rotated_img(Size(frame->size().height, frame->size().width), frame->type());

	warpAffine(*frame, *frame, rot_matrix, frame->size());		// выполняем вращение
}

//Поиск дескрипторов и вывод изображений с найденными дескрипторами
Mat Common::findDescriptors(IplImage *image, char* name){
	//-- Этап 1. Нахождение ключевых точек.

	//FeatureDetector * detector = new GFTTDetector();
	//FeatureDetector * detector = new FastFeatureDetector();
	//FeatureDetector * detector = new DenseFeatureDetector(1, 1, 0.1, 10,0, true, false);
	//FeatureDetector * detector = new StarFeatureDetector();	
	//FeatureDetector * detector = new BRISK();
	//FeatureDetector * detector = new MSER();
	//FeatureDetector * detector = new ORB();
	//FeatureDetector * detector = new SURF(1000);
	FeatureDetector *detector = new SIFT(1000);

	vector<KeyPoint> keypoints;
	detector->detect(image, keypoints);
	Mat img_keypoints, descriptors_object;
	drawKeypoints(image, keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	//-- Этап 2. Вычисление дескрипторов.
	SiftDescriptorExtractor extractor;
	extractor.compute(image, keypoints, descriptors_object);

	imshow(name, img_keypoints);
	return descriptors_object;
}

int Common::matchDescriptors(Mat ffd, Mat bfd){
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(ffd, bfd, matches);

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < ffd.rows; i++)	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;

	}

	vector< DMatch > good_matches;
	int c = 0;
	for (int i = 0; i < ffd.rows; i++)	{
		if (matches[i].distance <= 2 * min_dist)	{
			good_matches.push_back(matches[i]);
			c++;
		}
	}

	return c;
}

void Common::extractDescriptors(KeysImage *keysImage, Mat image, mutex *sinchMutex){
	// detectingkeypoints		

	//SurfFeatureDetector detector(1500);
	SurfFeatureDetector detector(1500);

	detector.detect(image, keysImage->keypoints);

	// computing descriptors
	SurfDescriptorExtractor extractor;
	Mat img_keypoints;

	extractor.compute(image, keysImage->keypoints, keysImage->descriptors);
	drawKeypoints(image, keysImage->keypoints, img_keypoints, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
}

void Common::matchDescriptorsToStereo(KeysImage *keysImage0, KeysImage *keysImage1, Mat frame[2]){
	vector< vector<DMatch> > matches;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
	matcher->knnMatch(keysImage0->descriptors, keysImage1->descriptors, matches, 50);

	//look whether the match is inside a defined area of the image
	//only 25% of maximum of possible distance
	double tresholdDist = 0.25 * sqrt(double(frame[0].size().height*frame[0].size().height + frame[0].size().width*frame[0].size().width));

	vector< DMatch > good_matches;
	vector<Point3f> map3d;
	good_matches.reserve(matches.size());
	allocator <int> res3dMat;
	Point2f from, to;

	Mat res = Mat::zeros(frame[0].size(), CV_8UC3);
	Mat imageMatches;

	for (size_t i = 0; i < matches.size(); ++i){
		for (int j = 0; j < matches[i].size(); j++)
		{
			from = keysImage0->keypoints[matches[i][j].queryIdx].pt;
			to = keysImage1->keypoints[matches[i][j].trainIdx].pt;

			//calculate local distance for each possible match
			double dist = sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));

			//save as best match if local distance is in specified area and on same height
			if (dist < tresholdDist && abs(from.y - to.y) < 5)
			{
				//set3D(result, from.x + abs(to.x - 640 - from.x) / 2, from.y + (to.y - from.y) / 2, 640 - abs(to.x - from.x));	
				Point3f coord3d;
				coord3d.y = from.y + abs(from.y - to.y) / 2;
				coord3d.x = from.x + abs((to.x) - from.x) / 2;
				coord3d.z = abs(to.x - from.x);
				map3d.push_back(coord3d);
				circle(res, Point(coord3d.x, coord3d.y), 2, Scalar(coord3d.z), 1, 8, 0);
				//cout << coord3d.y << endl;
				good_matches.push_back(matches[i][j]);
				//j = matches[i].size();
			}
		}
	}

	drawMatches(frame[0], keysImage0->keypoints, frame[1], keysImage1->keypoints, good_matches, imageMatches, Scalar(255, 255, 255));
	imshow("Matched", imageMatches);
	imshow("res",res);
}
