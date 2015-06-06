#pragma once

class Common{

public:
	struct KeysImage{
		Mat descriptors = Mat(0, 0, CV_8UC(1), Scalar::all(0));
		vector<KeyPoint> keypoints;
	};

	struct Captures{
		CvCapture
		*capture0 = 0,
		*capture1 = 0;
		//public:
		//~Captures();
	};

	struct DistortionMap{
		IplImage
		*mapx0 = 0,
		*mapy0 = 0,
		*mapx1 = 0,
		*mapy1 = 0;
	};

	static void rotateImage(Mat &frame, int angle);
	static void mergeImages(Mat &appended, Mat image1, Mat image2);
};


