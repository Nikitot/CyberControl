#pragma once
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

struct Frames{
	Mat
	frame0,
	frame1;
};

class Common{
public:
	void rotateImage(Mat *frame, int angle);
	Mat	findDescriptors(IplImage *image, char* name);
	int matchDescriptors(Mat ffd, Mat bfd);
	void extractDescriptorsSURF(KeysImage *keysImage, Mat image, char* name);
	void matchDescriptorsToStereo(KeysImage *keysImage0, KeysImage *keysImage1, Frames *frames);
};


