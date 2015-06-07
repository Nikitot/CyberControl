


class Common{

public:
	struct KeysImage{
		cv::Mat descriptors = cv::Mat(0, 0, CV_8UC(1), cv::Scalar::all(0));
		vector<cv::KeyPoint> keypoints;
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

	static void rotateImage(cv::Mat &frame, int angle);
	static void mergeImages(cv::Mat &appended, cv::Mat image1, cv::Mat image2);
};


