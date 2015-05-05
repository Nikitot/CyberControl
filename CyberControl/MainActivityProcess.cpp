#include "stdafx.h"
#include "Common.h"
#include "ChessCalibration.h"
#include "CapturesMatching.h"
#include "StereoPairCalibration.h"
#include "MainActivityProcess.h"
#include "StrucutreFromMotion.h"


ChessCalibration		*chessCalibration;
CapturesMatching		*capturesMatching;
StereoPairCalibration	*stereoPairCalibration;
StrucutreFromMotion		*strucutreFromMotion;


Captures				*captures;
DistortionMap			*distortionMap;
Mat						frame[2] = { Mat(480, 640, 0), Mat(480, 640, 0) };
Mat						gray_frame[2] = { Mat(480, 640, 0), Mat(480, 640, 0) };
bool					synch_flag[3] = { false, false, false };
bool					camera_status[2] = { false, false };
vector<Point2f>			good_points[2];

double RAD = 57.2957795;

struct opt_flow_parametrs {
	Size win_size = Size(11, 11);
	int max_level = 10;
	TermCriteria term_crit = TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	int deriv_lamda = 0;
	int lk_flags = 0;
	double min_eig_threshold = 0.01;
};

struct feature_detect_parametrs {
	Size win_size = Size(5, 5);
	int max_сorners = 1000;
	double quality_level = 0.001;
	double min_distance = 15;
	int block_size = 3;
	double k = 0.05;
};

opt_flow_parametrs opf_parametrs;
feature_detect_parametrs fd_parametrs;


MainActivityProcess::MainActivityProcess() {
	chessCalibration = new ChessCalibration();
	capturesMatching = new CapturesMatching();
	stereoPairCalibration = new StereoPairCalibration();

	captures = new Captures();
	distortionMap = new DistortionMap();

	stereoPreFilterSize = 9;
	stereoPreFilterCap = 10;
	stereoSADWindowSize = 21;
	stereoMinDisparity = 0;
	stereoNumDisparities = 4;
	stereoTextureThreshold = 0;
	stereoUniquenessRatio = 30;
}

MainActivityProcess::~MainActivityProcess() {
	delete chessCalibration;
	delete capturesMatching;
	delete stereoPairCalibration;
	delete captures;
	delete distortionMap;
}

void MainActivityProcess::createDepthMapFSCBM(IplImage *img0, IplImage *img1, CvMat *disp_visual) {
	CvStereoBMState *state = cvCreateStereoBMState(CV_STEREO_BM_BASIC, 64);
	state->preFilterType = CV_STEREO_BM_NORMALIZED_RESPONSE;
	state->preFilterSize = stereoPreFilterSize;
	state->preFilterCap = stereoPreFilterCap;
	state->SADWindowSize = stereoSADWindowSize;
	state->minDisparity = stereoMinDisparity * 16;
	state->numberOfDisparities = stereoNumDisparities * 16;
	state->textureThreshold = stereoTextureThreshold;
	state->uniquenessRatio = stereoUniquenessRatio;

	state->speckleRange = 18;
	state->disp12MaxDiff = 1;

	CvSize size = cvGetSize(img0);

	IplImage *img0_g = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage *img1_g = cvCreateImage(size, IPL_DEPTH_8U, 1);
	CvMat* disp = cvCreateMat(size.height, size.width, CV_16S);

	IplImage *img0r = cvCreateImage(size, IPL_DEPTH_8U, 1);
	IplImage  *img1r = cvCreateImage(size, IPL_DEPTH_8U, 1);

	cvCvtColor(img0, img0_g, CV_BGR2GRAY);
	cvCvtColor(img1, img1_g, CV_BGR2GRAY);

	cvFindStereoCorrespondenceBM(img0_g, img1_g, disp, state);
	cvNormalize(disp, disp_visual, 0, 255, CV_MINMAX);					//примен€ем полученную матрицу к диапазону 0 - 255 (сам€ €рка€ точка стала = 254, темна€ = 0)


	///*********************Establish storage matrix*********************/
	//CvMat* Q = cvCreateMat(4, 4, CV_32FC3);
	//
	//CvMat* rit3dimage = cvCreateMat(disp->height, disp->width, CV_16SC3); //CV_8U   CV_16SC1
	////Call necessary function with input matrix, output matrix, 
	//cvReprojectImageTo3D(disp_visual, rit3dimage, Q);
	//cvShowImage("ReprojectedTo3D", rit3dimage);
	///*****************************************************************/

	double framemin = 0;
	double framemax = 0;
	CvPoint p;
	cvMinMaxLoc(disp_visual, &framemin, &framemax, 0, &p);

	//cvCircle(img0, p, 20, CV_RGB(0, 255, 0), 5);

	//cvShowImage(name, disp_visual);
	//cvSaveImage("reslt.jpg", disp_visual);

	double *d = cvGet2D(disp_visual, p.y, p.x).val;

	cvReleaseMat(&disp);
	cvReleaseStereoBMState(&state);
	cvReleaseImage(&img0r);
	cvReleaseImage(&img1r);
	cvReleaseImage(&img0_g);
	cvReleaseImage(&img1_g);

}

void MainActivityProcess::mergeDisps(CvMat* dispVisual1, CvMat* dispVisual2, CvSize size) {
	for (int i = 0; i < size.height; i++) {
		for (int j = 0; j < size.width; j++) {
			CvScalar value1 = cvGet2D(dispVisual1, i, j);
			CvScalar value2 = cvGet2D(dispVisual2, i, j);

			value1.val[0] = value1.val[0] / 2;
			value1.val[1] = value1.val[1] / 2;
			value1.val[2] = value1.val[2] / 2;

			value2.val[0] = 127 + (255 - value2.val[0]) / 2;
			value2.val[1] = 127 + (255 - value2.val[1]) / 2;
			value2.val[2] = 127 + (255 - value2.val[2]) / 2;

			if (value1.val[0]) {
				cvSet2D(dispVisual2, i, j, value2);
				cvSet2D(dispVisual2, i, j, value1);
			}
		}
	}
}

void getCameraFlow(int CAPTURE, VideoCapture *cap, MainActivityProcess *mp) {

	cap = new VideoCapture(CAPTURE);
	mp->keysImage[CAPTURE] = KeysImage();

	if (cap->isOpened()) {
		cout << "camera # " << CAPTURE << "is statred" << endl;
	}
	camera_status[CAPTURE] = true;
	while (cap->isOpened()) {

		while ((synch_flag[0] != synch_flag[1]) || synch_flag[2] == true) {
		}
		*cap >> frame[CAPTURE];

		if (CAPTURE == 1)
			stereoPairCalibration->common->rotateImage(&frame[1], 180);		//переворачиваем изображение левой камеры
		else
			capturesMatching->correctivePerspective(&frame[0]);				//корректировка перспективы первой камеры

		if (mp->remapFlag) {
			mp->source = frame[0];
			remap(mp->source, frame[0], Mat(distortionMap->mapx0), Mat(distortionMap->mapy0), CV_INTER_CUBIC);					// Undistort image
		}
		//stereoPairCalibration->common->extractDescriptors(&mp->keysImage[CAPTURE], frame[CAPTURE], &sinchMutex);				//Break если все близко, исправить

		cvtColor(frame[CAPTURE], gray_frame[CAPTURE], COLOR_BGR2GRAY);

		good_points[CAPTURE].clear();
		goodFeaturesToTrack(gray_frame[CAPTURE], good_points[CAPTURE], fd_parametrs.max_сorners, fd_parametrs.quality_level, fd_parametrs.min_distance, Mat(), fd_parametrs.block_size, 0, fd_parametrs.k);

		synch_flag[CAPTURE] = !synch_flag[CAPTURE];
		synch_flag[2] = true;
	}
	camera_status[CAPTURE] = false;
	cout << "camera # " << CAPTURE << " is turned off" << endl;
}

void drawOptFlowMap(Mat flow, Mat &dst, int step)
{
	for (int y = 0; y < flow.rows / step; y++) {
		for (int x = 0; x < flow.cols / step; x++)
		{
			const Point2f& fxy = flow.at<Point2f>(y*step, x*step);

			Point p1(x*step, y*step);
			Point p2(round(x*step + fxy.x), round(y*step + fxy.y));

			float length_fxy = pow(pow(fxy.x, 2) + pow(fxy.y, 2), 0.5);
			float length_xy = pow(pow(step, 2) + pow(step, 2), 0.5);

			float color = length_fxy * 20;
			line(dst, Point(p1.x * 2, p1.y * 2), Point(p2.x * 2, p2.y * 2), CV_RGB(255, 255, 255));
			//circle(dst, Point(p1.x * 2, p1.y * 2), 2, Scalar(int(color), int(color), int(color)));
		}
	}
}

void impositionOptFlow(Mat &dst, Mat &frame0, Mat &frame1) {
	if (!camera_status[0] || !camera_status[1])
		return;
	Mat flow, gray0, gray1;
	cvtColor(frame0, gray0, COLOR_BGR2GRAY);
	resize(gray0, gray0, Size(frame0.cols / 2, frame0.rows / 2));

	cvtColor(frame1, gray1, COLOR_BGR2GRAY);
	resize(gray1, gray1, Size(frame1.cols / 2, frame1.rows / 2));

	calcOpticalFlowFarneback(gray0, gray1, flow, 0.5, 7, 10, 3, 7, 1.5, OPTFLOW_FARNEBACK_GAUSSIAN);
	drawOptFlowMap(flow, dst, 7);
}

void impositionOptFlowLK(vector<Point2f> &prev_features, vector<Point2f> &found_features, Mat prevgray, Mat gray
	, vector<float> &error, vector<uchar> &status) {
	if (camera_status[0] && camera_status[1] && prev_features.size()) {
		calcOpticalFlowPyrLK(prevgray, gray, prev_features, found_features, status, error
			, opf_parametrs.win_size, opf_parametrs.max_level, opf_parametrs.term_crit
			, opf_parametrs.lk_flags, opf_parametrs.min_eig_threshold);
	}
}

void getReconstuctionFlow() {
	vector <Point2f> found_opfl_points[2];
	vector <float> error;
	vector <uchar> status;

	while (!frame[0].empty() && !frame[1].empty()) {

		while (synch_flag[2] == false) {}
		Mat drawRes = (frame[0] + frame[1]) / 2;
		impositionOptFlowLK(good_points[0], found_opfl_points[0], gray_frame[0], gray_frame[1], error, status);

		if (good_points[0].size() > 0) {

			for (unsigned int i = 0; i < found_opfl_points[0].size(); i++) {
				try {

					double dy = found_opfl_points[0].at(i).y - good_points[0].at(i).y;
					double dx = found_opfl_points[0].at(i).x - good_points[0].at(i).x;

					double delta = pow(pow(dx, 2) + pow(dy, 2), 0.5);
					double angle = atan(dy / dx) * RAD;

					if (angle < 10 && angle > -10) {

						float i_color = (delta * (255 / (frame[0].cols / 15)));

						circle(drawRes, found_opfl_points[0].at(i), 1, CV_RGB(i_color, i_color, i_color), 2, 8, 0);
						line(drawRes, good_points[0].at(i), found_opfl_points[0].at(i), CV_RGB(i_color, i_color, i_color));
					}
					else
					{
						circle(drawRes, found_opfl_points[0].at(i), 1, CV_RGB(200, 0, 0), 2, 8, 0);
					}
				}
				catch (...) {}
			}

			strucutreFromMotion->calculation_SFM_SVD(found_opfl_points[0], good_points[0]);

			imshow("result", drawRes);
			imshow("frame0", frame[0]);
			imshow("frame1", frame[1]);
		}


		synch_flag[2] = false;

		if (waitKey(33) == 27)
			break;
	}
}

void setCalibration(int _argc, char* _argv[], MainActivityProcess *mp) {
	for (int i = 1; i < _argc; i++) {
		if (_argv[i][1] == 'c') {
			CvSize size = cvSize(frame[0].cols, frame[0].rows);										//update to c++
			distortionMap->mapx0 = capturesMatching->createRemap(size);
			distortionMap->mapy0 = cvCloneImage(distortionMap->mapx0);
			distortionMap->mapx1 = capturesMatching->createRemap(size);
			distortionMap->mapy1 = cvCloneImage(distortionMap->mapx1);

			cout << "The construction undistortion maps of FIRST camera started..." << endl;
			chessCalibration->calibration(distortionMap->mapx0, distortionMap->mapy0, mp->CAPTURE_0, 3, 3, true);
			cout << "The construction undistortion maps of SECOND camera started..." << endl;
			chessCalibration->calibration(distortionMap->mapx1, distortionMap->mapy1, mp->CAPTURE_1, 3, 3, false);

			bool remapFlag = true;
		}

		if (_argv[i][1] == 's') {
			cout << "Calibration stereo pair started..." << endl;
			stereoPairCalibration->calibration(captures);								//update to c++
		}
		if (_argv[i][1] == 'D') {
			mp->typeStereo = false;
		}
		if (_argv[i][1] == 'B') {
			mp->typeStereo = true;
		}
	}
	capturesMatching->openMatrix("./matrix.ini");
}

int waitCameras() {
	while(!camera_status[0] || !camera_status[0]) {}

	if (!frame[0].cols || !frame[1].cols
		|| frame[0].cols != frame[1].cols
		|| frame[0].rows != frame[1].rows) {
		cout << "Error: different resilution of cameras" << endl;
		//Sleep(300);
		return 1;
	}
}

void

int main(int argc, char* argv[]) {
	MainActivityProcess *mp = new MainActivityProcess();
	VideoCapture cap0, cap1;

	thread cameraFlow0(&getCameraFlow, mp->CAPTURE_0, &cap0, mp);
	thread cameraFlow1(&getCameraFlow, mp->CAPTURE_1, &cap1, mp);

	thread reconstructionFlow(&getReconstuctionFlow);

	cameraFlow0.detach();
	cameraFlow1.detach();

	waitCameras();

	setCalibration(argc, argv, mp);

	reconstructionFlow.detach();	



	cout << "Shuting down..." << endl;
	cap0.release();
	cap1.release();
	cvDestroyAllWindows();
	delete mp;
	return 0;
}