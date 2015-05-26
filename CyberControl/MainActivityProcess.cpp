#include "stdafx.h"
#include "Common.h"
#include "ChessCalibration.h"
#include "CapturesMatching.h"
#include "StereoPairCalibration.h"
#include "MainActivityProcess.h"
#include "StrucutreFromMotion.h"

#include <time.h>



ChessCalibration		*chessCalibration;
CapturesMatching		*capturesMatching;
StereoPairCalibration	*stereoPairCalibration;
StrucutreFromMotion		*strucutreFromMotion;
Captures				*captures;
DistortionMap			*distortionMap;

bool					camera_status[2] = { false, false };
bool					synch_flag[3] = { false, false, false };
Mat						frame[2] = { Mat(480, 640, 0), Mat(480, 640, 0) };
Mat						gray_frame[2] = { Mat(480, 640, 0), Mat(480, 640, 0) };


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

void MainActivityProcess::drawOptFlowMap(Mat flow, Mat &dst, int step)
{
	for (int y = 0; y < flow.rows / step; y++) {
		for (int x = 0; x < flow.cols / step; x++)
		{
			const Point& fxy = flow.at<Point2i>(y*step, x*step);

			Point p1(x*step, y*step);

			Point p2(x*step + fxy.x, y*step + fxy.y);

			double length_fxy = pow(pow(fxy.x, 2) + pow(fxy.y, 2), 0.5);
			double length_xy = pow(pow(step, 2) + pow(step, 2), 0.5);

			double color = length_fxy * 20;
			line(dst, Point(p1.x * 2, p1.y * 2), Point(p2.x * 2, p2.y * 2), CV_RGB(255, 255, 255));
			//circle(dst, Point(p1.x * 2, p1.y * 2), 2, Scalar(int(color), int(color), int(color)));
		}
	}
}

void MainActivityProcess::impositionOptFlow(Mat &dst, Mat &frame0, Mat &frame1) {
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

void MainActivityProcess::impositionOptFlowLK(vector<Point2f> &prev_features, vector<Point2f> &found_features, Mat prevgray, Mat gray
	, vector<float> &error, vector<uchar> &status) {
	if (camera_status[0] && camera_status[1] && prev_features.size()) {
		calcOpticalFlowPyrLK(prevgray, gray, prev_features, found_features, status, error
			, opf_parametrs.win_size, opf_parametrs.max_level, opf_parametrs.term_crit
			, opf_parametrs.lk_flags, opf_parametrs.min_eig_threshold);
	}
}

void MainActivityProcess::getReconstuctionFlow() {
	
	vector <Point2f> found_opfl_points[2];
	vector <float> error;
	vector <uchar> status;

	double RAD = 57.2957795;
	
	while (!frame[0].empty() && !frame[1].empty()) {

		while (synch_flag[2] == false);

		Mat drawRes = (frame[0] + frame[1]) / 2;
		this->impositionOptFlowLK(this->good_points[0], found_opfl_points[0], gray_frame[0], gray_frame[1], error, status);

		if (this->good_points[0].size() > 0) {

			for (unsigned int i = 0; i < found_opfl_points[0].size(); i++) {
				try {

					double dy = found_opfl_points[0].at(i).y - this->good_points[0].at(i).y;
					double dx = found_opfl_points[0].at(i).x - this->good_points[0].at(i).x;

					double delta = pow(pow(dx, 2) + pow(dy, 2), 0.5);
					double angle = atan(dy / dx) * RAD;

					if ((angle < 15 && angle > -15 && delta > 5) || delta < 5) {

						double i_color = (delta * (255 / (frame[0].cols / 15)));

						circle(drawRes, found_opfl_points[0].at(i), 1, CV_RGB(i_color, i_color, i_color), 2, 8, 0);
						line(drawRes, this->good_points[0].at(i), found_opfl_points[0].at(i), CV_RGB(i_color, i_color, i_color));

					}
					else
					{
						circle(drawRes, found_opfl_points[0].at(i), 1, CV_RGB(200, 0, 0), 2, 8, 0);
						found_opfl_points[0].erase(found_opfl_points[0].begin() + i);
						this->good_points[0].erase(this->good_points[0].begin() + i);
						i--;
					}
				}
				catch (...) {}
			}
			imshow("result", drawRes);

			if (waitKey(33) == 13)
				strucutreFromMotion->calculation_SFM_SVD(frame[0], frame[1], found_opfl_points[0], good_points[0]);
				//strucutreFromMotion->calculation_simple_Z(frame[0], frame[1], found_opfl_points[0], this->good_points[0]);
		}


		synch_flag[2] = false;

		if (waitKey(33) == 27)
			break;

	}
}

void MainActivityProcess::getCameraFramesFlow(int CAPTURE) {
	VideoCapture cap(CAPTURE);

	this->keysImage[CAPTURE] = KeysImage();

	while (cap.isOpened()) {
		camera_status[CAPTURE] = true;
		while (((synch_flag[0] != synch_flag[1]) || synch_flag[2] == true) && cap.isOpened()){
			if (waitKey(33) == 27){ break; }
		}
		if (waitKey(33) == 27){ break; }

		cap >> frame[CAPTURE];

		if (CAPTURE == 1)
			stereoPairCalibration->common->rotateImage(&frame[1], 180);		//переворачиваем изображение левой камеры
		else
			capturesMatching->correctivePerspective(&frame[0]);				//корректировка перспективы первой камеры

		if (this->remapFlag) {
			this->source = frame[0];
			remap(this->source, frame[0], Mat(distortionMap->mapx0), Mat(distortionMap->mapy0), CV_INTER_CUBIC);					// Undistort image
		}


		cvtColor(frame[CAPTURE], gray_frame[CAPTURE], COLOR_BGR2GRAY);

		this->good_points[CAPTURE].clear();
		goodFeaturesToTrack(gray_frame[CAPTURE], this->good_points[CAPTURE], this->fd_parametrs.max_сorners, this->fd_parametrs.quality_level, this->fd_parametrs.min_distance, Mat(), this->fd_parametrs.block_size, 0, this->fd_parametrs.k);

		synch_flag[CAPTURE] = !synch_flag[CAPTURE];
		synch_flag[2] = true;

	}
	camera_status[CAPTURE] = false;
	cout << "[INF] camera # " << CAPTURE << " is turned off" << endl;
}

void MainActivityProcess::setCalibration(int _argc, char* _argv[]) {
	for (int i = 1; i < _argc; i++) {
		if (_argv[i][1] == 'c') {
			CvSize size = cvSize(frame[0].cols, frame[0].rows);										//update to c++
			distortionMap->mapx0 = capturesMatching->createRemap(size);
			distortionMap->mapy0 = cvCloneImage(distortionMap->mapx0);
			distortionMap->mapx1 = capturesMatching->createRemap(size);
			distortionMap->mapy1 = cvCloneImage(distortionMap->mapx1);

			cout << "[INF] The construction undistortion maps of FIRST camera started" << endl;
			chessCalibration->calibration(distortionMap->mapx0, distortionMap->mapy0, this->CAPTURE_0, 3, 3, true);
			cout << "[INF] Construction undistortion maps of SECOND camera started" << endl;
			chessCalibration->calibration(distortionMap->mapx1, distortionMap->mapy1, this->CAPTURE_1, 3, 3, false);

			bool remapFlag = true;
		}

		if (_argv[i][1] == 's') {
			cout << "[INF] Calibration stereo pair started" << endl;
			stereoPairCalibration->calibration(captures);								//update to c++
		}
		if (_argv[i][1] == 'D') {
			this->typeStereo = false;
		}
		if (_argv[i][1] == 'B') {
			this->typeStereo = true;
		}
	}
	capturesMatching->openMatrix("./matrix.ini");
}

int MainActivityProcess::waitCameras() {
	while (!camera_status[0] || !camera_status[0]);

	if (!frame[0].cols || !frame[1].cols
		|| frame[0].cols != frame[1].cols
		|| frame[0].rows != frame[1].rows) {
		cout << "[ERR] different resilution of cameras" << endl;
		return 1;
	}
	cout << "[INF] the flow of images from both cameras started" << endl;

	return 0;
}

int MainActivityProcess::mainActivity(int _argc, char* _argv[]){

	thread cameraFlow0 = thread(&MainActivityProcess::getCameraFramesFlow, this, this->CAPTURE_0);
	thread cameraFlow1 = thread(&MainActivityProcess::getCameraFramesFlow, this, this->CAPTURE_1);

	cameraFlow0.detach();
	cameraFlow1.detach();

	if (this->waitCameras()) 
		return 1;
	this->setCalibration(_argc, _argv);

	//glutInit(&argc, argv);
	////we initizlilze the glut. functions
	//glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	//glutInitWindowPosition(frame[0].cols / 2, frame[0].rows / 2);
	//glutCreateWindow("Point Cloud");
	//strucutreFromMotion->init();

	thread reconstructionFlow(&MainActivityProcess::getReconstuctionFlow, this);
	reconstructionFlow.detach();

	//glutDisplayFunc(strucutreFromMotion->draw_point_cloud);
	//glutReshapeFunc(strucutreFromMotion->reshape);
	////Set the function for the animation.
	//glutIdleFunc(strucutreFromMotion->animation);
	//glutMainLoop();

	while (1){
		if (waitKey(33) == 27) break;
	}

	cout << "[INF] shuting down..." << endl;
	cvDestroyAllWindows();
	return 0;
}

int main(int argc, char* argv[]) {
	MainActivityProcess map;
	return map.mainActivity(argc, argv);
}