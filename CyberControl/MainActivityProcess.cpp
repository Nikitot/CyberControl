#include "stdafx.h"
#include "Common.h"
#include "ChessCalibration.h"
#include "CapturesMatching.h"
#include "StereoPairCalibration.h"
#include "MainActivityProcess.h"


ChessCalibration		*chessCalibration;
CapturesMatching		*capturesMatching;
StereoPairCalibration	*stereoPairCalibration;

Captures				*captures;
DistortionMap			*distortionMap;
Frames					*frames;


MainActivityProcess::MainActivityProcess(){
	chessCalibration = new ChessCalibration();
	capturesMatching = new CapturesMatching();
	stereoPairCalibration = new StereoPairCalibration();

	captures = new Captures();
	distortionMap = new DistortionMap();
	frames = new Frames;

	stereoPreFilterSize = 9;
	stereoPreFilterCap = 10;
	stereoSADWindowSize = 21;
	stereoMinDisparity = 0;
	stereoNumDisparities = 4;
	stereoTextureThreshold = 0;
	stereoUniquenessRatio = 30;
}

MainActivityProcess::~MainActivityProcess(){
	delete chessCalibration;
	delete capturesMatching;
	delete stereoPairCalibration;

	delete captures;
	delete distortionMap;
	delete frames;
}

void MainActivityProcess::createDepthMapFSCBM(IplImage *img0, IplImage *img1, CvMat *disp_visual){
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

void MainActivityProcess::mergeDisps(CvMat* dispVisual1, CvMat* dispVisual2, CvSize size){
	for (int i = 0; i < size.height; i++){
		for (int j = 0; j < size.width; j++){
			CvScalar value1 = cvGet2D(dispVisual1, i, j);
			CvScalar value2 = cvGet2D(dispVisual2, i, j);

			value1.val[0] = value1.val[0] / 2;
			value1.val[1] = value1.val[1] / 2;
			value1.val[2] = value1.val[2] / 2;

			value2.val[0] = 127 + (255 - value2.val[0]) / 2;
			value2.val[1] = 127 + (255 - value2.val[1]) / 2;
			value2.val[2] = 127 + (255 - value2.val[2]) / 2;

			if (value1.val[0]){
				cvSet2D(dispVisual2, i, j, value2);
				cvSet2D(dispVisual2, i, j, value1);
			}
		}
	}
}



int MainActivityProcess::mainActivity(int _argc, char* _argv[]){


	VideoCapture cap0(CAPTURE_0), cap1(CAPTURE_1);
	cap0 >> frames->frame0;
	cap1 >> frames->frame1;


	if (!cap0.isOpened() || !cap1.isOpened()){
		cout << "Error: filed conection to devices: " << endl;
		if (!cap0.isOpened())
			cout << "\tcamera #0" << endl;
		if (!cap1.isOpened())
			cout << "\tcamera #1 " << endl;

		return -1;
	}
	if (frames->frame0.cols != frames->frame1.cols
		|| frames->frame0.rows != frames->frame1.rows){
		cout << "Error: different resilution of cameras" << endl;
		return -1;
	}
	int width = frames->frame0.cols,
		height = frames->frame0.rows;

	Mat dispVisual0 = frames->frame0;
	Mat dispVisual1 = frames->frame1;

	bool remapFlag = false;

	for (int i = 1; i < _argc; i++){
		if (_argv[i][1] == 'c'){
			CvSize size = cvSize(width, height);										//update to c++
			distortionMap->mapx0 = capturesMatching->createRemap(size);
			distortionMap->mapy0 = cvCloneImage(distortionMap->mapx0);
			distortionMap->mapx1 = capturesMatching->createRemap(size);
			distortionMap->mapy1 = cvCloneImage(distortionMap->mapx1);

			cout << "The construction undistortion maps of FIRST camera started..." << endl;
			chessCalibration->calibration(distortionMap->mapx0, distortionMap->mapy0, CAPTURE_0, 3, 3, true);
			cout << "The construction undistortion maps of SECOND camera started..." << endl;
			chessCalibration->calibration(distortionMap->mapx1, distortionMap->mapy1, CAPTURE_1, 3, 3, false);

			bool remapFlag = true;
		}

		if (_argv[i][1] == 's'){
			cout << "Calibration stereo pair started..." << endl;
			stereoPairCalibration->calibration(captures);								//update to c++
		}
	}

	capturesMatching->openMatrix("C:\\Users\\Nikita\\Source\\Repos\\CyberControl\\Debug\\matrix.ini");

	while (1){
		cap0 >> frames->frame0;
		cap1 >> frames->frame1;

		stereoPairCalibration->common->rotateImage(&frames->frame1, 180);		//переворачиваем изображение левой камеры
		capturesMatching->correctivePerspective(&frames->frame0);				//корректировка перспективы одной из камер дл€ более точного результата

		if (remapFlag){
			source = frames->frame0;
			remap(source, frames->frame0, Mat(distortionMap->mapx0), Mat(distortionMap->mapy0), CV_INTER_CUBIC);					// Undistort image
			source = frames->frame1;
			remap(source, frames->frame1, Mat(distortionMap->mapx1), Mat(distortionMap->mapy1), CV_INTER_CUBIC);					// Undistort image
		}

		if (!_argv[1]) _argv[1] = "C:\\noname\\";

		//cvShowImage("fr0", frames->frame0);
		//cvShowImage("fr1", frames->frame1);

		//createDepthMapFSCBM(frames->frame1, frames->frame0, dispVisual1);			//вызов функции расчета карты глубины в близи
		//createDepthMapFSCBM(frames->frame0, frames->frame1, dispVisual2);			//вызов функции расчета карты глубины на рассто€нии
		//cvShowImage("dispVisual1", dispVisual1);
		//cvShowImage("dispVisual2", dispVisual2);
		//mergeDisps(dispVisual1, dispVisual2, size);

		KeysImage keysImage0, keysImage1;

		stereoPairCalibration->common->extractDescriptorsSURF(&keysImage0, frames->frame0, "frame0");
		stereoPairCalibration->common->extractDescriptorsSURF(&keysImage1, frames->frame1, "frame1");



		char c = cvWaitKey(33);
		if (c == 27)		{
			cout << "Shuting down..." << endl;
			break;
		}

	}
	cvDestroyAllWindows();
	return 0;
}