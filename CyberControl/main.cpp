#include "stdafx.h"
#include "ChessCalibration.h"
#include "CapturesMatching.h"
#include "StereoPairCalibration.h"

int nStereoPreFilterSize;
int nStereoPreFilterCap;
int nStereoSADWindowSize;
int nStereoMinDisparity;
int nStereoNumDisparities;
int nStereoTextureThreshold;
int nStereoUniquenessRatio;


CvPoint lastP;

void depthMap(IplImage *img0, IplImage *img1, char *name){

	CvStereoBMState *state = cvCreateStereoBMState(CV_STEREO_BM_BASIC, 64);

	state->preFilterType = CV_STEREO_BM_NORMALIZED_RESPONSE;
	state->preFilterSize = nStereoPreFilterSize;
	state->preFilterCap = nStereoPreFilterCap;
	state->SADWindowSize = nStereoSADWindowSize;
	state->minDisparity = nStereoMinDisparity * 16;
	state->numberOfDisparities = nStereoNumDisparities * 16;
	state->textureThreshold = nStereoTextureThreshold;
	state->uniquenessRatio = nStereoUniquenessRatio;

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
	CvMat* disp_visual = cvCreateMat(size.height, size.width, CV_8U);

	cvNormalize(disp, disp_visual, 0, 255, CV_MINMAX);					//применяем полученную матрицу к диапазону 0 - 255 (самя яркая точка стала = 254, темная = 0)



	////Establish storage matrix
	//CvMat* Q = cvCreateMat(4, 4, CV_32FC3);
	//
	//CvMat* rit3dimage = cvCreateMat(disp->height, disp->width, CV_16SC3); //CV_8U   CV_16SC1
	////Call necessary function with input matrix, output matrix, 
	//cvReprojectImageTo3D(disp_visual, rit3dimage, Q);
	//cvShowImage("ReprojectedTo3D", rit3dimage);



	double framemin = 0;
	double framemax = 0;
	CvPoint p;
	cvMinMaxLoc(disp_visual, &framemin, &framemax, 0, &p);

	//cvCircle(img0, p, 20, CV_RGB(0, 255, 0), 5);

	cvShowImage(name, disp_visual);
	cvSaveImage("reslt.jpg", disp_visual);

	double *d = cvGet2D(disp_visual, p.y, p.x).val;

	cvReleaseMat(&disp);
	cvReleaseStereoBMState(&state);
	cvReleaseMat(&disp_visual);
	cvReleaseImage(&img0r);
	cvReleaseImage(&img1r);
	cvReleaseImage(&img0_g);
	cvReleaseImage(&img1_g);

}

void initParams(){
	nStereoPreFilterSize = 9;
	nStereoPreFilterCap = 29;
	nStereoSADWindowSize = 21;
	nStereoMinDisparity = 0;
	nStereoNumDisparities = 4;
	nStereoTextureThreshold = 0;
	nStereoUniquenessRatio = 20;
}

//Поиск дескрипторов и вывод изображений с найденными дескрипторами
Mat findDescriptors(IplImage *image, char* name){
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

int main(int argc, char* argv[])
{

	ChessCalibration *chessCalibration = new ChessCalibration();
	CapturesMatching *capturesMatching = new CapturesMatching();
	StereoPairCalibration *stereoPairCalibration = new StereoPairCalibration();

	CvCapture *capture0 = 0, *capture1 = 0;
	IplImage *mapx0 = 0, *mapy0 = 0, *mapx1 = 0, *mapy1 = 0;
	IplImage *frame0 = 0, *frame1 = 0;


	initParams();

	capturesMatching->openMatrix("C:\\Users\\Nikita\\Source\\Repos\\CyberControl\\Debug\\matrix.ini");

	capture0 = capturesMatching->cameraFrame(1);
	frame0 = cvQueryFrame(capture0);	//получение изображения камеры
	capture1 = capturesMatching->cameraFrame(2);
	frame1 = cvQueryFrame(capture1);
	if (!frame0 || !frame1){
		cout << "Error: error connecting to captures" << endl;
		cout << "Poweroff through 5 seconds..." << endl;
		getchar();
		return -1;
	}


	mapx0 = capturesMatching->createRemap(cvGetSize(frame0));
	mapy0 = cvCloneImage(mapx0);
	mapx1 = capturesMatching->createRemap(cvGetSize(frame1));
	mapy1 = cvCloneImage(mapx1);

	bool remapFlag = false;

	for (int i = 1; i <= argc; i++){
		if (argv[i][1] == 'c'){
			cout << "The construction undistortion maps of FIRST camera started..." << endl;
			chessCalibration->calibration(mapx0, mapy0, 0, 3, 3);
			cout << "The construction undistortion maps of SECOND camera started..." << endl;
			chessCalibration->calibration(mapx1, mapy1, 1, 3, 3);

			bool remapFlag = true;
		}

		if (argv[i][1] == 's'){
			cout << "Calibration stereo pair started..." << endl;
			stereoPairCalibration->calibration(capture0, capture1);
		}
	}

	lastP = cvPoint(320, 240);
	int rememberShift = -1;

	while (1){
		frame0 = cvQueryFrame(capture0);						//получение изображения камеры
		frame1 = cvQueryFrame(capture1);

		CvPoint2D32f center = cvPoint2D32f(frame0->width / 2, frame0->height / 2);
		double angle = 180;										// на 60 градусов по часовой стрелке
		double scale = 1;										// масштаб
		CvMat* rot_mat = cvCreateMat(2, 3, CV_32FC1);
		cv2DRotationMatrix(center, angle, scale, rot_mat);
																// выполняем вращение
		cvWarpAffine(frame0, frame0, rot_mat);
		capturesMatching->correctivePerspective(frame0);		//корректировка перспективы одной из камер для более точного результата


		if (remapFlag){
			IplImage *t0 = cvCloneImage(frame0);
			IplImage *t1 = cvCloneImage(frame1);
			cvRemap(t0, frame0, mapx0, mapy0);					// Undistort image
			cvRemap(t1, frame1, mapx1, mapy1);					// Undistort image
			cvReleaseImage(&t0);
			cvReleaseImage(&t1);
		}

		if (!argv[1]) argv[1] = "C:\\noname\\";

		//cvShowImage("fr0", frame0);					//Поиск дескрипоторов для второго метода
		//cvShowImage("fr1", frame1);

		//depthMap(frame1, frame0, argv[1]);			//вызов функции расчета карты глубины !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		/*
		IplImage *dst = 0;
		dst = cvCreateImage(cvGetSize(frame0),frame0->depth, frame0->nChannels);
		cvAddWeighted(frame0, 0.5, frame1, 0.5, 0.0, dst);
		cvShowImage("fr0-fr1", dst);
		cvReleaseImage(&dst);
		*/

		char c = cvWaitKey(33);
		if (c == 27)		{
			break;
		}
	}

	delete chessCalibration;
	delete capturesMatching;
	delete stereoPairCalibration;


	cout << "Shuting down..." << endl;
	cvDestroyAllWindows();
	cvReleaseCapture(&capture0);
	cvReleaseCapture(&capture1);

	cout << "Poweroff through 5 seconds..." << endl;
	//_sleep(5000);
	getchar();
	return 0;
}