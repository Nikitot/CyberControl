#include "stdafx.h"
#include "Common.h"
#include "StereoPairCalibration.h"


int matrix[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
int controlCount = 0;


int* StereoPairCalibration::minCoordinats(CvPoint2D32f *corners, int board_w, int board_h){
	int *vals = new int[4];

	vals[0] = (int)corners[0].x;
	vals[1] = (int)corners[0].y;
	vals[2] = (int)corners[1].x;
	vals[3] = (int)corners[1].y;

	int cit = 0;

	for (int i = 0; i < board_w; i++){
		for (int j = 0; j < board_h; j++){
			int x = (int)corners[cit].x;					//���������� ������� �����
			int y = (int)corners[cit].y;

			vals[0] = (int)MIN(vals[0], x);				//������� �� x
			vals[1] = (int)MIN(vals[1], y);				//������� �� y

			vals[2] = (int)MAX(vals[2], x);				//�������� �� x
			vals[3] = (int)MAX(vals[3], y);				//�������� �� y

			cit++;
		}
	}
	return vals;
}

void StereoPairCalibration::correctivePerspective(IplImage *frame){

	int width = frame->width;
	int height = frame->height;

	// �����
	CvPoint2D32f srcQuad[4], dstQuad[4];

	// ������� ��������������
	CvMat* warp_matrix = cvCreateMat(3, 3, CV_32FC1);

	// ����� �����
	srcQuad[0].x = 0;						srcQuad[0].y = 0;						//src Top left
	srcQuad[1].x = (float)width - 1;		srcQuad[1].y = 0;							//src Top right
	srcQuad[2].x = 0;						srcQuad[2].y = (float)height - 1;		//src Bottom left
	srcQuad[3].x = (float)width - 1;		srcQuad[3].y = (float)height - 1;			//src Bot right

	dstQuad[0].x = (float)-matrix[0];			dstQuad[0].y = (float)matrix[1];					//dst Top left       
	dstQuad[1].x = (float)(width + matrix[2]);	dstQuad[1].y = (float)matrix[3];					//dst Top right        
	dstQuad[2].x = (float)-matrix[4];			dstQuad[2].y = (float)(height + matrix[5]);			//dst Bot left          
	dstQuad[3].x = (float)(width + matrix[6]);	dstQuad[3].y = (float)(height + matrix[7]);			//dst Bot right

	// �������� ������� ��������������
	cvGetPerspectiveTransform(srcQuad, dstQuad, warp_matrix);
	// �������������� �����������
	cvWarpPerspective(frame, frame, warp_matrix);

}

void StereoPairCalibration::shiftImage(int dif, int k1, int k2){

	if (dif < 0){
		matrix[k1]++;
		matrix[k2]++;
		controlCount = 0;
	}
	else if (dif > 0){
		matrix[k1]--;
		matrix[k2]--;
		controlCount = 0;
	}
	else{
		//cout << "Write matrix" << endl;
		ofstream out("./matrix.ini"); // ��������� �����
		for (int i = 0; i < 8; i++) out << matrix[i] << " "; //������
		out.close();//�� ���������� ��������� ����
		controlCount++;
	}
}

bool StereoPairCalibration::findChess(IplImage *frame0, IplImage *frame1, char *name){
	IplImage	*image1 = cvCloneImage(frame0),
		*image2 = cvCloneImage(frame1);

	int board_w = 7; // Board width in squares
	int board_h = 7; // Board height 

	CvSize board_sz = cvSize(board_w, board_h);

	CvPoint2D32f *corners1 = new CvPoint2D32f[board_w * board_h];
	CvPoint2D32f *corners2 = new CvPoint2D32f[board_w * board_h];

	int corner_count = board_w * board_h;
	int found1 = cvFindChessboardCorners(image1, board_sz, corners1, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	int found2 = cvFindChessboardCorners(image2, board_sz, corners2, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
	bool findChessFLAG = false;

	if (found1 && found2){
		int *vals1 = minCoordinats(corners1, board_w, board_h);
		int *vals2 = minCoordinats(corners2, board_w, board_h);

		shiftImage(vals1[1] - vals2[1], 1, 3);										//����� ����������� �� min(y)
		shiftImage(vals1[3] - vals2[3], 5, 7);										//����� ����������� �� max(y)
		//shiftImage((vals1[2] - vals1[0]) - (vals2[2] - vals2[0]), 2, 6);			//����� ����������� x
		
		shiftImage(vals2[0] - vals1[0], 0, 4);										//����� ����������� �� min(x)
		shiftImage(vals1[2] - vals2[2], 2, 6);										//����� ����������� �� max(x)

		cvDrawChessboardCorners(image1, board_sz, corners1, corner_count, found1);	//������������ ���� ������� ����� ������ �� ������
		cvDrawChessboardCorners(image1, board_sz, corners2, corner_count, found2);	//������������ ���� ������� ����� ������ �� ������
		findChessFLAG = true;


		//(Mat)image1 = ((Mat)image1 + (Mat)image2) / 2;
	}
	cvShowImage("Calibration cameras direction", image1);


	return findChessFLAG;
}

void StereoPairCalibration::calibration(int CAPTURE_0, int CAPTURE_1){

	VideoCapture cap0(CAPTURE_0), cap1(CAPTURE_1);
	Mat frame[2];

	while (true){
		cap0 >> frame[0];
		cap1 >> frame[1];
		Common::rotateImage(frame[1], 180);

		correctivePerspective(&(IplImage)frame[0]);

		findChess(&(IplImage)frame[0], &(IplImage)frame[1], "frame0");		//����� ������� ��� �������� ���������� �����

		char c = cvWaitKey(33);
		if (c == 27){
			cout << "[INF] Calibration stereo pair of force interrupted!" << endl;
			break;
		}

		if (controlCount > 5){
			cout << "[INF] Stereo pair successfully calibrated" << endl;
			cvDestroyAllWindows();
			return;
		}
	}
}