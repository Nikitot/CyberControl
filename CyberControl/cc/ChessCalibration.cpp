
#include "stdafx.h"
#include "Common.h"
#include "ChessCalibration.h"

//������ 11-1.
//������ ������ � ������ ��������� �����,
//������ � ������������������ ������������ ���������� �����
//� ���������� ������

// calib.cpp 
//������������� ������� ������: 
// calib board_w board_h number_of_views 
// 
// ������� �p� - ���������/���������� �����, ESC - �����
//


int board_w;
int board_h;
int numCapture;

void ChessCalibration::calibration(IplImage *mapx, IplImage *mapy, int c, int n_boards, int board_dt, bool rotate){
	//common
	numCapture = c;
	board_w = 23;
	board_h = 15;

	int board_n = board_w * board_h;
	CvSize board_sz = cvSize(board_w, board_h);
	CvCapture* capture = cvCreateCameraCapture(c);

	assert(capture);

	cvNamedWindow("Calibration");
	//�������� ��������� ������
	CvMat* image_points = cvCreateMat(n_boards*board_n, 2, CV_32FC1);
	CvMat* object_points = cvCreateMat(n_boards*board_n, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(n_boards, 1, CV_32SC1);
	CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(5, 1, CV_32FC1);

	CvPoint2D32f* corners = new CvPoint2D32f[board_n];
	int corner_count;
	int successes = 0;
	int step, frame = 0;
	IplImage *image = cvQueryFrame(capture);
	IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);//subpixel

	// ������ � ����� ����������� � ������ ���� �� �� ������� n_boards
	// ������ �������� (��� ���� �� ����� ���� �������)
	while (successes < n_boards) {
		//���������� board_dt �������, ��������������� �������������, ��������� �����
		if (frame++ % board_dt == 0) {
			//������� ���� ��������� �����:
			int found = cvFindChessboardCorners(
				image, board_sz, corners, &corner_count,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
				);

			//��������� ������������� �������� �� ���� �����
			cvCvtColor(image, gray_image, CV_BGR2GRAY);
			cvFindCornerSubPix(gray_image, corners, corner_count,
				cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			//�������� ���
			cvDrawChessboardCorners(image, board_sz, corners,
				corner_count, found);
			cvShowImage("Calibration", image);

			// ���� �� �������� ������� �����, ������� �� � ����� ������
			if (corner_count == board_n) {
				step = successes*board_n;
				for (int i = step, j = 0; j < board_n; ++i, ++j) {
					CV_MAT_ELEM(*image_points, float, i, 0) = (float)corners[j].x;  //float
					CV_MAT_ELEM(*image_points, float, i, 1) = (float)corners[j].y;
					CV_MAT_ELEM(*object_points, float, i, 0) = (float)(j / board_w);
					CV_MAT_ELEM(*object_points, float, i, 1) = (float)(j%board_w);
					CV_MAT_ELEM(*object_points, float, i, 2) = (float)0.0f;
				}
				CV_MAT_ELEM(*point_counts, int, successes, 0) = board_n;
				successes++;
			}
		} //end skip board_dt between chessboard capture

		//Handle pause/unpause and ESC
		int c = cvWaitKey(15);
		if (c == 'p'){
			c = 0;
			while (c != 'p' && c != 27){
				cout << "	* The construction of undistortion maps paused..." << endl;
				c = cvWaitKey(250);
				cout << "	* The construction of undistortion maps resumed..." << endl;
			}
		}
		if (c == 27){
			cout << "Warning: The construction of undistortion maps force interrupted!" << endl;
			return;
		}

		image = cvQueryFrame(capture); //�������� ��������� �����������
	} //����� ������������������ ������ WHILE.

	//�������� ������� � ������ ���������� ��������� ����, ������� ���� �������
	CvMat* object_points2 = cvCreateMat(successes*board_n, 3, CV_32FC1);
	CvMat* image_points2 = cvCreateMat(successes*board_n, 2, CV_32FC1);
	CvMat* point_counts2 = cvCreateMat(successes, 1, CV_32SC1);
	//���������� ����� � ������� ����������� �������
	//���� �� ������ ��� �������� � ��������� ���� ������.
	//�� ����� ������ ����� ��������:
	//image_points->rows = object_points->rows = \
				   //successes*board_n; point_counts->rows = successes;
	//
	for (int i = 0; i < successes*board_n; ++i) {
		CV_MAT_ELEM(*image_points2, float, i, 0) =
			CV_MAT_ELEM(*image_points, float, i, 0);
		CV_MAT_ELEM(*image_points2, float, i, 1) =
			CV_MAT_ELEM(*image_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 0) =
			CV_MAT_ELEM(*object_points, float, i, 0);
		CV_MAT_ELEM(*object_points2, float, i, 1) =
			CV_MAT_ELEM(*object_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 2) =
			CV_MAT_ELEM(*object_points, float, i, 2);
	}
	for (int i = 0; i < successes; ++i){ //����� ��� �� �� �����
		CV_MAT_ELEM(*point_counts2, int, i, 0) =
			CV_MAT_ELEM(*point_counts, int, i, 0);
	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	// ��� ���� ����� �� ����� ��� ���� ��������� �����, ������� ��� �����.
	// �������������� ������� ���������� ���������� ���, ��� ��� ���������
	// ���������� ����� ����� ����������� 1.0
	//
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1.0f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1.0f;

	//��������� ������!
	cvCalibrateCamera2(
		object_points2, image_points2,
		point_counts2, cvGetSize(image),
		intrinsic_matrix, distortion_coeffs,
		NULL, NULL, 0 //CV_CALIB_FIX_ASPECT_RATIO
		);

	// ��������� ���������� ��������� � ���������
	cvSave("Intrinsics.xml", intrinsic_matrix);
	cvSave("Distortion.xml", distortion_coeffs);

	// ������ �������� ���� ������ ����� � ����������:
	CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
	CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");

	// ������ ����� �����������, ������� �� ����� ������������
	// ��� ���� ����������� ������.
	//

	cvInitUndistortMap(
		intrinsic,
		distortion,
		mapx,
		mapy
		);
	cvDestroyAllWindows();
	cout << "The construction of undistortion maps succsessfuli complited." << endl;
	return;
}