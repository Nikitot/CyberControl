#pragma once
class StereoPairCalibration
{

public:

	Common *common;
	StereoPairCalibration();
	~StereoPairCalibration();
	/*
		����������
		*/
	void calibration(Captures *captures);
private:
	/*
		������� ���������� ������� � ���������� �������� ��������� �����
		*/
	int *minCoordinats(CvPoint2D32f *corners, int board_w, int board_h);

	/*
		������������ ����������� ������� ����������� � ������������ �� ������
		*/
	void correctivePerspective(IplImage *frame0);

	/*
		������ �����������, ���� dif != 0,
		dif - �������� �������� ����� ��������� ������ �� ���� ������������
		*/
	void shiftImage(int dif, int k1, int k2);

	/*
		����� ��������� �����
		*/
	bool findChess(IplImage *image1, IplImage *image2, char *name);



};

