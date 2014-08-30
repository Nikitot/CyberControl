#include "stdafx.h"
#include "MainActivityProcess.h"


int main(int argc, char* argv[]){
	MainActivityProcess *mainActivityProcess = new MainActivityProcess();
	int exit_code = mainActivityProcess->mainActivity(argc, argv);
	delete mainActivityProcess;
	return exit_code;
}