//#include <lime/LimeSuite.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdio.h>

#include "../LMS8FE.h"

int openPort(char* portName);
void closePort(int fd);

int main(int argc, char** argv)
{	
/*	
	if (argc != 2)
	{
		printf("Error: Wrong number of parameters\n");
		printf("Usage: limeRFE_USB_test <LimeRFE COM port>\n");
		printf("Example: limeRFE_USB_test COM3\n");
		exit(1);
	}

	//Open port
        lms8fe_dev_t* lms8fe = LMS8FE_Open(argv[1], nullptr);
*/
	//milans 221201 TMP TMP TMP TMP TMP
	lms8fe_dev_t* lms8fe = LMS8FE_Open("COM4", nullptr);

	if (lms8fe == nullptr) {
		std::cout << "Error: failed to open device" << std::endl;
		return -1;
	}
	else {
		std::cout << "Port opened" << std::endl;
	}
/*
	//Configure LimeRFE to use channel HAM 2m channel in receive mode.
	//Transmit output is routed to TX/RX output. Notch is off. Attenuation is 0.
	LMS8FE_Configure(rfe, LMS8FE_CID_HAM_0145, LMS8FE_CID_HAM_0145, LMS8FE_PORT_1, LMS8FE_PORT_1, LMS8FE_MODE_RX, 
	LMS8FE_NOTCH_OFF, 0, 0, 0);
	//or simpler (by using default arguments)
	//RFE_Configure(NULL, fd, RFE_CID_HAM_0145);

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	//Change mode to transmit
	LMS8FE_Mode(rfe, LMS8FE_MODE_TX);

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
*/	

	//Reset LimeRFE
	LMS8FE_Reset(lms8fe);

	lms8fe_boardState state;
	int result;	
	result = LMS8FE_GetState(lms8fe, &state);
	printf("state.DA1_EN = %d\n", state.DA1_EN);

	state.DA1_EN = 1;

	result = LMS8FE_SetState(lms8fe, state);

	result = LMS8FE_GetState(lms8fe, &state);
	printf("state.DA1_EN = %d\n", state.DA1_EN);

	//Close port
	LMS8FE_Close(lms8fe);

	return 0;
}
