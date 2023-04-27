//#include <lime/LimeSuite.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdio.h>
#include <unistd.h>

#include "../LMS8FE.h"
#include "../LMS8001/lime/LMS8API.h"

int openPort(char* portName);
void closePort(int fd);

lms8fe_dev_t* lms8fe = NULL;
lms_device_t* lms8_device = NULL;

int error()
{
    if (lms8_device != NULL)
        LMS8_Close(lms8_device);
    if (lms8fe != NULL)
        LMS8FE_Close(lms8fe);
    exit(-1);
}

int main(int argc, char** argv)
{	
	lms8fe_boardState state;
	int result = 0;	
    int isLocked = 0;

    // SC1905 variables
    int rfinAgc = 0;
    int rffbAgc = 0;
    float centerFreq = 0.0;
    float bandwidth = 0.0;
    char stringValue[100];


    lms8fe = LMS8FE_Open("/dev/ttyACM0", nullptr);

	if (lms8fe == nullptr) {
		std::cout << "Error: failed to open device" << std::endl;
		return -1;
	}
	else {
		std::cout << "Port opened" << std::endl;
	}

	printf("Resetting LMS8FE board ...\n");
    LMS8FE_Reset(lms8fe);
    
    printf("Loading LMS8FE board ini file ...\n");
    LMS8FE_LoadConfig(lms8fe, "Conf_LMS8FE1.ini");

    printf("Selecting channel 1\n");
    result = LMS8FE_Select_Channel(lms8fe, LMS8FE_CH1);

    printf("Enable LMS8001...\n");
    LMS8FE_LMS8_Enable(lms8fe, 1);

    if (LMS8FE_LMS8_Open(lms8fe, &lms8_device))
        error();

    printf("Resetting LMS8001...\n");
    result = LMS8_Reset(lms8_device);
    if (result != LMS_SUCCESS)
        printf("Error resetting.\n");
    printf("Finished\n");
    
    printf("Loading LMS8001 state...\n");
    result = LMS8_LoadConfig(lms8_device, "Conf_LMS8001.ini");
    if (result != LMS_SUCCESS)
        printf("Error loading state.\n");
    printf("Finished\n");

    printf("Smart Tuning PLL...\n");

    result = LMS8_PLL_Smart_Tune(lms8_device, 4.9);
    if (result != LMS_SUCCESS)
        printf("Error locking. Result: %d\n", result);

    isLocked = LMS8_Get_SPI_Reg_bits(lms8_device, PLL_LOCK);
    printf("Is Locked: %d\n", isLocked);
    
    printf("Enable SC1905...\n");
    LMS8FE_SC1905_Enable(lms8fe, LMS8FE_CH1, 1); // Enable=1; Disable=0
	
    sleep(2); // important delay
    printf("Finished\n");

    //result = LMS8FE_SC1905_Apply_Frequency(lms8fe, 7, 1950, 2050);
    result = LMS8FE_SC1905_Set_Duty_Cycle_Feedback(lms8fe, 1);  // On=1/Off=0
    result = LMS8FE_SC1905_Set_Adaptation_State(lms8fe, 1); //Running=1/Frozen=0
    result = LMS8FE_SC1905_Set_Correction_Enable(lms8fe, 1); // FW Control=1/Disabled=0
   
    result = LMS8FE_SC1905_Read_RFIN_AGC(lms8fe, &rfinAgc);
    printf("SC1905 rfinAgc = %d\n", rfinAgc);
   
    result = LMS8FE_SC1905_Read_RFFB_AGC(lms8fe, &rffbAgc);
    printf("SC1905 rffbAgc = %d\n", rffbAgc);

    result = LMS8FE_SC1905_Read_Center_Frequency(lms8fe, &centerFreq);
    printf("SC1905 centerFreq = %f\n", centerFreq);

    result = LMS8FE_SC1905_Read_Signal_Bandwidth(lms8fe, &bandwidth);
    printf("SC1905 bandwidth = %f\n", bandwidth);
   
    result = LMS8FE_SC1905_Read_Error_Code(lms8fe, stringValue);
    printf("SC1905 Error code = %s\n", stringValue);

    result = LMS8FE_SC1905_Read_Warning_Code(lms8fe, stringValue);
    printf("SC1905 Warning code = %s\n", stringValue);

    float attenuation = 0.0;
    result = LMS8FE_Get_TX_Att(lms8fe, LMS8FE_CH1, &attenuation);
    printf("TX1 att. = %8.3f\n", attenuation);

    attenuation += 0.25;
    result = LMS8FE_Set_TX_Att(lms8fe, LMS8FE_CH1, attenuation);

    result = LMS8FE_Get_TX_Att(lms8fe, LMS8FE_CH1, &attenuation);
    printf("TX1 att. = %8.3f\n", attenuation);

    attenuation = 0.0;
    result = LMS8FE_Get_ORX_Att(lms8fe, LMS8FE_CH1, &attenuation);
    printf("ORX1 att. = %8.3f\n", attenuation);

    attenuation += 0.25;
    result = LMS8FE_Set_ORX_Att(lms8fe, LMS8FE_CH1, attenuation);
    
    result = LMS8FE_Get_ORX_Att(lms8fe, LMS8FE_CH1, &attenuation);
    printf("ORX1 att. = %8.3f\n", attenuation);
	


   //Close device
    LMS8_Close(lms8_device);

    //Close port
	LMS8FE_Close(lms8fe);
	return 0;
}
