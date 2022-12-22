//#include <lime/LimeSuite.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <stdio.h>

#include "../LMS8FE.h"
#include "../LMS8001/lime/LMS8API.h"

int openPort(char* portName);
void closePort(int fd);

//Device structure, should be initialize to NULL
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
	if (argc != 2)
	{
		printf("Error: Wrong number of parameters\n");
		printf("Usage: limelms8fe_USB_test <Limelms8fe COM port>\n");
		printf("Example: limelms8fe_USB_test COM3\n");
		exit(1);
	}
	//Open port
    lms8fe = LMS8FE_Open(argv[1], nullptr);

	if (lms8fe == nullptr) {
		std::cout << "Error: failed to open device" << std::endl;
		return -1;
	}
	else {
		std::cout << "Port opened" << std::endl;
	}

	unsigned char cinfo[4];
    int result;

//	LIMElms8fe_GetInfo(fd, cinfo);
	LMS8FE_GetInfo(lms8fe, cinfo);

	printf("Firmware version: %d\n", (int)cinfo[2]);
	printf("Hardware version: %#x\n", (int)cinfo[3]);

    LMS8FE_Reset(lms8fe);

    printf("Enable LMS8001...\n");
    LMS8FE_LMS8_Enable(lms8fe, 1);

    if (LMS8FE_LMS8_Open(lms8fe, &lms8_device))
        error();

    printf("Selecting channel 1\n");
    result = LMS8FE_Select_Channel(lms8fe, LMS8FE_CH1);

    const lms_dev_info_t* info = LMS8_GetDeviceInfo(lms8_device);
    printf("Firmware: %s\n", info->firmwareVersion);
    printf("Harware: %s\n", info->hardwareVersion);
    printf("Protocol: %s\n", info->protocolVersion);

    //Test SPI
    uint16_t address = 0x0004;
    uint16_t value;
    if(LMS8_ReadLMSReg(lms8_device, 0x0004, &value) != LMS_SUCCESS)
        error();
    printf("Address: 0x%04x; Value: 0x%02x\n", address, value);
    value = 0x01;
    if(LMS8_WriteLMSReg(lms8_device, 0x0004, value) != LMS_SUCCESS)
        error();
    LMS8_ReadLMSReg(lms8_device, 0x0004, &value);
    printf("Address: 0x%04x; Value: 0x%02x\n", address, value);
    value = 0x00;
    LMS8_WriteLMSReg(lms8_device, 0x0004, value);
    LMS8_ReadLMSReg(lms8_device, 0x0004, &value);
    printf("Address: 0x%04x; Value: 0x%02x\n", address, value);

    printf("Loading state...\n");
    result = LMS8_LoadConfig(lms8_device, "c:\\Data\\Temp\\LMS8001_state.ini");
    if (result != LMS_SUCCESS)
        printf("Error loading state.\n");
    printf("Finished\n");

    printf("Resetting...\n");
    result = LMS8_Reset(lms8_device);
    if (result != LMS_SUCCESS)
        printf("Error resetting.\n");
    printf("Finished\n");

    int isLocked;

    isLocked = LMS8_Get_SPI_Reg_bits(lms8_device, PLL_LOCK);
    printf("Is Locked: %d\n", isLocked);

    printf("Tuning PLL...\n");

    result = LMS8_PLL_Tune(lms8_device, 4.5);
    if (result != LMS_SUCCESS)
        printf("Error locking. Result: %d\n", result);

    isLocked = LMS8_Get_SPI_Reg_bits(lms8_device, PLL_LOCK);
    printf("Is Locked: %d\n", isLocked);

    printf("Resetting...\n");
    result = LMS8_Reset(lms8_device);
    if (result != LMS_SUCCESS)
        printf("Error resetting.\n");
    printf("Finished\n");

    isLocked = LMS8_Get_SPI_Reg_bits(lms8_device, PLL_LOCK);
    printf("Is Locked: %d\n", isLocked);

    printf("Smart Tuning PLL...\n");

    result = LMS8_PLL_Smart_Tune(lms8_device, 5.5);
    if (result != LMS_SUCCESS)
        printf("Error locking. Result: %d\n", result);

    isLocked = LMS8_Get_SPI_Reg_bits(lms8_device, PLL_LOCK);
    printf("Is Locked: %d\n", isLocked);

    printf("Selecting channel 2\n");
    result = LMS8FE_Select_Channel(lms8fe, LMS8FE_CH2);

    printf("Resetting...\n");
    result = LMS8_Reset(lms8_device);
    if (result != LMS_SUCCESS)
        printf("Error resetting.\n");
    printf("Finished\n");

    isLocked = LMS8_Get_SPI_Reg_bits(lms8_device, PLL_LOCK);
    printf("Is Locked: %d\n", isLocked);

    printf("Smart Tuning PLL...\n");

    result = LMS8_PLL_Smart_Tune(lms8_device, 3.5);
    if (result != LMS_SUCCESS)
        printf("Error locking. Result: %d\n", result);

    isLocked = LMS8_Get_SPI_Reg_bits(lms8_device, PLL_LOCK);
    printf("Is Locked: %d\n", isLocked);

//    printf("Disabling LMS8001...\n");
//    LMS8FE_LMS8_Enable(lms8fe, 0);

    //Close device
    LMS8_Close(lms8_device);

	//Close port
	LMS8FE_Close(lms8fe);

	return 0;
}
