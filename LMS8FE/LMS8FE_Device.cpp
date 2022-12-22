/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   LMS8FE_Device.cpp
 * Author: ignas
 *
 * Created on September 4, 2019, 1:18 PM
 */

#include "LMS8FE_Device.h"
#include "LMS8FE_constants.h"
// #include "lms7_device.h"

LMS8FE_Device::LMS8FE_Device(lms_device_t *dev, LMS8FE_COM com) : sdrDevice(dev),
																  com(com)
{
	//	Cmd_GetConfig(dev, com, &boardState);
}

LMS8FE_Device::~LMS8FE_Device()
{
}