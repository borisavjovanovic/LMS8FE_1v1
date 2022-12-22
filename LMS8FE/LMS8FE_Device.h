/*
 * File:   LMS8FE_Device.h
 * Author: ignas
 *
 * Created on September 4, 2019, 1:18 PM
 */

#ifndef LMS8FE_DEVICE_H
#define LMS8FE_DEVICE_H

#include "LMS8FE_constants.h"

class LMS8FE_Device
{
public:
    LMS8FE_Device(lms_device_t *d, LMS8FE_COM com);
    LMS8FE_Device(const LMS8FE_Device &) = delete;
    LMS8FE_Device &operator=(const LMS8FE_Device &) = delete;
    ~LMS8FE_Device();
    lms_device_t *sdrDevice;
    struct LMS8FE_COM com;

private:
    lms8fe_boardState boardState;
};

#endif /* LMS8FE_DEVICE_H */
