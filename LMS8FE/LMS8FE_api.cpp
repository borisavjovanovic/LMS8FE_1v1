#include "LMS8FE_constants.h"
// #include "lms7_device.h"
#include "LMS8FE_Device.h"
#include <cstdint>
// milans 220420
// #include "Logger.h"
#include "./LMS8001/API/lms8_device.h"
#include "./LMS8001/lms8001/lmsComms.h"
#include <iostream>

#define SERIAL_BAUDRATE 9600
// #define SERIAL_BAUDRATE 115200

/****************************************************************************
 *
 *   LimeLMS8FE API Functions
 *
 *****************************************************************************/
extern "C" API_EXPORT lms8fe_dev_t *CALL_CONV LMS8FE_Open(const char *serialport, lms_device_t *dev)
{
    if (dev == nullptr && serialport == nullptr)
        return nullptr;

    int result;

    LMS8FE_COM com;
    // for unix
    com.fd = -1;
    com.hComm = -1; // B.J.
#ifndef __unix__
    com.hComm = 0;
#endif
    if (serialport != nullptr)
    {
        result = Lms8fe_serialport_init(serialport, SERIAL_BAUDRATE, &com);
        if (result == -1)
            return nullptr;
        cout << "com.fd =" << com.fd << std::endl; // B.J.  
        cout << "com.hComm ="  <<(int)com.hComm << std::endl; //B.J.    
        // milans 220421
        //		result = Cmd_Hello(com);
        //         if (result == LMS8FE_ERROR_COMM) {
        //             return nullptr;
        //         }
    }
    else {
        cout << "com.fd =" << com.fd << std::endl; // B.J.  
        cout << "com.hComm ="  <<(int)com.hComm << std::endl; //B.J. 
        //Lms8fe_Cmd_Hello(dev, com); 
    }
    return new LMS8FE_Device(dev, com);
}

extern "C" API_EXPORT void CALL_CONV LMS8FE_Close(lms8fe_dev_t *lms8fe)
{
    if (!lms8fe)
        return;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    if ((dev->com).fd >= 0)
        Lms8fe_serialport_close(dev->com);
    delete dev;
}

extern "C" API_EXPORT int CALL_CONV LMS8FE_GetInfo(lms8fe_dev_t *lms8fe, unsigned char *cinfo)
{
    Lms8fe_boardInfo info;
    int result = 0;
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    result = Lms8fe_Cmd_GetInfo(dev->sdrDevice, dev->com, &info);
    cinfo[0] = info.fw_ver;
    cinfo[1] = info.hw_ver;
    cinfo[2] = info.status1;
    cinfo[3] = info.status2;

    return result;
}

extern "C" API_EXPORT int LMS8FE_LoadConfig(lms8fe_dev_t *lms8fe, const char *filename)
{

    int result = 0;
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    result = Lms8fe_Cmd_LoadConfig(dev->sdrDevice, dev->com, filename);

    //        if (result == 0)
    //            dev->UpdateState();

    return result;
}

extern "C" API_EXPORT int LMS8FE_Reset(lms8fe_dev_t *lms8fe)
{
    int result = 0;

    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    result = Lms8fe_Cmd_Reset(dev->sdrDevice, dev->com);

    //        if (result == 0)
    //            dev->UpdateState();

    return result;
}

//milans 221128
extern "C" API_EXPORT int LMS8FE_LMS8_Enable(lms8fe_dev_t * lms8fe, int value)
{
    int result = 0;

    if (!lms8fe)
        return -1;
    auto* dev = static_cast<LMS8FE_Device*>(lms8fe);

    result = Lms8fe_Cmd_lms8_Enable(dev->sdrDevice, dev->com, value);

    //        if (result == 0)
    //            dev->UpdateState();

    return result;
}

/*
extern "C" API_EXPORT int LMS8FE_Configure(lms8fe_dev_t *lms8fe, char channelIDRX, char channelIDTX, char portRX, char portTX, char mode, char notch, char attenuation, char enableSWR, char sourceSWR)
{
    lms8fe_boardState state = {channelIDRX, channelIDTX, portRX, portTX, mode, notch, attenuation, enableSWR, sourceSWR};
    return LMS8FE_ConfigureState(lms8fe, state);
}

extern "C" API_EXPORT int LMS8FE_ConfigureState(lms8fe_dev_t *lms8fe, lms8fe_boardState state)
{
    int result = 0;
            if (!lms8fe)
                return -1;
            auto* dev = static_cast<LMS8FE_Device*>(lms8fe);

    //        dev->AutoFreq(state);

        result = Cmd_Configure(dev->sdrDevice, dev->com, state.channelIDRX, state.channelIDTX, state.selPortRX, state.selPortTX, state.mode, state.notchOnOff, state.attValue, state.enableSWR, state.sourceSWR);

            if (result == 0)
    //            dev->UpdateState(state);
    return result;
}
*/
// milans 221201
// This function may be useful... Implement it!
// Maybe there should be LMS8FE_GetState, and LMS8FE_SetState...
extern "C" API_EXPORT int LMS8FE_SetState(lms8fe_dev_t * lms8fe, lms8fe_boardState state)
{
    int result = 0;

    if (!lms8fe)
        return -1;
    auto* dev = static_cast<LMS8FE_Device*>(lms8fe);

    result = Lms8fe_Cmd_Configure(dev->sdrDevice, dev->com, state);

    return result;
}

extern "C" API_EXPORT int LMS8FE_GetState(lms8fe_dev_t * lms8fe, lms8fe_boardState* state)
{
    int result = 0;

    if (!lms8fe)
        return -1;
    auto* dev = static_cast<LMS8FE_Device*>(lms8fe);

    result = Lms8fe_Cmd_GetConfig(dev->sdrDevice, dev->com, state);

    return result;
}

/*
extern "C" API_EXPORT int LMS8FE_GetState(lms8fe_dev_t *lms8fe, lms8fe_boardState *state)
{
    int result = 0;

    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    result = Lms8fe_Cmd_GetConfig(dev->sdrDevice, dev->com, state);

    return result;
}
*/
/*
extern "C" API_EXPORT int LMS8FE_Mode(lms8fe_dev_t *lms8fe, int mode)
{
    int result = 0;

    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    result = Lms8fe_Cmd_Mode(dev->sdrDevice, dev->com, mode);

    //        if (result == 0)
    //            dev->UpdateState(mode);
    return result;
}

extern "C" API_EXPORT int LMS8FE_ReadADC(lms8fe_dev_t *lms8fe, int adcID, int *value)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_ReadADC(dev->sdrDevice, dev->com, adcID, value);
}

extern "C" API_EXPORT int LMS8FE_ConfGPIO(lms8fe_dev_t *lms8fe, int gpioNum, int direction)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_ConfGPIO(dev->sdrDevice, dev->com, gpioNum, direction);
}

extern "C" API_EXPORT int LMS8FE_SetGPIO(lms8fe_dev_t *lms8fe, int gpioNum, int val)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_SetGPIO(dev->sdrDevice, dev->com, gpioNum, val);
}

extern "C" API_EXPORT int LMS8FE_GetGPIO(lms8fe_dev_t *lms8fe, int gpioNum, int *val)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_GetGPIO(dev->sdrDevice, dev->com, gpioNum, val);
}

API_EXPORT int CALL_CONV LMS8FE_AssignSDRChannels(lms8fe_dev_t *lms8fe, int rx, int tx)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    //    dev->SetChannels(rx, tx);
    return 0;
}

extern "C" API_EXPORT int LMS8FE_Fan(lms8fe_dev_t *lms8fe, int enable)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_Fan(dev->sdrDevice, dev->com, enable);
}

extern "C" API_EXPORT int LMS8FE_Diode(lms8fe_dev_t *lms8fe, int state)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_Diode(dev->sdrDevice, dev->com, state);
}

extern "C" API_EXPORT int LMS8FE_DiodeSPI(lms8fe_dev_t *lms8fe, int state)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_DiodeSPI(dev->sdrDevice, dev->com, state);
}
*/
extern "C" API_EXPORT int LMS8FE_SC1905_SPI_Message_Memory(lms8fe_dev_t *lms8fe, uint16_t address, uint8_t *val, bool isRead, int bytesNo, bool isEEPROM)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_SC1905_SPI_Message_Memory(dev->sdrDevice, dev->com, address, val, isRead, bytesNo, isEEPROM);
}

extern "C" API_EXPORT int LMS8FE_SC1905_SPI_Special_Command(lms8fe_dev_t *lms8fe, int command)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_SC1905_SPI_Special_Command(dev->sdrDevice, dev->com, command);
}

extern "C" API_EXPORT int LMS8FE_SC1905_Reset(lms8fe_dev_t *lms8fe)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_SC1905_Reset(dev->sdrDevice, dev->com);
}

extern "C" API_EXPORT int LMS8FE_SC1905_Apply_Frequency(lms8fe_dev_t *lms8fe, int freqRange, int minFreq, int maxFreq)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_SC1905_Apply_Frequency(dev->sdrDevice, dev->com, freqRange, minFreq, maxFreq);
}

extern "C" API_EXPORT int LMS8FE_Set_Config_Full(lms8fe_dev_t *lms8fe, uint8_t *state, int size)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_Set_Config_Full(dev->sdrDevice, dev->com, state, size);
}

extern "C" API_EXPORT int LMS8FE_Get_Config_Full(lms8fe_dev_t *lms8fe, uint8_t *state, int size)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);

    return Lms8fe_Cmd_Get_Config_Full(dev->sdrDevice, dev->com, state, size);
}

extern "C" API_EXPORT int LMS8FE_LMS8_Open(lms8fe_dev_t * lms8fe, lms_device_t * *device)
{
    if (device == nullptr)
    {
//        lime::error("Device pointer cannot be NULL");
        return -1;
    }

    auto dev = LMS8_Device::CreateDevice();
    if (dev == nullptr)
    {
//        lime::error("Unable to open device");
        return -1;
    }
    *device = dev;

    LMScomms* lms8controlPort = dev->GetConnection();

    LMS8FE_COM com = ((LMS8FE_Device*)lms8fe)->com;

    lms8controlPort->InheritCOM(com.hComm);
    lms8controlPort->lms8fe_do_mask = true;
    lms8controlPort->lms8fe_cmd_mask = 0x80;

    return LMS_SUCCESS;
}

//milans 221130
extern "C" API_EXPORT int LMS8FE_Select_Channel(lms8fe_dev_t * lms8fe, int channel)
{
    int result = 0;

    if (!lms8fe)
        return -1;
    auto* dev = static_cast<LMS8FE_Device*>(lms8fe);

    result = Lms8fe_Cmd_Select_Channel(dev->sdrDevice, dev->com, channel);

    //        if (result == 0)
    //            dev->UpdateState();

    return result;
}
// B.J.
extern "C" API_EXPORT int LMS8FE_SPI_write(lms8fe_dev_t *lms8fe, uint16_t maddress, uint16_t address, uint16_t data)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    return Lms8fe_SPI_write(dev->sdrDevice, maddress, address, data);
}

extern "C" API_EXPORT int LMS8FE_SPI_read(lms8fe_dev_t *lms8fe, uint16_t maddress, uint16_t address, uint16_t * pData)
{
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    return Lms8fe_SPI_read(dev->sdrDevice, maddress, address, pData);
}


extern "C" API_EXPORT int LMS8FE_SPI_write_buffer(lms_device_t *lms8fe, unsigned char *c, int size) {
    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    return Lms8fe_spi_write_buffer(dev->sdrDevice, c, size);
}

/*
extern "C" API_EXPORT int LMS8FE_SPI_read_buffer2(lms_device_t *lms8fe, unsigned char *c, int size) {

    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    return Lms8fe_spi_read_buffer2(dev->sdrDevice, c, size);
}
*/

extern "C" API_EXPORT int LMS8FE_SPI_read_buffer(lms_device_t *lms8fe, unsigned char *c, int size) {

    if (!lms8fe)
        return -1;
    auto *dev = static_cast<LMS8FE_Device *>(lms8fe);
    return Lms8fe_spi_read_buffer(dev->sdrDevice, c, size);
}

