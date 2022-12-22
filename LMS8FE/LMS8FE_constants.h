#ifndef __limeLMS8FE_constants__
#define __limeLMS8FE_constants__

#include "LMS8FE.h"

#include <fcntl.h> // File control definitions
#include "LimeSuite.h"
using namespace std;
#include <string.h>
#include <math.h>

#ifdef _MSC_VER
#include <tchar.h>

#define O_NOCTTY 0
#define IXANY 0x00000800 /* any char will restart after stop */

#include <winsock2.h>
#endif // WIN

#ifdef __unix__
#include <unistd.h>
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <getopt.h>

// tchar.h
typedef char TCHAR;

#endif // LINUX

typedef struct LMS8FE_COM
{
#ifndef __unix__
	HANDLE hComm;
#else
	int hComm; // B.J. add com port file descriptor for unix
#endif
	int fd;
} LMS8FE_COM;

#define LMS8FE_I2C 0
#define LMS8FE_USB 1

#define LMS8FE_BUFFER_SIZE 16
#define LMS8FE_BUFFER_SIZE_MODE 2

// test
#define LMS8FE_CMD_LED_ONOFF 0xFF

#define GPIO_SCL 6
#define GPIO_SDA 7

#define LMS8FE_I2C_FSCL 100E3 // Approx. SCL frequency - ???

#define LMS8FE_CMD_HELLO 0x00

// CTRL
#define LMS8FE_CMD_MODE 0xd1
#define LMS8FE_CMD_CONFIG 0xd2
#define LMS8FE_CMD_MODE_FULL 0xd3
// #define LMS8FE_CMD_CONFIG_FULL             0xd4

#define LMS8FE_CMD_READ_ADC1 0xa1
#define LMS8FE_CMD_READ_ADC2 0xa2
#define LMS8FE_CMD_READ_TEMP 0xa3

#define LMS8FE_CMD_CONFGPIO45 0xb1
#define LMS8FE_CMD_SETGPIO45 0xb2
#define LMS8FE_CMD_GETGPIO45 0xb3

#define LMS8FE_CMD_FAN 0xc1

// General CTRL
// milans 220614
// #define LMS8FE_CMD_GET_INFO                0xe1
#define LMS8FE_CMD_GET_INFO 0x11
// milans 220624
#define LMS8FE_CMD_SET_CONFIG_FULL 0x12
#define LMS8FE_CMD_GET_CONFIG_FULL 0x13
// milans 220714
#define LMS8FE_CMD_RESET           0x14
//milans 221128
#define LMS8FE_CMD_LMS8_ENABLE     0x15
#define LMS8FE_CMD_SELECT_CHANNEL  0x16

// #define LMS8FE_CMD_RESET                   0xe2
//#define LMS8FE_CMD_GET_CONFIG 0xe3
// #define LMS8FE_CMD_GET_CONFIG_FULL         0xe4
//#define LMS8FE_CMD_I2C_MASTER 0xe5

// milans 220429
// #define LMS8FE_CMD_DIODE                   0xf1
//#define LMS8FE_CMD_DIODE 0x71
// milans 220506
// #define LMS8FE_CMD_DIODESPI                0xf2
//#define LMS8FE_CMD_DIODESPI 0x72

// SC1905 Control
// #define LMS8FE_CMD_SC1905_SPI_MESSAGE_MEMORY  0x91
// #define LMS8FE_CMD_SC1905_RESET               0x92
// #define LMS8FE_CMD_SC1905_SPI_SPECIAL_COMMAND 0x93
// #define LMS8FE_CMD_SC1905_SPI_EEPROM          0x94

#define LMS8FE_CMD_SC1905_SPI_MESSAGE_MEMORY 0x61
#define LMS8FE_CMD_SC1905_RESET 0x62
#define LMS8FE_CMD_SC1905_SPI_SPECIAL_COMMAND 0x63
#define LMS8FE_CMD_SC1905_SPI_EEPROM 0x64

#define LMS8FE_DISABLE 0
#define LMS8FE_ENABLE 1

#define LMS8FE_OFF 0
#define LMS8FE_ON 1

#define LMS8FE_MAX_HELLO_ATTEMPTS 10

#define LMS8FE_TIME_BETWEEN_HELLO_MS 200

#define LMS8FE_TYPE_INDEX_WB 0
#define LMS8FE_TYPE_INDEX_HAM 1
#define LMS8FE_TYPE_INDEX_CELL 2
#define LMS8FE_TYPE_INDEX_COUNT 3

#define LMS8FE_CHANNEL_INDEX_WB_1000 0
#define LMS8FE_CHANNEL_INDEX_WB_4000 1
#define LMS8FE_CHANNEL_INDEX_WB_COUNT 2

#define LMS8FE_CHANNEL_INDEX_HAM_0030 0
#define LMS8FE_CHANNEL_INDEX_HAM_0070 1
#define LMS8FE_CHANNEL_INDEX_HAM_0145 2
#define LMS8FE_CHANNEL_INDEX_HAM_0220 3
#define LMS8FE_CHANNEL_INDEX_HAM_0435 4
#define LMS8FE_CHANNEL_INDEX_HAM_0920 5
#define LMS8FE_CHANNEL_INDEX_HAM_1280 6
#define LMS8FE_CHANNEL_INDEX_HAM_2400 7
#define LMS8FE_CHANNEL_INDEX_HAM_3500 8
#define LMS8FE_CHANNEL_INDEX_HAM_COUNT 9

#define LMS8FE_CHANNEL_INDEX_CELL_BAND01 0
#define LMS8FE_CHANNEL_INDEX_CELL_BAND02 1
#define LMS8FE_CHANNEL_INDEX_CELL_BAND03 2
#define LMS8FE_CHANNEL_INDEX_CELL_BAND07 3
#define LMS8FE_CHANNEL_INDEX_CELL_BAND38 4
#define LMS8FE_CHANNEL_INDEX_CELL_COUNT 5

#define LMS8FE_PORT_1_NAME "TX/RX (J3)"		   // J3 - TX/RX
#define LMS8FE_PORT_2_NAME "TX (J4)"		   // J4 - TX
#define LMS8FE_PORT_3_NAME "30 MHz TX/RX (J5)" // J5 - 30 MHz TX/RX

#define LMS8FE_TXRX_VALUE_RX 0
#define LMS8FE_TXRX_VALUE_TX 1

#define LMS8FE_NOTCH_DEFAULT 0

#define LMS8FE_NOTCH_BIT_OFF 1
#define LMS8FE_NOTCH_BIT_ON 0

#define LMS8FE_NOTCH_BYTE 8
#define LMS8FE_NOTCH_BIT 0
#define LMS8FE_ATTEN_BYTE 12
#define LMS8FE_ATTEN_BIT 0 // LSB bit - Attenuation is 3-bit value
#define LMS8FE_PORTTX_BYTE 11
#define LMS8FE_PORTTX_BIT 5

#define LMS8FE_MODE_RX 0
#define LMS8FE_MODE_TX 1
#define LMS8FE_MODE_NONE 2
#define LMS8FE_MODE_TXRX 3

#define LMS8FE_MCU_BYTE_PA_EN_BIT 0
#define LMS8FE_MCU_BYTE_LNA_EN_BIT 1
#define LMS8FE_MCU_BYTE_TXRX0_BIT 2
#define LMS8FE_MCU_BYTE_TXRX1_BIT 3
#define LMS8FE_MCU_BYTE_RELAY_BIT 4

#define LMS8FE_CHANNEL_RX 0
#define LMS8FE_CHANNEL_TX 1

typedef struct
{
	unsigned char status1;
	unsigned char status2;
	unsigned char fw_ver;
	unsigned char hw_ver;
} Lms8fe_boardInfo;

struct Lms8fe_guiState
{
	double powerCellCorr;
	double powerCorr;
	double rlCorr;
};

#if __cplusplus
extern "C"
{
#endif

	int Lms8fe_write_buffer_fd(LMS8FE_COM com, unsigned char *c, int size);
	int Lms8fe_read_buffer_fd(LMS8FE_COM com, unsigned char *data, int size);
	int Lms8fe_write_buffer(lms_device_t *dev, LMS8FE_COM com, unsigned char *data, int size);
	int Lms8fe_read_buffer(lms_device_t *dev, LMS8FE_COM com, unsigned char *data, int size);

	//int Lms8fe_spi_read_buffer2(lms_device_t *lms, unsigned char *c, int size);
	int Lms8fe_spi_read_buffer(lms_device_t *lms, unsigned char *c, int size);  // B.J.
	int Lms8fe_spi_write_buffer(lms_device_t *lms, unsigned char *c, int size);
	


	int Lms8fe_my_read(LMS8FE_COM com, char *buffer, int count);
	int Lms8fe_my_write(LMS8FE_COM com, char *buffer, int count);
	int Lms8fe_serialport_write(LMS8FE_COM com, const char *str, int len);
	int Lms8fe_serialport_read(LMS8FE_COM com, char *buff, int len);
	int Lms8fe_serialport_init(const char *serialport, int baud, LMS8FE_COM *com);
	int Lms8fe_serialport_close(LMS8FE_COM com);
	int Lms8fe_Cmd_GetInfo(lms_device_t *dev, LMS8FE_COM com, Lms8fe_boardInfo *info);
	int Lms8fe_Cmd_GetConfig(lms_device_t *dev, LMS8FE_COM com, lms8fe_boardState *state);
	int Lms8fe_Cmd_Hello(lms_device_t *dev, LMS8FE_COM com);
	int Lms8fe_Cmd_LoadConfig(lms_device_t *dev, LMS8FE_COM com, const char *filename);
	int Lms8fe_Cmd_Reset(lms_device_t *dev, LMS8FE_COM com);
//milans 221128
	int Lms8fe_Cmd_lms8_Enable(lms_device_t* dev, LMS8FE_COM com, uint8_t value);
//milans 221130
	int Lms8fe_Cmd_Select_Channel(lms_device_t* dev, LMS8FE_COM com, uint8_t channel);
	//	int Lms8fe_Cmd_ConfigureState(lms_device_t* dev, LMS8FE_COM com, lms8fe_boardState state);
	//	int Lms8fe_Cmd_Configure(lms_device_t *dev, LMS8FE_COM com, int channelIDRX, int channelIDTX = -1, int selPortRX = 0, int selPortTX = 0, int mode = 0, int notch = 0, int attenuation = 0, int enableSWR = 0, int sourceSWR = 0);
	int Lms8fe_Cmd_Configure(lms_device_t *dev, LMS8FE_COM com, lms8fe_boardState state);
//	int Lms8fe_Cmd_Mode(lms_device_t *dev, LMS8FE_COM com, int mode);
//	int Lms8fe_Cmd_ReadADC(lms_device_t *dev, LMS8FE_COM com, int adcID, int *value);
//	int Lms8fe_Cmd_Cmd(lms_device_t *dev, LMS8FE_COM com, unsigned char *buf);
//	int Lms8fe_Cmd_ConfGPIO(lms_device_t *dev, LMS8FE_COM com, int gpioNum, int direction);
//	int Lms8fe_Cmd_SetGPIO(lms_device_t *dev, LMS8FE_COM com, int gpioNum, int val);
//	int Lms8fe_Cmd_GetGPIO(lms_device_t *dev, LMS8FE_COM com, int gpioNum, int *val);
//	int Lms8fe_Cmd_Fan(lms_device_t *dev, LMS8FE_COM com, int enable);

	// milans 220429
//	int Lms8fe_Cmd_Diode(lms_device_t *dev, LMS8FE_COM com, int state);
	// milans 220506
//	int Lms8fe_Cmd_DiodeSPI(lms_device_t *dev, LMS8FE_COM com, int state);
	// milans 220520
	int Lms8fe_Cmd_SC1905_SPI_Message_Memory(lms_device_t *dev, LMS8FE_COM com, uint16_t address, uint8_t *val, bool isRead, int bytesNo, bool isEEPROM = false);
	// milans 220526
	int Lms8fe_Cmd_SC1905_Reset(lms_device_t *dev, LMS8FE_COM com);
	// milans 220527
	int Lms8fe_Cmd_SC1905_Apply_Frequency(lms_device_t *dev, LMS8FE_COM com, int freqRange, int minFreq, int maxFreq);
	// milans 220530
	int Lms8fe_Cmd_SC1905_SPI_Special_Command(lms_device_t *dev, LMS8FE_COM com, int command);
	// milans 220624
	int Lms8fe_Cmd_Set_Config_Full(lms_device_t *dev, LMS8FE_COM com, uint8_t *state, int size);
	// milans 220715
	int Lms8fe_Cmd_Get_Config_Full(lms_device_t *dev, LMS8FE_COM com, uint8_t *state, int size);

	//	int ReadConfig(const char *filename, lms8fe_boardState *stateBoard, guiState *stateGUI);
	int Lms8fe_ReadConfig(const char *filename, lms8fe_boardState *stateBoard);

	// milans 220722
	//	int Lms8fe_SaveConfig(const char *filename, lms8fe_boardState state, guiState stateGUI);
	int Lms8fe_SaveConfig(const char *filename, lms8fe_boardState state);

	/************************************************************************
	 * I2C Functions
	 *************************************************************************/
/*
	void Lms8fe_mySleep(double sleepms);
	void Lms8fe_i2c_dly(void);
	int Lms8fe_i2c_setVal(lms_device_t *lms, int bitGPIO, int value);
	int Lms8fe_i2c_getVal(lms_device_t *lms, int bitGPIO);
	int Lms8fe_i2c_start(lms_device_t *lms);
	int Lms8fe_i2c_stop(lms_device_t *lms);
	int Lms8fe_i2c_rx(lms_device_t *lms, char ack, unsigned char *d);
	int Lms8fe_i2c_tx(lms_device_t *lms, unsigned char d);
	int Lms8fe_i2c_write_buffer(lms_device_t *lms, unsigned char *c, int size);
	int Lms8fe_i2c_read_buffer(lms_device_t *lms, unsigned char *c, int size);
*/
//B.J.
int Lms8fe_SPI_write(lms_device_t *lms, uint16_t maddress, uint16_t address, uint16_t data);
int Lms8fe_SPI_read(lms_device_t *lms, uint16_t maddress, uint16_t address, uint16_t *data);

#if __cplusplus
}
#endif

#endif // __limeLMS8FE_constants__
