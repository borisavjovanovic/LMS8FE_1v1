/**
 * @file limeLMS8FE/limeLMS8FE.h
 *
 * @brief LimeLMS8FE API functions
 *
 * Copyright (C) 2016 Lime Microsystems
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __limeLMS8FE__
#define __limeLMS8FE__

// milans 220420
// #include "LimeSuite.h"
#include "LimeSuite.h"
#include <cstdint>

#define LMS8FE_CH1 0
#define LMS8FE_CH2 1

/// LimeLMS8FE I2C address
#define LMS8FE_I2C_ADDRESS 0x51

/// LimeLMS8FE Channel IDs
/*
#define LMS8FE_CID_WB_1000 0
#define LMS8FE_CID_WB_4000 1
#define LMS8FE_CID_HAM_0030 2
#define LMS8FE_CID_HAM_0145 3
#define LMS8FE_CID_HAM_0435 4
#define LMS8FE_CID_HAM_1280 5
#define LMS8FE_CID_HAM_2400 6
#define LMS8FE_CID_HAM_3500 7
#define LMS8FE_CID_CELL_BAND01 8
#define LMS8FE_CID_CELL_BAND02 9
#define LMS8FE_CID_CELL_BAND03 10
#define LMS8FE_CID_CELL_BAND07 11
#define LMS8FE_CID_CELL_BAND38 12
#define LMS8FE_CID_COUNT 13
#define LMS8FE_CID_AUTO (-2)
*/
/*
#define LMS8FE_CID_WB_1000 1
#define LMS8FE_CID_WB_4000 2
#define LMS8FE_CID_HAM_0030 3
#define LMS8FE_CID_HAM_0070 4
#define LMS8FE_CID_HAM_0145 5
#define LMS8FE_CID_HAM_0220 6
#define LMS8FE_CID_HAM_0435 7
#define LMS8FE_CID_HAM_0920 8
#define LMS8FE_CID_HAM_1280 9
#define LMS8FE_CID_HAM_2400 10
#define LMS8FE_CID_HAM_3500 11
#define LMS8FE_CID_CELL_BAND01 12
#define LMS8FE_CID_CELL_BAND02 13
#define LMS8FE_CID_CELL_BAND03 14
#define LMS8FE_CID_CELL_BAND07 15
#define LMS8FE_CID_CELL_BAND38 16
#define LMS8FE_CID_AUTO (-2)
#define LMS8FE_CID_NOT_SELECTED 100
*/
/// LimeLMS8FE Ports
//#define LMS8FE_PORT_1 1 ///< Connector J3 - 'TX/RX'
//#define LMS8FE_PORT_2 2 ///< Connector J4 - 'TX'
//#define LMS8FE_PORT_3 3 ///< Connector J5 - '30 MHz TX/RX'

/// LimeLMS8FE convenience constants for notch on/off control
//#define LMS8FE_NOTCH_OFF 0
//#define LMS8FE_NOTCH_ON 1

/// LimeLMS8FE Modes
//#define LMS8FE_MODE_RX 0   ///< RX - Enabled; TX - Disabled
//#define LMS8FE_MODE_TX 1   ///< RX - Disabled; TX - Enabled
//#define LMS8FE_MODE_NONE 2 ///< RX - Disabled; TX - Disabled
//#define LMS8FE_MODE_TXRX 3 ///< RX - Enabled; TX - Enabled

/// LimeLMS8FE ADC constants
//#define LMS8FE_ADC1 0		///< ADC #1, this ADC value is proportional to output power in dB.
//#define LMS8FE_ADC2 1		///< ADC #2, this ADC value is proportional to reflection coefficient in dB.
//#define LMS8FE_ADC_VREF 5.0 ///< ADC referent voltage
//#define LMS8FE_ADC_BITS 10	///< ADC resolution

/// LimeLMS8FE error codes
#define LMS8FE_SUCCESS 0
#define LMS8FE_ERROR_COMM_SYNC -4 ///< Error synchronizing communication
#define LMS8FE_ERROR_GPIO_PIN -3  ///< Non-configurable GPIO pin specified. Only pins 4 and 5 are configurable.
#define LMS8FE_ERROR_CONF_FILE -2 ///< Problem with .ini configuration file
#define LMS8FE_ERROR_COMM -1	  ///< Communication error

// milans - Delete these old codes, I kept them temporarily to be able to compile
#define LMS8FE_ERROR_TX_CONN 0xF1			   ///< Wrong TX connector - not possible to route TX of the selecrted channel to the specified port
#define LMS8FE_ERROR_RX_CONN 0xF2			   ///< Wrong RX connector - not possible to route RX of the selecrted channel to the specified port
#define LMS8FE_ERROR_RXTX_SAME_CONN 0xF3	   ///< Mode TXRX not allowed - when the same port is selected for RX and TX, it is not allowed to use mode RX & TX
#define LMS8FE_ERROR_CELL_WRONG_MODE 0xF4	   ///< Wrong mode for cellular channel - Cellular FDD bands (1, 2, 3, and 7) are only allowed mode RX & TX, while TDD band 38 is allowed only RX or TX mode
#define LMS8FE_ERROR_CELL_TX_NOT_EQUAL_RX 0xF5 ///< Cellular channels must be the same both for RX and TX
#define LMS8FE_ERROR_WRONG_CHANNEL_CODE 0xF6   ///< Requested channel code is wrong

#define LMS8FE_ERROR_SC1905_RSR 1 ///< RSR problem: Either the initial value was wrong, or the timeout has occured
#define LMS8FE_ERROR_SC1905_CHK 2 ///< RSR CHKblem: Checksum value does not match the Message Reply Buffer value
#define LMS8FE_ERROR_SC1905_EEPROM_BUSY 3
#define LMS8FE_ERROR_SC1905_EEPROM_LOCKED 4
#define LMS8FE_ERROR_SC1905_EEPROM_UNLOCKED 5
// milans 220530
#define LMS8FE_ERROR_SC1905_SPECIAL_MESSAGE_UNKNOWN 0xa1	 ///< SC1905 Special Message unknown or not implemented
#define LMS8FE_ERROR_SC1905_SPECIAL_MESSAGE_WRONG_REPLY 0xa2 ///< SC1905 provided wrong reply to Special Message
#define LMS8FE_ERROR_SC1905_MAXPWRCLEARONGOING_TOO_LONG 0xa3 ///< SC1905 provided wrong reply to Special Message

#define LMS8FE_SC1905_MAXPWRCLEARONGOING_ATTEMPTS 20 ///< Max number of attempts to read the MaxPWRClearOnGOing flag until value 0x00 is returned

/// LimeLMS8FE configurable GPIO pins
#define LMS8FE_GPIO4 4
#define LMS8FE_GPIO5 5

/// LimeLMS8FE configurable GPIO pins direction
#define LMS8FE_GPIO_DIR_OUT 0 ///< Output
#define LMS8FE_GPIO_DIR_IN 1  ///< Input

/// LimeLMS8FE SWR subsystem enable
#define LMS8FE_SWR_DISABLE 0 ///< Disable
#define LMS8FE_SWR_ENABLE 1	 ///< Enable

/// LimeLMS8FE SWR subsystem source
#define LMS8FE_SWR_SRC_EXT 0  ///< External - SWR signals are supplied to the connectors J18 for forward and J17 for forward signal. To be supplied from the external coupler.
#define LMS8FE_SWR_SRC_CELL 1 ///< Cellular - Power Meter signal is provided internally from the cellular TX amplifier outputs.

#define LMS8FE_SC1905_COMMAND_CLEAR_WARNING 0
#define LMS8FE_SC1905_COMMAND_ACTIVATE_OUTPUTS 1
#define LMS8FE_SC1905_COMMAND_WRITE_MAXPWRCALPARAM_A 2
#define LMS8FE_SC1905_COMMAND_WRITE_MAXPWRCALPARAM_B 3
#define LMS8FE_SC1905_COMMAND_CLEAR_MAXPWRCALPARAM_A 4
#define LMS8FE_SC1905_COMMAND_CLEAR_MAXPWRCALPARAM_B 5
// Add additional commands if needed

#define LMS8FE_SC1905_FREQUENCY_A 0
#define LMS8FE_SC1905_FREQUENCY_B 1

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/
#define CHAIN_SIZE 6 // chain bytes
// #define STATE_SIZE                   7  // board state bytes (chain bytes + MCU STATE byte)
#define STATE_SIZE 8 // board state bytes (chain bytes + MCU STATE byte + MISC byte)

#define SPI_2_MCU_DIR_OUT_in_BYTE 0
#define SPI_2_MCU_DIR_OUT_in_BIT 0

#define LMS8001_1_SSENn_BYTE 0
#define LMS8001_1_SSENn_BIT 1

#define LMS8001_2_SSENn_BYTE 0
#define LMS8001_2_SSENn_BIT 2

#define EXT_PLL_SSENn_BYTE 0
#define EXT_PLL_SSENn_BIT 3

#define LMS8001_1_RESETn_BYTE 0
#define LMS8001_1_RESETn_BIT 4

#define LMS8001_2_RESETn_BYTE 0
#define LMS8001_2_RESETn_BIT 5

#define SC1905_1_SSENn_BYTE 0
#define SC1905_1_SSENn_BIT 6

#define SC1905_2_SSENn_BYTE 0
#define SC1905_2_SSENn_BIT 7

#define GPIO_SEL_A_LMS8001_BYTE 1
#define GPIO_SEL_A_LMS8001_BIT 0

#define SC1905_1_RESETn_BYTE 1
#define SC1905_1_RESETn_BIT 1

#define SC1905_2_RESETn_BYTE 1
#define SC1905_2_RESETn_BIT 2

#define BYPASS_AMP1_BYTE 2
#define BYPASS_AMP1_BIT 0

#define DISABLE_AMP1_BYTE 2
#define DISABLE_AMP1_BIT 1

#define BYPASS_AMP2_BYTE 2
#define BYPASS_AMP2_BIT 2

#define DISABLE_AMP2_BYTE 2
#define DISABLE_AMP2_BIT 3

#define PA1_A_EN_BYTE 3
#define PA1_A_EN_BIT 0

#define PA1_B_EN_BYTE 3
#define PA1_B_EN_BIT 1

#define PA2_A_EN_BYTE 3
#define PA2_A_EN_BIT 2

#define PA2_B_EN_BYTE 3
#define PA2_B_EN_BIT 3

#define LNA1_EN_BYTE 3
#define LNA1_EN_BIT 4

#define LNA2_EN_BYTE 3
#define LNA2_EN_BIT 5

#define DA1_EN_BYTE 3
#define DA1_EN_BIT 6

#define DA2_EN_BYTE 3
#define DA2_EN_BIT 7

#define PA1_A_B_CTRL_BYTE 4
#define PA1_A_B_CTRL_BIT 0

#define PA2_A_B_CTRL_BYTE 4
#define PA2_A_B_CTRL_BIT 1

#define PA1_CPL_D0_BYTE 4
#define PA1_CPL_D0_BIT 2

#define PA1_CPL_D1_BYTE 4
#define PA1_CPL_D1_BIT 3

#define PA1_CPL_D2_BYTE 4
#define PA1_CPL_D2_BIT 4

#define PA1_CPL_D3_BYTE 4
#define PA1_CPL_D3_BIT 5

#define PA1_CPL_D4_BYTE 4
#define PA1_CPL_D4_BIT 6

#define PA1_CPL_D5_BYTE 4
#define PA1_CPL_D5_BIT 7

#define PA1_CPL_D6_BYTE 5
#define PA1_CPL_D6_BIT 0

#define PA2_CPL_D0_BYTE 5
#define PA2_CPL_D0_BIT 1

#define PA2_CPL_D1_BYTE 5
#define PA2_CPL_D1_BIT 2

#define PA2_CPL_D2_BYTE 5
#define PA2_CPL_D2_BIT 3

#define PA2_CPL_D3_BYTE 5
#define PA2_CPL_D3_BIT 4

#define PA2_CPL_D4_BYTE 5
#define PA2_CPL_D4_BIT 5

#define PA2_CPL_D5_BYTE 5
#define PA2_CPL_D5_BIT 6

#define PA2_CPL_D6_BYTE 5
#define PA2_CPL_D6_BIT 7

// MCU state byte

#define MCU_BYTE 6
#define MCU_TXRX_1_BIT 0
#define MCU_TXRX_2_BIT 1

// MISC byte

#define MISC_BYTE 7
#define MISC_CHANNEL_BIT 0

/*************************************************************************************/
/*************************************************************************************/
/*************************************************************************************/

/// LimeLMS8FE board configuration parameters
typedef struct
{
	//	char channelIDRX;	///<RX channel ID (convenience constants defined in limeLMS8FE.h).For example constant LMS8FE_CID_HAM_0145 identifies 2m(144 - 146 MHz) HAM channel.
	//	char channelIDTX;	///<TX channel ID (convenience constants defined in limeLMS8FE.h).For example constant LMS8FE_CID_HAM_0145 identifies 2m(144 - 146 MHz) HAM channel.If - 1 then the same channel as for RX is used.
	//	char selPortRX;	///<RX port (convenience constants defined in limeLMS8FE.h).
	//	char selPortTX;	///<TX port (convenience constants defined in limeLMS8FE.h).
	//	char mode;	///<Operation mode (defined in limeLMS8FE.h). Not all modes all applicable to all congfigurations.HAM channels using same port for RX and TX are not allowed LMS8FE_MODE_TXRX mode. Cellular FDD bands 1, 2, 3, and 7 are always in LMS8FE_MODE_TXRX mode.Cellular TDD band 38 can not be in LMS8FE_MODE_TXRX.
	//	char notchOnOff;	///<Specifies whether the notch filter is applied or not (convenience constants defined in limeLMS8FE.h).
	//	char attValue;	///<Specifies the attenuation in the RX path. Attenuation [dB] = 2 * attenuation.
	//	char enableSWR;	///<Enable SWR subsystem. (convenience constants defined in limeLMS8FE.h).
	//	char sourceSWR;	///<SWR subsystem source. (convenience constants defined in limeLMS8FE.h).

	char SPI_2_MCU_DIR_OUT_in;
	char LMS8001_1_SSENn;
	char LMS8001_2_SSENn;
	char EXT_PLL_SSENn;
	char LMS8001_1_RESETn;
	char LMS8001_2_RESETn;
	char SC1905_1_SSENn;
	char SC1905_2_SSENn;
	char GPIO_SEL_A_LMS8001;
	char SC1905_1_RESETn;
	char SC1905_2_RESETn;
	char BP_AMP1;
	char SD_AMP1;
	char BP_AMP2;
	char SD_AMP2;
	char PA1_A_EN;
	char PA1_B_EN;
	char PA2_A_EN;
	char PA2_B_EN;
	char LNA1_EN;
	char LNA2_EN;
	char DA1_EN;
	char DA2_EN;
	char PA1_A_B_CTRL;
	char PA2_A_B_CTRL;
	char PA1_CPL_ATT;
	char PA2_CPL_ATT;
	char TXRX_1;
	char TXRX_2;
	char ActiveChannel;
} lms8fe_boardState;

#ifdef __cplusplus
extern "C"
{
#endif

#if defined _WIN32 || defined __CYGWIN__
#define CALL_CONV __cdecl
#ifdef __GNUC__
#define API_EXPORT __attribute__((dllexport))
#else
#define API_EXPORT __declspec(dllexport)
#endif
#elif defined _DOXYGEN_ONLY_
/** Marks an API routine to be made visible to the dynamic loader.
 *  This is OS and/or compiler-specific. */
#define API_EXPORT
/** Specifies calling convention, if necessary.
 *  This is OS and/or compiler-specific. */
#define CALL_CONV
#else
#define API_EXPORT __attribute__((visibility("default")))
#define CALL_CONV
#endif

	typedef void lms8fe_dev_t;
	/****************************************************************************
	 *   LimeLMS8FE API Functions
	 *****************************************************************************/
	/**
	 * @defgroup LMS8FE_API    LimeLMS8FE API Functions
	 *
	 * @{
	 */
	/**
	 *This functions opens LimeLMS8FE device. Connection can be direct via USB or through SDR board.
	 *
	 * @param serialport  Serial port name, (e.g. COM3) for control via USB. NULL if LimeLMS8FE is controlled via SDR.
	 * @param dev         LimeSDR device obtained by invoking LMS_Open. May be NULL if direct USB connection is used.
	 *
	 * @return            Positive number on success, (-1) on failure
	 */
	API_EXPORT lms8fe_dev_t *CALL_CONV LMS8FE_Open(const char *serialport, lms_device_t *dev);

	/**
	 *This function closes the device previously opened with LMS8FE_Open.
	 *
	 * @param lms8fe         handle previously obtained from invoking LMS8FE_Open.
	 *
	 * @return            None
	 */
	API_EXPORT void CALL_CONV LMS8FE_Close(lms8fe_dev_t *lms8fe);

	/**
	 *This function gets the firmware and hardware version, as well as 2 status bytes (reserved for future use).
	 *
	 * @param lms8fe         handle previously obtained from invoking LMS8FE_Open.
	 * @param cinfo       Board info: cinfo[0] - Firmware version; cinfo[1] - Hardware version; cinfo[2] - Status (reserved for future use); cinfo[3] - Status (reserved for future use)
	 *
	 * @return             0 on success, other on failure (see LimeLMS8FE error codes)
	 */
	API_EXPORT int CALL_CONV LMS8FE_GetInfo(lms8fe_dev_t *lms8fe, unsigned char *cinfo);

	/**
	 *This function loads LimeLMS8FE configuration from an .ini file, and configures the board accordingly.
	 *
	 * @param lms8fe         handle previously obtained from invoking LMS8FE_Open.
	 * @param filename    Full path to .ini configuration file
	 *
	 * @return            0 on success, other on failure (see LimeLMS8FE error codes)
	 */
	API_EXPORT int CALL_CONV LMS8FE_LoadConfig(lms8fe_dev_t *lms8fe, const char *filename);

	/**
	 *This function Resets the board.
	 *
	 * @param lms8fe         handle previously obtained from invoking LMS8FE_Open.
	 *
	 * @return            0 on success, other on failure (see LimeLMS8FE error codes)
	 */
	API_EXPORT int CALL_CONV LMS8FE_Reset(lms8fe_dev_t *lms8fe);

//milans 221128
	API_EXPORT int CALL_CONV LMS8FE_LMS8_Enable(lms8fe_dev_t* lms8fe, int value);
//milans 221130
	API_EXPORT int CALL_CONV LMS8FE_Select_Channel(lms8fe_dev_t* lms8fe, int channel);
//milans 221201
	API_EXPORT int CALL_CONV LMS8FE_SetState(lms8fe_dev_t* lms8fe, lms8fe_boardState state);
	API_EXPORT int CALL_CONV LMS8FE_GetState(lms8fe_dev_t* lms8fe, lms8fe_boardState* state);

	/**
	 *This function configures the LimeLMS8FE board.
	 *
	 * @param lms8fe         handle previously obtained from invoking LMS8FE_Open.
	 * @param channelIDRX   RX channel to be acitvated (convenience constants defined in limeLMS8FE.h). For example constant LMS8FE_CID_HAM_0145 identifies 2m (144 - 146 MHz) HAM channel.
	 * @param channelIDTX   TX channel to be acitvated (convenience constants defined in limeLMS8FE.h). For example constant LMS8FE_CID_HAM_0145 identifies 2m (144 - 146 MHz) HAM channel. If -1 then the same channel as for RX is used.
	 * @param portRX        RX port (convenience constants defined in limeLMS8FE.h).
	 * @param portTX        TX port (convenience constants defined in limeLMS8FE.h).
	 * @param mode          Operation mode (defined in limeLMS8FE.h). Not all modes all applicable to all congfigurations. HAM channels using same port for RX and TX are not allowed LMS8FE_MODE_TXRX mode. Cellular FDD bands 1, 2, 3, and 7 are always in LMS8FE_MODE_TXRX mode. Cellular TDD band 38 can not be in LMS8FE_MODE_TXRX.
	 * @param notch         Specifies whether the notch filter is applied or not (convenience constants defined in limeLMS8FE.h).
	 * @param attenuation   Specifies the attenuation in the RX path. Attenuation [dB] = 2 * attenuation.
	 * @param enableSWR     Enable SWR subsystem. (convenience constants defined in limeLMS8FE.h).
	 * @param sourceSWR     SWR subsystem source. (convenience constants defined in limeLMS8FE.h).
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
	//API_EXPORT int CALL_CONV LMS8FE_Configure(lms8fe_dev_t *lms8fe, char channelIDRX, char channelIDTX, char portRX, char portTX, char mode, char notch, char attenuation, char enableSWR, char sourceSWR);

	/**
	 *This function configures the LimeLMS8FE board. It's functionality is identical to LMS8FE_Configure, with different arguments.
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param state         Structure containing configuration parameters.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
	//API_EXPORT int LMS8FE_ConfigureState(lms8fe_dev_t *lms8fe, lms8fe_boardState state);

	/**
	 *This function gets the state of the LimeLMS8FE board. It's functionality is identical to Cmd_GetConfig internal command
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param state         Pointer to structure where configuration parameters are returned.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
	//API_EXPORT int LMS8FE_GetState(lms8fe_dev_t *lms8fe, lms8fe_boardState *state);

	/**
	 *This function sets the LimeLMS8FE mode (receive, transmit, both, or none)
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param mode          Operation mode (convenience constants defined in limeLMS8FE.h). Not all modes all applicable to all congfigurations. HAM channels using same port for RX and TX are not allowed LMS8FE_MODE_TXRX mode. Cellular FDD bands 1, 2, 3, and 7 are always in LMS8FE_MODE_TXRX mode. Cellular TDD band 38 can not be in LMS8FE_MODE_TXRX.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
	//API_EXPORT int CALL_CONV LMS8FE_Mode(lms8fe_dev_t *lms8fe, int mode);

	/**
	 *This function reads the value of the speficied ADC.
	 *
	 * @param lms8fe         handle previously obtained from invoking LMS8FE_Open.
	 * @param adcID         Specifies which ADC is to be read (convenience constants defined in limeLMS8FE.h).
	 * @param value         ADC value.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int CALL_CONV LMS8FE_ReadADC(lms8fe_dev_t *lms8fe, int adcID, int *value);

	/**
	 *This function configures GPIO pin. Only pins 4 and 5 are configurable.
	 *
	 * @param lms8fe         handle previously obtained from invoking LMS8FE_Open.
	 * @param gpioNum       GPIO pin number. Only pins 4 and 5 are configurable.
	 * @param direction     GPIO pin direction (convenience constants defined in limeLMS8FE.h). 0 - Output; 1 - Input.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int CALL_CONV LMS8FE_ConfGPIO(lms8fe_dev_t *lms8fe, int gpioNum, int direction);

	/**
	 *This function sets the GPIO pin value. GPIO pin should have been previously configured as output using LMS8FE_ConfGPIO function.
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param gpioNum       GPIO pin number. Only pins 4 and 5 are configurable.
	 * @param val           GPIO pin value.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int CALL_CONV LMS8FE_SetGPIO(lms8fe_dev_t *lms8fe, int gpioNum, int val);

	/**
	 *This function reads the GPIO pin value. GPIO pin should have been previously configured as input using LMS8FE_ConfGPIO function.
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param gpioNum       GPIO pin number. Only pins 4 and 5 are configurable.
	 * @param val           GPIO pin value.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int CALL_CONV LMS8FE_GetGPIO(lms8fe_dev_t *lms8fe, int gpioNum, int *val);

	/**
	 * Links LimeLMS8FE Rx and Tx to specific SDR boards channels for automatic band
	 * selection and RF switching purposes. By default channel 0 is used, so this
	 * function is only needed if different channel is going to be used.
	 *
	 * @param lms8fe        handle previously obtained from invoking LMS8FE_Open.
	 * @param rxChan     Rx channel index
	 * @param txChan     Tx channels index
	 *
	 * @return            0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int CALL_CONV LMS8FE_AssignSDRChannels(lms8fe_dev_t *lms8fe, int rxChan, int txChan);

	/**
	 *This function enables/disables the Lime_LMS8FE fan.
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param enable        fan state: 0 - disable; 1 - enable.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int CALL_CONV LMS8FE_Fan(lms8fe_dev_t *lms8fe, int enable);

	/**
	 *This function is TEST !!!.
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param state         diode state: 0 - OFF; 1 - ON.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int LMS8FE_Diode(lms8fe_dev_t *lms8fe, int state);

	/**
	 *This function is TEST !!!. Control diode on through SPI.
	 *
	 * @param lms8fe           handle previously obtained from invoking LMS8FE_Open.
	 * @param state         diode state: 0 - OFF; 1 - ON.
	 *
	 * @return              0 on success, other on failure (see LimeLMS8FE error codes)
	 */
//	API_EXPORT int LMS8FE_DiodeSPI(lms8fe_dev_t *lms8fe, int state);

	/**
	 *This function is TEST !!! Read or write the register in SC1905 using SPI
	 *
	 */
	API_EXPORT int LMS8FE_SC1905_SPI_Message_Memory(lms8fe_dev_t *lms8fe, uint16_t address, uint8_t *val, bool isRead, int bytesNo, bool isEEPROM = false);

	/**
	 *This function is TEST !!! Send special command to CS1905
	 *
	 */
	API_EXPORT int LMS8FE_SC1905_SPI_Special_Command(lms8fe_dev_t *lms8fe, int command);

	/**
	 *This function is TEST !!! Read or write the register in SC1905 using SPI
	 *
	 */
	API_EXPORT int LMS8FE_SC1905_Reset(lms8fe_dev_t *lms8fe);

	/**
	 *This function is TEST !!! Set the frequencies of SC1905
	 *
	 */
	API_EXPORT int LMS8FE_SC1905_Apply_Frequency(lms8fe_dev_t *lms8fe, int freqRange, int minFreq, int maxFreq);

	/**
	 *This function is TEST !!! Send full configuration data to LMS8FE
	 *
	 */
	API_EXPORT int LMS8FE_Set_Config_Full(lms8fe_dev_t *lms8fe, uint8_t *state, int size);

	/**
	 *This function is TEST !!! Read full configuration data from LMS8FE
	 *
	 */
	API_EXPORT int LMS8FE_Get_Config_Full(lms8fe_dev_t *lms8fe, uint8_t *state, int size);

	/**
	 *This function is TEST !!! Get the LMS8001 device, to be used with LMS8 API functions
	 *
	 */
	extern "C" API_EXPORT int LMS8FE_LMS8_Open(lms8fe_dev_t * lms8fe, lms_device_t * *device);

	/** @} (End LMS8FE_API) */

		// B.J. temprary
	API_EXPORT int LMS8FE_SPI_write(lms8fe_dev_t *lms8fe, uint16_t maddress, uint16_t address, uint16_t data);
	API_EXPORT int LMS8FE_SPI_read(lms8fe_dev_t *lms8fe, uint16_t maddress, uint16_t address, uint16_t * pData);
	API_EXPORT int LMS8FE_SPI_write_buffer(lms_device_t *lms, unsigned char *c, int size);
	API_EXPORT int LMS8FE_SPI_read_buffer(lms_device_t *lms, unsigned char *c, int size);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __limeLMS8FE__
