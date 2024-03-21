/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: See git blame.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"
#include "glaux.h"
#include "warp.h"
#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "Activity-Classifier.h"


#define							kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define							kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define							kWarpConstantStringErrorSanity		"\rSanity check failed!"

#if (WARP_BUILD_ENABLE_DEVAT45DB || WARP_BUILD_ENABLE_DEVIS25xP)
	#define WARP_BUILD_ENABLE_FLASH 1
#else
	#define WARP_BUILD_ENABLE_FLASH 0
#endif

#define BUFFER_LENGTH_ACCEL 200 //4 seconds at 50Hz
#define BUFFER_LENGTH_CLASSIFICATION 100
#define NUM_AXIS 1

uint16_t accelerationBuffer[BUFFER_LENGTH_ACCEL];
uint8_t		classificationBuffer[BUFFER_LENGTH_CLASSIFICATION];

int bufferIndexAccel = 0;
int bufferIndexClassification = 0;

void addToBuffer16(uint16_t *buffer, size_t bufferSize, int *bufferIndex, uint16_t value);
void addToBuffer8(uint8_t *buffer, size_t bufferSize, int *bufferIndex, uint8_t value);

#include "devMMA8451Q.h"

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
	volatile WarpI2CDeviceState			deviceMMA8451QState;
#endif

typedef enum
{
	kWarpFlashReadingCountBitField 	= 0b1,
	kWarpFlashRTCTSRBitField 		= 0b10,
	kWarpFlashRTCTPRBitField 		= 0b100,
	kWarpFlashADXL362BitField 		= 0b1000,
	kWarpFlashAMG8834BitField 		= 0b10000,
	kWarpFlashMMA8541QBitField		= 0b100000,
	kWarpFlashMAG3110BitField		= 0b1000000,
	kWarpFlashL3GD20HBitField		= 0b10000000,
	kWarpFlashBME680BitField		= 0b100000000,
	kWarpFlashBMX055BitField		= 0b1000000000,
	kWarpFlashCCS811BitField		= 0b10000000000,
	kWarpFlashHDC1000BitField		= 0b100000000000,
	kWarpFlashRV8803C7BitField		= 0b100000000000000,
	kWarpFlashNumConfigErrors		= 0b1000000000000000,
} WarpFlashSensorBitFieldEncoding;

volatile i2c_master_state_t		  i2cMasterState;
volatile spi_master_state_t		  spiMasterState;
volatile spi_master_user_config_t spiUserConfig;
volatile lpuart_user_config_t	  lpuartUserConfig;
volatile lpuart_state_t			  lpuartState;

volatile bool		  gWarpBooted						   = false;
volatile uint32_t	  gWarpI2cBaudRateKbps				   = kWarpDefaultI2cBaudRateKbps;
volatile uint32_t	  gWarpUartBaudRateBps				   = kWarpDefaultUartBaudRateBps;
volatile uint32_t	  gWarpSpiBaudRateKbps				   = kWarpDefaultSpiBaudRateKbps;
volatile uint32_t	  gWarpSleeptimeSeconds				   = kWarpDefaultSleeptimeSeconds;
volatile WarpModeMask gWarpMode							   = kWarpModeDisableAdcOnSleep;
volatile uint32_t	  gWarpI2cTimeoutMilliseconds		   = kWarpDefaultI2cTimeoutMilliseconds;
volatile uint32_t	  gWarpSpiTimeoutMicroseconds		   = kWarpDefaultSpiTimeoutMicroseconds;
volatile uint32_t	  gWarpUartTimeoutMilliseconds		   = kWarpDefaultUartTimeoutMilliseconds;
volatile uint32_t	  gWarpMenuPrintDelayMilliseconds	   = kWarpDefaultMenuPrintDelayMilliseconds;
volatile uint32_t	  gWarpSupplySettlingDelayMilliseconds = kWarpDefaultSupplySettlingDelayMilliseconds;
volatile uint16_t	  gWarpCurrentSupplyVoltage			   = kWarpDefaultSupplyVoltageMillivolts;

char		  gWarpPrintBuffer[kWarpDefaultPrintBufferSizeBytes];

#if WARP_BUILD_EXTRA_QUIET_MODE
	volatile bool gWarpExtraQuietMode = true;
#else
	volatile bool gWarpExtraQuietMode = false;
#endif

/*
 *	Since only one SPI transaction is ongoing at a time in our implementaion
 */
uint8_t							gWarpSpiCommonSourceBuffer[kWarpMemoryCommonSpiBufferBytes];
uint8_t							gWarpSpiCommonSinkBuffer[kWarpMemoryCommonSpiBufferBytes];

static void						sleepUntilReset(void);
static void						lowPowerPinStates(void);

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	static void					disableTPS62740(void);
	static void					enableTPS62740(uint16_t voltageMillivolts);
	static void					setTPS62740CommonControlLines(uint16_t voltageMillivolts);
#endif

static void						dumpProcessorState(void);
static void						repeatRegisterReadForDeviceAndAddress(WarpSensorDevice warpSensorDevice, uint8_t baseAddress,
								bool autoIncrement, int chunkReadsPerAddress, bool chatty,
								int spinDelay, int repetitionsPerAddress, uint16_t sssupplyMillivolts,
								uint16_t adaptiveSssupplyMaxMillivolts, uint8_t referenceByte);
static int						char2int(int character);
static void						activateAllLowPowerSensorModes(bool verbose);
static void						powerupAllSensors(void);
static uint8_t						readHexByte(void);
static int						read4digits(void);
static void 					writeAllSensorsToFlash(int menuDelayBetweenEachRun, int loopForever);
static void						printAllSensors(bool printHeadersAndCalibration, bool hexModeFlag, int menuDelayBetweenEachRun, bool loopForever);

/*
 *	TODO: change the following to take byte arrays
 */
WarpStatus						writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte);
WarpStatus						writeBytesToSpi(uint8_t *  payloadBytes, int payloadLength);


void							warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState);

/*
* Flash related functions
*/
	WarpStatus					flashReadAllMemory();
#if (WARP_BUILD_ENABLE_FLASH)
	WarpStatus 					flashHandleEndOfWriteAllSensors();
	WarpStatus					flashWriteFromEnd(size_t nbyte, uint8_t* buf);
	WarpStatus					flashReadMemory(uint16_t startPageNumber, uint8_t startPageOffset, size_t nbyte, void *buf);
	void 						flashHandleReadByte(uint8_t readByte, uint8_t *  bytesIndex, uint8_t *  readingIndex, uint8_t *  sensorIndex, uint8_t *  measurementIndex, uint8_t *  currentSensorNumberOfReadings, uint8_t *  currentSensorSizePerReading, uint16_t *  sensorBitField, uint8_t *  currentNumberOfSensors, int32_t *  currentReading);
	uint8_t						flashGetNSensorsFromSensorBitField(uint16_t sensorBitField);
	void						flashDecodeSensorBitField(uint16_t sensorBitField, uint8_t sensorIndex, uint8_t* sizePerReading, uint8_t* numberOfReadings);
#endif

/*
 *	Derived from KSDK power_manager_demo.c BEGIN>>>
 */
clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
			break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	LLW_IRQHandler override. Since FRDM_KL03Z48M is not defined,
 *	according to power_manager_demo.c, what we need is LLW_IRQHandler.
 *	However, elsewhere in the power_manager_demo.c, the code assumes
 *	FRDM_KL03Z48M _is_ defined (e.g., we need to use LLWU_IRQn, not
 *	LLW_IRQn). Looking through the code base, we see in
 *
 *		ksdk1.1.0/platform/startup/MKL03Z4/gcc/startup_MKL03Z4.S
 *
 *	that the startup initialization assembly requires LLWU_IRQHandler,
 *	not LLW_IRQHandler. See power_manager_demo.c, circa line 216, if
 *	you want to find out more about this dicsussion.
 */
void
LLWU_IRQHandler(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	LLWU_HAL_ClearExternalPinWakeupFlag(LLWU_BASE, (llwu_wakeup_pin_t)BOARD_SW_LLWU_EXT_PIN);
}

/*
 *	IRQ handler for the interrupt from RTC, which we wire up
 *	to PTA0/IRQ0/LLWU_P7 in Glaux. BOARD_SW_LLWU_IRQ_HANDLER
 *	is a synonym for PORTA_IRQHandler.
 */
void
BOARD_SW_LLWU_IRQ_HANDLER(void)
{
	/*
	 *	BOARD_* defines are defined in warp.h
	 */
	PORT_HAL_ClearPortIntFlag(BOARD_SW_LLWU_BASE);
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t
callback0(power_manager_notify_struct_t *  notify, power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}
/*
 *	Derived from KSDK power_manager_demo.c <<END
 */



void
sleepUntilReset(void)
{
	while (1)
	{
#if (WARP_BUILD_ENABLE_DEVSI4705)
		GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
#endif

		warpLowPowerSecondsSleep(1, false /* forceAllPinsIntoLowPowerState */);

#if (WARP_BUILD_ENABLE_DEVSI4705)
		GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
#endif

		warpLowPowerSecondsSleep(60, true /* forceAllPinsIntoLowPowerState */);
	}
}


void
enableLPUARTpins(void)
{
	/*
	 *	Enable UART CLOCK
	 */
	CLOCK_SYS_EnableLpuartClock(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *
	 *	TODO: we don't use hw flow control so don't need RTS/CTS
	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt3);

	//TODO: we don't use hw flow control so don't need RTS/CTS
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	//	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MISO_UART_RTS);
	//	GPIO_DRV_SetPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Initialize LPUART0. See KSDK13APIRM.pdf section 40.4.3, page 1353
	 */
	lpuartUserConfig.baudRate = gWarpUartBaudRateBps;
	lpuartUserConfig.parityMode = kLpuartParityDisabled;
	lpuartUserConfig.stopBitCount = kLpuartOneStopBit;
	lpuartUserConfig.bitCountPerChar = kLpuart8BitsPerChar;
	lpuartUserConfig.clockSource = kClockLpuartSrcMcgIrClk;

	LPUART_DRV_Init(0,(lpuart_state_t *)&lpuartState,(lpuart_user_config_t *)&lpuartUserConfig);
}


void
disableLPUARTpins(void)
{
	/*
	 *	LPUART deinit
	 */
	LPUART_DRV_Deinit(0);

	/*
	 *	Set UART pin association. See, e.g., page 99 in
	 *
	 *		https://www.nxp.com/docs/en/reference-manual/KL03P24M48SF0RM.pdf
	 *
	 *	Setup:
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX for UART TX
	 *		PTB4/kWarpPinI2C0_SCL_UART_RX for UART RX
	 *		PTA6/kWarpPinSPI_MISO_UART_RTS for UART RTS
	 *		PTA7/kWarpPinSPI_MOSI_UART_CTS for UART CTS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	/*
	 * We don't use the HW flow control and that messes with the SPI any way
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO_UART_RTS);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI_UART_CTS);

	/*
	 *	Disable LPUART CLOCK
	 */
	CLOCK_SYS_DisableLpuartClock(0);
}

void addToBuffer(uint16_t *buffer, size_t bufferSize, int *bufferIndex, uint16_t value) {
	buffer[*bufferIndex] = value;
	
	*bufferIndex = (*bufferIndex + 1) % bufferSize;
}



WarpStatus
sendBytesToUART(uint8_t *  bytes, size_t nbytes)
{
	lpuart_status_t	status;

	status = LPUART_DRV_SendDataBlocking(0, bytes, nbytes, gWarpUartTimeoutMilliseconds);
	if (status != 0)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
warpEnableI2Cpins(void)
{
	/*
	* Returning here if Glaux variant doesn't work. The program hangs. It seems to be okay if it is done only in the disable function.
	*/
// #if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		// return;
// #else
	CLOCK_SYS_EnableI2cClock(0);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	(ALT2 == I2C)
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	(ALT2 == I2C)
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);

	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);
// #endif
}



void
warpDisableI2Cpins(void)
{
#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
		return;
#else
	I2C_DRV_MasterDeinit(0 /* I2C instance */);

	/*
	 *	Setup:
	 *
	 *		PTB3/kWarpPinI2C0_SCL_UART_TX	-->	disabled
	 *		PTB4/kWarpPinI2C0_SDA_UART_RX	-->	disabled
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);

	CLOCK_SYS_DisableI2cClock(0);
#endif
}


#if (WARP_BUILD_ENABLE_GLAUX_VARIANT)
void
lowPowerPinStates(void)
{
	/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state, except for the
		 *	sacrificial pins (WLCSP package, Glaux) where we set them to disabled. We choose
		 *	to set non-disabled pins to '0'.
	 *
	 *	NOTE: Pin state "disabled" means default functionality is active.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	Leave PTA0/1/2 SWD pins in their default state (i.e., as SWD / Alt3).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
	 *
	 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	/*
	 *	Disable PTA5
	 *
	 *	NOTE: Enabling this significantly increases current draw
	 *	(from ~180uA to ~4mA) and we don't need the RTC on Glaux.
	 *
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

	/*
	 *	PTA6, PTA7, PTA8, and PTA9 on Glaux are SPI and sacrificial SPI.
	 *
	 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
	 *
	 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
	 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */

	/*
		 *	In Glaux, PTA12 is a sacrificial pin for SWD_RESET, so careful not to drive it.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);



	/*
	 *			PORT B
	 *
	 *	PTB0 is LED on Glaux. PTB1 is unused, and PTB2 is FLASH_!CS
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortMuxAsGpio);

	/*
	 *	PTB3 and PTB4 (I2C pins) are true open-drain and we
	 *	purposefully leave them disabled since they have pull-ups.
	 *	PTB5 is sacrificial for I2C_SDA, so disable.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);

	/*
	 *	NOTE:
	 *
	 *	The KL03 has no PTB8, PTB9, or PTB12.  Additionally, the WLCSP package
	 *	we in Glaux has no PTB6, PTB7, PTB10, or PTB11.
	 */

	/*
		 *	In Glaux, PTB13 is a sacrificial pin for SWD_RESET, so careful not to drive it.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);

	GPIO_DRV_SetPinOutput(kGlauxPinFlash_SPI_nCS);
	GPIO_DRV_ClearPinOutput(kGlauxPinLED);

	return;
}
#else
void
lowPowerPinStates(void)
{
	/*
		 *	Following Section 5 of "Power Management for Kinetis L Family" (AN5088.pdf),
		 *	we configure all pins as output and set them to a known state. We choose
		 *	to set them all to '0' since it happens that the devices we want to keep
		 *	deactivated (SI4705) also need '0'.
	 */

	/*
	 *			PORT A
	 */
	/*
	 *	For now, don't touch the PTA0/1/2 SWD pins. Revisit in the future.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

	/*
		 *	PTA3 and PTA4 are the EXTAL0/XTAL0. They are also connected to the clock output
		 *	of the RV8803 (and PTA4 is a sacrificial pin for PTA3), so do not want to drive them.
		 *	We however have to configure PTA3 to Alt0 (kPortPinDisabled) to get the EXTAL0
		 *	functionality.
	 *
	 *	NOTE:	kPortPinDisabled is the equivalent of `Alt0`
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 4, kPortPinDisabled);

	/*
	 *	Disable PTA5
	 *
	 *	NOTE: Enabling this significantly increases current draw
	 *	(from ~180uA to ~4mA) and we don't need the RTC on revC.
	 *
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 5, kPortPinDisabled);

	/*
	 *	Section 2.6 of Kinetis Energy Savings – Tips and Tricks says
	 *
	 *		"Unused pins should be configured in the disabled state, mux(0),
		 *		to prevent unwanted leakage (potentially caused by floating inputs)."
	 *
		 *	However, other documents advice to place pin as GPIO and drive low or high.
		 *	For now, leave disabled. Filed issue #54 low-power pin states to investigate.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 8, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9, kPortPinDisabled);

	/*
	 *	NOTE: The KL03 has no PTA10 or PTA11
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 12, kPortPinDisabled);


	/*
	 *			PORT B
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 1, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 2, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 5, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 6, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 7, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortPinDisabled);
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortPinDisabled);
}
#endif



void
warpScaleSupplyVoltage(uint16_t voltageMillivolts)
{
	if (voltageMillivolts == gWarpCurrentSupplyVoltage)
	{
		return;
	}

#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	if (voltageMillivolts >= 1800 && voltageMillivolts <= 3300)
	{
		enableTPS62740(voltageMillivolts);
		gWarpCurrentSupplyVoltage = voltageMillivolts;
	}
	else
	{
			warpPrint(RTT_CTRL_RESET RTT_CTRL_BG_BRIGHT_RED RTT_CTRL_TEXT_BRIGHT_WHITE kWarpConstantStringErrorInvalidVoltage RTT_CTRL_RESET "\n", voltageMillivolts);
	}
#endif
}



void
warpDisableSupplyVoltage(void)
{
#if (!WARP_BUILD_ENABLE_GLAUX_VARIANT && !WARP_BUILD_ENABLE_FRDMKL03)
	disableTPS62740();

	/*
		 *	Vload ramp time of the TPS62740 is 800us max (datasheet, Table 8.5 / page 6)
	 */
	OSA_TimeDelay(gWarpSupplySettlingDelayMilliseconds);
#endif
}


void
warpLowPowerSecondsSleep(uint32_t sleepSeconds, bool forceAllPinsIntoLowPowerState)
{
	WarpStatus	status = kWarpStatusOK;

	/*
	 *	Set all pins into low-power states. We don't just disable all pins,
	 *	as the various devices hanging off will be left in higher power draw
	 *	state. And manuals say set pins to output to reduce power.
	 */
	if (forceAllPinsIntoLowPowerState)
	{
		lowPowerPinStates();
	}

	warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* Sleep Seconds */);
	if ((status != kWarpStatusOK) && (status != kWarpStatusPowerTransitionErrorVlpr2Vlpr))
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPR, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}

	status = warpSetLowPowerMode(kWarpPowerModeVLPS, sleepSeconds);
	if (status != kWarpStatusOK)
	{
		warpPrint("warpSetLowPowerMode(kWarpPowerModeVLPS, 0 /* sleep seconds : irrelevant here */)() failed...\n");
	}
}

/*
void
printPinDirections(void)
{
	warpPrint("I2C0_SDA:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SDA_UART_RX));
	OSA_TimeDelay(100);
	warpPrint("I2C0_SCL:%d\n", GPIO_DRV_GetPinDir(kWarpPinI2C0_SCL_UART_TX));
	OSA_TimeDelay(100);
	warpPrint("SPI_MOSI:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MOSI_UART_CTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_MISO:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_MISO_UART_RTS));
	OSA_TimeDelay(100);
	warpPrint("SPI_SCK_I2C_PULLUP_EN:%d\n", GPIO_DRV_GetPinDir(kWarpPinSPI_SCK_I2C_PULLUP_EN));
	OSA_TimeDelay(100);
				warpPrint("ADXL362_CS:%d\n", GPIO_DRV_GetPinDir(kWarpPinADXL362_CS));
				OSA_TimeDelay(100);
}
*/



void
dumpProcessorState(void)
{
	uint32_t	cpuClockFrequency;

	CLOCK_SYS_GetFreq(kCoreClock, &cpuClockFrequency);
	warpPrint("\r\n\n\tCPU @ %u KHz\n", (cpuClockFrequency / 1000));
	warpPrint("\r\tCPU power mode: %u\n", POWER_SYS_GetCurrentMode());
	warpPrint("\r\tCPU clock manager configuration: %u\n", CLOCK_SYS_GetCurrentConfiguration());
	warpPrint("\r\tRTC clock: %d\n", CLOCK_SYS_GetRtcGateCmd(0));
	warpPrint("\r\tSPI clock: %d\n", CLOCK_SYS_GetSpiGateCmd(0));
	warpPrint("\r\tI2C clock: %d\n", CLOCK_SYS_GetI2cGateCmd(0));
	warpPrint("\r\tLPUART clock: %d\n", CLOCK_SYS_GetLpuartGateCmd(0));
	warpPrint("\r\tPORT A clock: %d\n", CLOCK_SYS_GetPortGateCmd(0));
	warpPrint("\r\tPORT B clock: %d\n", CLOCK_SYS_GetPortGateCmd(1));
	warpPrint("\r\tFTF clock: %d\n", CLOCK_SYS_GetFtfGateCmd(0));
	warpPrint("\r\tADC clock: %d\n", CLOCK_SYS_GetAdcGateCmd(0));
	warpPrint("\r\tCMP clock: %d\n", CLOCK_SYS_GetCmpGateCmd(0));
	warpPrint("\r\tVREF clock: %d\n", CLOCK_SYS_GetVrefGateCmd(0));
	warpPrint("\r\tTPM clock: %d\n", CLOCK_SYS_GetTpmGateCmd(0));
}


void
printBootSplash(uint16_t gWarpCurrentSupplyVoltage, uint8_t menuRegisterAddress, WarpPowerManagerCallbackStructure *  powerManagerCallbackStructure)
{
	/*
	 *	We break up the prints with small delays to allow us to use small RTT print
	 *	buffers without overrunning them when at max CPU speed.
	 */
	warpPrint("\r\n\n\n\n[ *\t\t\t\tWarp (HW revision C) / Glaux (HW revision B)\t\t\t* ]\n");
	warpPrint("\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
	warpPrint("\r\tSupply=%dmV,\tDefault Target Read Register=0x%02x\n",
			  gWarpCurrentSupplyVoltage, menuRegisterAddress);
	warpPrint("\r\tI2C=%dkb/s,\tSPI=%dkb/s,\tUART=%db/s,\tI2C Pull-Up=%d\n\n",
			  gWarpI2cBaudRateKbps, gWarpSpiBaudRateKbps, gWarpUartBaudRateBps);
	warpPrint("\r\tSIM->SCGC6=0x%02x\t\tRTC->SR=0x%02x\t\tRTC->TSR=0x%02x\n", SIM->SCGC6, RTC->SR, RTC->TSR);
	warpPrint("\r\tMCG_C1=0x%02x\t\t\tMCG_C2=0x%02x\t\tMCG_S=0x%02x\n", MCG_C1, MCG_C2, MCG_S);
	warpPrint("\r\tMCG_SC=0x%02x\t\t\tMCG_MC=0x%02x\t\tOSC_CR=0x%02x\n", MCG_SC, MCG_MC, OSC_CR);
	warpPrint("\r\tSMC_PMPROT=0x%02x\t\t\tSMC_PMCTRL=0x%02x\t\tSCB->SCR=0x%02x\n", SMC_PMPROT, SMC_PMCTRL, SCB->SCR);
	warpPrint("\r\tPMC_REGSC=0x%02x\t\t\tSIM_SCGC4=0x%02x\tRTC->TPR=0x%02x\n\n", PMC_REGSC, SIM_SCGC4, RTC->TPR);
	warpPrint("\r\t%ds in RTC Handler to-date,\t%d Pmgr Errors\n", gWarpSleeptimeSeconds, powerManagerCallbackStructure->errorCount);
}

void
blinkLED(int pin)
{
	GPIO_DRV_SetPinOutput(pin);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(pin);
	OSA_TimeDelay(200);

	return;
}

void
warpPrint(const char *fmt, ...)
{
	if (gWarpExtraQuietMode)
	{
		return;
	}

	int	fmtlen;
	va_list	arg;

/*
 *	We use an ifdef rather than a C if to allow us to compile-out
 *	all references to SEGGER_RTT_*printf if we don't want them.
 *
 *	NOTE: SEGGER_RTT_vprintf takes a va_list* rather than a va_list
 *	like usual vprintf. We modify the SEGGER_RTT_vprintf so that it
 *	also takes our print buffer which we will eventually send over
 *	BLE. Using SEGGER_RTT_vprintf() versus the libc vsnprintf saves
 *	2kB flash and removes the use of malloc so we can keep heap
 *	allocation to zero.
 */
#if (WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF)
	/*
	 *	We can't use SEGGER_RTT_vprintf to format into a buffer
	 *	since SEGGER_RTT_vprintf formats directly into the special
	 *	RTT memory region to be picked up by the RTT / SWD mechanism...
	 */
	va_start(arg, fmt);
		fmtlen = SEGGER_RTT_vprintf(0, fmt, &arg, gWarpPrintBuffer, kWarpDefaultPrintBufferSizeBytes);
	va_end(arg);

	if (fmtlen < 0)
	{
		SEGGER_RTT_WriteString(0, gWarpEfmt);

	#if (WARP_BUILD_ENABLE_DEVBGX)
		if (gWarpBooted)
		{
					WarpStatus	status;

			enableLPUARTpins();
			initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
					status = sendBytesToUART((uint8_t *)gWarpEfmt, strlen(gWarpEfmt)+1);
			if (status != kWarpStatusOK)
			{
				SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
			}
			disableLPUARTpins();

			/*
			 *	We don't want to deInit() the BGX since that would drop
			 *	any remote terminal connected to it.
			 */
					//deinitBGX();
		}
	#endif

		return;
	}

	/*
	 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
	 */
	#if (WARP_BUILD_ENABLE_DEVBGX)
	if (gWarpBooted)
	{
				WarpStatus	status;

		enableLPUARTpins();
		initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);

				status = sendBytesToUART((uint8_t *)gWarpPrintBuffer, max(fmtlen, kWarpDefaultPrintBufferSizeBytes));
		if (status != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
		}
		disableLPUARTpins();

		/*
		 *	We don't want to deInit() the BGX since that would drop
		 *	any remote terminal connected to it.
		 */
				//deinitBGX();
	}
	#endif

#else
	/*
	 *	If we are not compiling in the SEGGER_RTT_printf,
	 *	we just send the format string of warpPrint()
	 */
	SEGGER_RTT_WriteString(0, fmt);

	/*
	 *	If WARP_BUILD_ENABLE_DEVBGX, also send the fmt to the UART / BLE.
	 */
	#if (WARP_BUILD_ENABLE_DEVBGX)
	if (gWarpBooted)
	{
				WarpStatus	status;

		enableLPUARTpins();
		initBGX(kWarpDefaultSupplyVoltageMillivoltsBGX);
		status = sendBytesToUART(fmt, strlen(fmt));
		if (status != kWarpStatusOK)
		{
			SEGGER_RTT_WriteString(0, gWarpEuartSendChars);
		}
		disableLPUARTpins();

		/*
		 *	We don't want to deInit() the BGX since that would drop
		 *	any remote terminal connected to it.
		 */
				//deinitBGX();
	}
	#endif
#endif


	/*
	 *	Throttle to enable SEGGER to grab output, otherwise "run" mode may miss lines.
	 */
	OSA_TimeDelay(5);

	return;
}


int
main(void)
{
	WarpStatus				status;
	uint8_t					key;
	WarpSensorDevice			menuTargetSensor		= kWarpSensorBMX055accel;
	volatile WarpI2CDeviceState *		menuI2cDevice			= NULL;
	uint8_t					menuRegisterAddress		= 0x00;
	rtc_datetime_t				warpBootDate;
	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	/*
	 *	We use this as a template later below and change the .mode fields for the different other modes.
	 */
	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
		/*
		 *	NOTE: POWER_SYS_SetMode() depends on this order
		 *
		 *	See KSDK13APIRM.pdf Section 55.5.3
		 */
		&warpPowerModeWaitConfig,
		&warpPowerModeStopConfig,
		&warpPowerModeVlprConfig,
		&warpPowerModeVlpwConfig,
		&warpPowerModeVlpsConfig,
		&warpPowerModeVlls0Config,
		&warpPowerModeVlls1Config,
		&warpPowerModeVlls3Config,
		&warpPowerModeRunConfig,
	};

	WarpPowerManagerCallbackStructure		powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};

	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);

	/*
	 *	Set board crystal value (Warp revB and earlier).
	 */
	g_xtal0ClkFreq = 32768U;

	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();

	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);

	/*
	 *	When booting to CSV stream, we wait to be up and running as soon as possible after
	 *	a reset (e.g., a reset due to waking from VLLS0)
	 */
	if (!WARP_BUILD_BOOT_TO_CSVSTREAM)
	{
		warpPrint("\n\n\n\rBooting Warp, in 3... ");
		OSA_TimeDelay(1000);
		warpPrint("2... ");
		OSA_TimeDelay(1000);
		warpPrint("1...\n\n\n\r");
		OSA_TimeDelay(1000);
	}

	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM, /* The default value of this is defined in fsl_clock_MKL03Z4.h as 2 */
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);

	/*
	 *	Initialize RTC Driver (not needed on Glaux, but we enable it anyway for now
	 *	as that lets us use the current sleep routines). NOTE: We also don't seem to
	 *	be able to go to VLPR mode unless we enable the RTC.
	 */
	RTC_DRV_Init(0);

	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);

	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));

	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);

	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	warpPrint("About to GPIO_DRV_Init()... ");
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);
	warpPrint("done.\n");

	/*
	 *	Make sure the SWD pins, PTA0/1/2 SWD pins in their ALT3 state (i.e., as SWD).
	 *
	 *	See GitHub issue https://github.com/physical-computation/Warp-firmware/issues/54
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 1, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 2, kPortMuxAlt3);

#if (WARP_BUILD_ENABLE_DEVMMA8451Q)
		initMMA8451Q(	0x1D	/* i2cAddress */,	kWarpDefaultSupplyVoltageMillivoltsMMA8451Q	);
#endif


	do
	{
		WarpStatus status;

		status = readSensorRegisterMMA8451Q(0x05, 2);
		if (status != kWarpStatusOK)
		{
			warpPrint("readSensorRegisterMMA8451Q() failed...\n");
		}

		else
		{
			warpPrint("readSensorRegisterMMA8451Q() succeeded.\n");
		}

		int16_t data_z;
		data_z = (int16_t) ((deviceMMA8451QState.i2cBuffer[0] & 0xFF << 6) | (deviceMMA8451QState.i2cBuffer[1])) >> 2;

		// add to buffer
		addToBuffer16(&accelerationBuffer, BUFFER_LENGTH_ACCEL, &bufferIndexAccel, data_z);
	} while (bufferIndexAccel != 0);
	
	while (1)
	{
		// retrieve triaxial data from MMA8451Q
		WarpStatus status;

		status = readSensorRegisterMMA8451Q(0x05, 2);
		if (status != kWarpStatusOK)
		{
			warpPrint("readSensorRegisterMMA8451Q() failed...\n");
		}

		else
		{
			warpPrint("readSensorRegisterMMA8451Q() succeeded.\n");
		}

		int16_t data_z;
		data_z = (int16_t) ((deviceMMA8451QState.i2cBuffer[0] & 0xFF << 6) | (deviceMMA8451QState.i2cBuffer[1])) >> 2;

		// add to buffer
		addToBuffer16(&accelerationBuffer, BUFFER_LENGTH_ACCEL, &bufferIndexAccel, data_z);

		// pass the buffer to the activity detection function
		// if the activity is detected, send a message to the user
		uint8_t activity;

		processData(accelerationBuffer, BUFFER_LENGTH_ACCEL, &classificationBuffer, &bufferIndexClassification, BUFFER_LENGTH_CLASSIFICATION);

		if (activity == WALK)
		{
			warpPrint("Walking detected!\n");
		}
		else if (activity == RUN)
		{
			warpPrint("Running detected!\n");
		}
		else if (activity == REST)
		{
			warpPrint("Resting detected!\n");
		}
		else if (activity == ERR)
		{
			warpPrint("Error detected!\n");
		}

		// once the buffer is full compute the activity for the last 10 seconds
		}
	return 0;
}


WarpStatus
writeByteToI2cDeviceRegister(uint8_t i2cAddress, bool sendCommandByte, uint8_t commandByte, bool sendPayloadByte, uint8_t payloadByte)
{
	i2c_status_t	status;
	uint8_t		commandBuffer[1];
	uint8_t		payloadBuffer[1];
	i2c_device_t	i2cSlaveConfig =
			{
				.address = i2cAddress,
				.baudRate_kbps = gWarpI2cBaudRateKbps
			};

	commandBuffer[0] = commandByte;
	payloadBuffer[0] = payloadByte;

	status = I2C_DRV_MasterSendDataBlocking(
						0	/* instance */,
						&i2cSlaveConfig,
						commandBuffer,
						(sendCommandByte ? 1 : 0),
						payloadBuffer,
						(sendPayloadByte ? 1 : 0),
		gWarpI2cTimeoutMilliseconds);

	return (status == kStatus_I2C_Success ? kWarpStatusOK : kWarpStatusDeviceCommunicationFailed);
}
