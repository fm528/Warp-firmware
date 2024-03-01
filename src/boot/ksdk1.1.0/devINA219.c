/*!
 * @file Adafruit_INA219.cpp
 *
 * @mainpage Adafruit INA219 current/power monitor IC
 *
 * @section intro_sec Introduction
 *
 *  Driver for the INA219 current sensor
 *
 *  This is a library for the Adafruit INA219 breakout
 *  ----> https://www.adafruit.com/product/904
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing
 *  products from Adafruit!
 *
 * @section author Author
 *
 * Written by Bryan Siepert and Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include <stdlib.h>

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

#include "errstrs.h"
#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devINA219.h"

extern volatile WarpI2CDeviceState deviceINA219State;
extern volatile uint32_t gWarpI2cBaudRateKbps;
extern volatile uint32_t gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t gWarpSupplySettlingDelayMilliseconds;

// /*!
//  *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
//  *          of current.  Each unit of current corresponds to 100uA, and
//  *          each unit of power corresponds to 2mW. Counter overflow
//  *          occurs at 3.2A.
//  *  @note   These calculations assume a 0.1 ohm resistor is present
//  */
// void Adafruit_INA219::setCalibration_32V_2A() {
//   // By default we use a pretty huge range for the input voltage,
//   // which probably isn't the most appropriate choice for system
//   // that don't use a lot of power.  But all of the calculations
//   // are shown below if you want to change the settings.  You will
//   // also need to change any relevant register settings, such as
//   // setting the VBUS_MAX to 16V instead of 32V, etc.

//   // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
//   // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08,
//   // 0.04) RSHUNT = 0.1               (Resistor value in ohms)

//   // 1. Determine max possible current
//   // MaxPossible_I = VSHUNT_MAX / RSHUNT
//   // MaxPossible_I = 3.2A

//   // 2. Determine max expected current
//   // MaxExpected_I = 2.0A

//   // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
//   // MinimumLSB = MaxExpected_I/32767
//   // MinimumLSB = 0.000061              (61uA per bit)
//   // MaximumLSB = MaxExpected_I/4096
//   // MaximumLSB = 0,000488              (488uA per bit)

//   // 4. Choose an LSB between the min and max values
//   //    (Preferrably a roundish number close to MinLSB)
//   // CurrentLSB = 0.0001 (100uA per bit)

//   // 5. Compute the calibration register
//   // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
//   // Cal = 4096 (0x1000)

//   ina219_calValue = 4096;

//   // 6. Calculate the power LSB
//   // PowerLSB = 20 * CurrentLSB
//   // PowerLSB = 0.002 (2mW per bit)

//   // 7. Compute the maximum current and shunt voltage values before overflow
//   //
//   // Max_Current = Current_LSB * 32767
//   // Max_Current = 3.2767A before overflow
//   //
//   // If Max_Current > Max_Possible_I then
//   //    Max_Current_Before_Overflow = MaxPossible_I
//   // Else
//   //    Max_Current_Before_Overflow = Max_Current
//   // End If
//   //
//   // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
//   // Max_ShuntVoltage = 0.32V
//   //
//   // If Max_ShuntVoltage >= VSHUNT_MAX
//   //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
//   // Else
//   //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
//   // End If

//   // 8. Compute the Maximum Power
//   // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
//   // MaximumPower = 3.2 * 32V
//   // MaximumPower = 102.4W

//   // Set multipliers to convert raw current/power values
//   ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
//   ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

//   // Set Calibration register to 'Cal' calculated above
//   Adafruit_BusIO_Register calibration_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
//   calibration_reg.write(ina219_calValue, 2);

//   // Set Config register to take into account the settings above
//   uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
//                     INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
//                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
//                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
//   Adafruit_BusIO_Register config_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
//   _success = config_reg.write(config, 2);
// }

// /*!
//  *  @brief  Configures to INA219 to be able to measure up to 32V and 1A
//  *          of current.  Each unit of current corresponds to 40uA, and each
//  *          unit of power corresponds to 800uW. Counter overflow occurs at
//  *          1.3A.
//  *  @note   These calculations assume a 0.1 ohm resistor is present
//  */
// void Adafruit_INA219::setCalibration_32V_1A() {
//   // By default we use a pretty huge range for the input voltage,
//   // which probably isn't the most appropriate choice for system
//   // that don't use a lot of power.  But all of the calculations
//   // are shown below if you want to change the settings.  You will
//   // also need to change any relevant register settings, such as
//   // setting the VBUS_MAX to 16V instead of 32V, etc.

//   // VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
//   // VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
//   // RSHUNT = 0.1			(Resistor value in ohms)

//   // 1. Determine max possible current
//   // MaxPossible_I = VSHUNT_MAX / RSHUNT
//   // MaxPossible_I = 3.2A

//   // 2. Determine max expected current
//   // MaxExpected_I = 1.0A

//   // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
//   // MinimumLSB = MaxExpected_I/32767
//   // MinimumLSB = 0.0000305             (30.5uA per bit)
//   // MaximumLSB = MaxExpected_I/4096
//   // MaximumLSB = 0.000244              (244uA per bit)

//   // 4. Choose an LSB between the min and max values
//   //    (Preferrably a roundish number close to MinLSB)
//   // CurrentLSB = 0.0000400 (40uA per bit)

//   // 5. Compute the calibration register
//   // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
//   // Cal = 10240 (0x2800)

//   ina219_calValue = 10240;

//   // 6. Calculate the power LSB
//   // PowerLSB = 20 * CurrentLSB
//   // PowerLSB = 0.0008 (800uW per bit)

//   // 7. Compute the maximum current and shunt voltage values before overflow
//   //
//   // Max_Current = Current_LSB * 32767
//   // Max_Current = 1.31068A before overflow
//   //
//   // If Max_Current > Max_Possible_I then
//   //    Max_Current_Before_Overflow = MaxPossible_I
//   // Else
//   //    Max_Current_Before_Overflow = Max_Current
//   // End If
//   //
//   // ... In this case, we're good though since Max_Current is less than
//   // MaxPossible_I
//   //
//   // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
//   // Max_ShuntVoltage = 0.131068V
//   //
//   // If Max_ShuntVoltage >= VSHUNT_MAX
//   //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
//   // Else
//   //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
//   // End If

//   // 8. Compute the Maximum Power
//   // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
//   // MaximumPower = 1.31068 * 32V
//   // MaximumPower = 41.94176W

//   // Set multipliers to convert raw current/power values
//   ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
//   ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

//   // Set Calibration register to 'Cal' calculated above
//   Adafruit_BusIO_Register calibration_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
//   calibration_reg.write(ina219_calValue, 2);

//   // Set Config register to take into account the settings above
//   uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
//                     INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
//                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
//                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
//   Adafruit_BusIO_Register config_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
//   _success = config_reg.write(config, 2);
// }

// /*!
//  *  @brief set device to alibration which uses the highest precision for
//  *     current measurement (0.1mA), at the expense of
//  *     only supporting 16V at 400mA max.
//  */
// void Adafruit_INA219::setCalibration_16V_400mA() {

//   // Calibration which uses the highest precision for
//   // current measurement (0.1mA), at the expense of
//   // only supporting 16V at 400mA max.

//   // VBUS_MAX = 16V
//   // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
//   // RSHUNT = 0.1               (Resistor value in ohms)

//   // 1. Determine max possible current
//   // MaxPossible_I = VSHUNT_MAX / RSHUNT
//   // MaxPossible_I = 0.4A

//   // 2. Determine max expected current
//   // MaxExpected_I = 0.4A

//   // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
//   // MinimumLSB = MaxExpected_I/32767
//   // MinimumLSB = 0.0000122              (12uA per bit)
//   // MaximumLSB = MaxExpected_I/4096
//   // MaximumLSB = 0.0000977              (98uA per bit)

//   // 4. Choose an LSB between the min and max values
//   //    (Preferrably a roundish number close to MinLSB)
//   // CurrentLSB = 0.00005 (50uA per bit)

//   // 5. Compute the calibration register
//   // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
//   // Cal = 8192 (0x2000)

//   ina219_calValue = 8192;

//   // 6. Calculate the power LSB
//   // PowerLSB = 20 * CurrentLSB
//   // PowerLSB = 0.001 (1mW per bit)

//   // 7. Compute the maximum current and shunt voltage values before overflow
//   //
//   // Max_Current = Current_LSB * 32767
//   // Max_Current = 1.63835A before overflow
//   //
//   // If Max_Current > Max_Possible_I then
//   //    Max_Current_Before_Overflow = MaxPossible_I
//   // Else
//   //    Max_Current_Before_Overflow = Max_Current
//   // End If
//   //
//   // Max_Current_Before_Overflow = MaxPossible_I
//   // Max_Current_Before_Overflow = 0.4
//   //
//   // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
//   // Max_ShuntVoltage = 0.04V
//   //
//   // If Max_ShuntVoltage >= VSHUNT_MAX
//   //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
//   // Else
//   //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
//   // End If
//   //
//   // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
//   // Max_ShuntVoltage_Before_Overflow = 0.04V

//   // 8. Compute the Maximum Power
//   // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
//   // MaximumPower = 0.4 * 16V
//   // MaximumPower = 6.4W

//   // Set multipliers to convert raw current/power values
//   ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
//   ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

//   // Set Calibration register to 'Cal' calculated above
//   Adafruit_BusIO_Register calibration_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
//   calibration_reg.write(ina219_calValue, 2);
//   // Set Config register to take into account the settings above
//   uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
//                     INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
//                     INA219_CONFIG_SADCRES_12BIT_1S_532US |
//                     INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

//   Adafruit_BusIO_Register config_reg =
//       Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
//   _success = config_reg.write(config, 2);
// }

// /*!
//  *  @brief  Provides the the underlying return value from the last operation
//  *          called on the device.
//  *  @return true: Last operation was successful false: Last operation failed
//  *  @note   For function calls that have intermediary device operations,
//  *          e.g. calibration before read/write, only the final operation's
//  *          result is stored.
//  */
// bool Adafruit_INA219::success() { return _success; }

/*!
 *  initialise the INA219
 */
void initINA219(const uint8_t i2cAddress, uint16_t operatingVoltattgeMillivolts)
{
    deviceINA219State.i2cAddress = i2cAddress;
    deviceINA219State.operatingVoltageMillivolts = operatingVoltattgeMillivolts;

    configureSensorINA219(
            INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS|
            INA219_CONFIG_SADCRES_12BIT_1S_532US|
            INA219_CONFIG_BADCRES_12BIT|
            INA219_CONFIG_GAIN_1_40MV|
            INA219_CONFIG_BVOLTAGERANGE_16V|
            INA219_CONFIG_RESET);

    

    return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
    uint8_t payloadByte[2], commandByte[1];
    i2c_status_t status;

    switch (deviceRegister)
    {
    case INA219_REG_CALIBRATION:
    case INA219_REG_CONFIG:
    {
        /* OK */
        break;
    }

    default:
    {
        return kWarpStatusBadDeviceCommand;
    }
    }

    i2c_device_t slave =
        {
            .address = deviceINA219State.i2cAddress,
            .baudRate_kbps = gWarpI2cBaudRateKbps};

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

    commandByte[0] = deviceRegister;
    payloadByte[0] = (payload >> 8) & 0xFF;
    payloadByte[1] = payload & 0xFF;
    warpEnableI2Cpins();
    status = I2C_DRV_MasterSendDataBlocking(
        0,
        &slave,
        commandByte,
        1,
        payloadByte,
        2,
        1000);
    
    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t cmdBuf[1] = {0xFF};
    i2c_status_t status1, status2;

    i2c_device_t slave =
        {
            .address = deviceINA219State.i2cAddress,
            .baudRate_kbps = gWarpI2cBaudRateKbps};

    USED(numberOfBytes);

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    warpEnableI2Cpins();
    cmdBuf[0] = deviceRegister;

    status1 = I2C_DRV_MasterSendDataBlocking(
        0,
        &slave,
        cmdBuf,
        1,
        NULL,
        0,
        gWarpI2cTimeoutMilliseconds);

    status2 = I2C_DRV_MasterReceiveDataBlocking(
        0,
        &slave,
        NULL,
        0,
        (uint8_t *)deviceINA219State.i2cBuffer,
        numberOfBytes,
        gWarpI2cTimeoutMilliseconds);

    if ((status1 != kStatus_I2C_Success || status2 != kStatus_I2C_Success))
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint16_t config)
{
    WarpStatus status1, status2;

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

    uint16_t payloadByte = config;
    status1 = writeSensorRegisterINA219(INA219_REG_CONFIG, payloadByte);
    status2 = writeSensorRegisterINA219(INA219_REG_CALIBRATION, 0xA000);
    return (status1 | status2);
}

void
printSensorDataINA219(bool hexModeFlag)
{
    uint16_t readSensorRegisterValueLSB;
    uint16_t readSensorRegisterValueMSB;
    int16_t readSensorRegisterValueCombined;
    WarpStatus i2cReadStatus;

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    i2cReadStatus = readSensorRegisterINA219(INA219_REG_CURRENT, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xff) << 8) | (readSensorRegisterValueLSB & 0xff);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint("----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("0x%02x 0x%02x, ", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint(" %d, ", readSensorRegisterValueCombined * 10);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(INA219_REG_BUSVOLTAGE, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xff) << 8) | (readSensorRegisterValueLSB & 0xff);
    
    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint("----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("0x%02x 0x%02x, ", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint(" %d, ", readSensorRegisterValueCombined);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(INA219_REG_POWER, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xff) << 8) | (readSensorRegisterValueLSB & 0xff);

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint("----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("0x%02x 0x%02x, ", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint(" %d, ", readSensorRegisterValueCombined);
        }
    }

    return;
}
