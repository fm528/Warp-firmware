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

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "Activity-Classifier.h"


// Function to implement a memory-efficient median filter
uint16_t medianFilter(uint16_t data[], uint16_t size)
{
    // Sort the data array in ascending order
    for (uint16_t i = 0; i < size - 1; i++)
    {
        for (uint16_t j = 0; j < size - i - 1; j++)
        {
            if (data[j] > data[j + 1])
            {
                uint16_t temp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = temp;
            }
        }
    }

    // Calculate the median value
    uint16_t median;
    if (size % 2 == 0)
    {
        median = (data[size / 2 - 1] + data[size / 2]) / 2;
    }
    else
    {
        median = data[size / 2];
    }

    return median;
}

// Function to process high pass filtered data and perform step counting
uint8_t processData(uint16_t data[][3], uint16_t size)
{
    // Apply median filter to the data
    uint16_t filteredData[size][3];
    for (uint16_t i = 0; i < size; i++)
    {
        for (uint16_t j = 0; j < 13; j++)
        {
            filteredData[i][j] = medianFilter(data[i], 13);
        }
    }

    // Calculate the product of magnitude and duration
    uint32_t product = 0;
    for (uint16_t i = 0; i < size; i++)
    {
        for (uint16_t j = 0; j < 13; j++)
        {
            product += filteredData[i][j];
        }
    }

    // Define the threshold for step counting
    uint16_t stepCountThreshold = 2;

    // Check if the product is larger than the threshold
    if (product > threshold)
    {
        // Mark maximal, minimal, and mid points for a window of length 25
        uint16_t maxPoints = 0;
        uint16_t minPoints = 0;
        uint16_t midPoints = 0;
        for (uint16_t i = 0; i < size; i++)
        {
            for (uint16_t j = 0; j < 25; j++)
            {
                if (filteredData[i][j] > filteredData[i][j + 1] && filteredData[i][j] > filteredData[i][j - 1])
                {
                    maxPoints++;
                }
                else if (filteredData[i][j] < filteredData[i][j + 1] && filteredData[i][j] < filteredData[i][j - 1])
                {
                    minPoints++;
                }
                else
                {
                    midPoints++;
                }
            }
        }

        // Count the number of changes from max to min
        uint16_t stepCount = 0;
        for (uint16_t i = 0; i < size; i++)
        {
            for (uint16_t j = 0; j < 25; j++)
            {
                if (filteredData[i][j] > filteredData[i][j + 1] && filteredData[i][j] > filteredData[i][j - 1] &&
                    filteredData[i][j + 1] < filteredData[i][j + 2] && filteredData[i][j - 1] < filteredData[i][j - 2])
                {
                    stepCount++;
                }
            }
        }

        // Check if the step count is above the threshold
        if (stepCount > stepCountThreshold)
        {
            // Return the step count
            return "STEP";
        }
        else
        {
            // Return REST
            return "REST";
        }
    }
    else
    {
        // Return REST
        return REST;
    }
}