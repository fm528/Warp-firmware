// This is a module that coontains the functions required to classify the activity of the user
// using the accelerometer data. The module first high passes the accelerometer data to remove the
// gravity component and then low passes to remove noise.  The module then uses the filtered data
// to classify the activity of the user. The module uses a simple threshold based classifier to
// classify the activity of the user. This calculates the magnitude-duration area of the filtered
// data and compares it with the threshold values to classify the activity of the user. The module
// also contains the functions to set the threshold values for the classifier.

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
    // Create a copy of the data array
    uint16_t sortedData[size];
    memcpy(sortedData, data, size * sizeof(uint16_t));

    // Sort the data array in ascending order
    for (uint16_t i = 0; i < size - 1; i++)
    {
        for (uint16_t j = 0; j < size - i - 1; j++)
        {
            if (sortedData[j] > sortedData[j + 1])
            {
                uint16_t temp = sortedData[j];
                sortedData[j] = sortedData[j + 1];
                sortedData[j + 1] = temp;
            }
        }
    }

    // Calculate the median value
    uint16_t median;
    if (size % 2 == 0)
    {
        median = (sortedData[size / 2 - 1] + sortedData[size / 2]) / 2;
    }
    else
    {
        median = sortedData[size / 2];
    }

    return median;
}

uint8_t classifyActivity(uint16_t accelerationData[], uint16_t size, uint16_t filterLength, uint16_t windowLength, uint16_t threshold)
{
    // Apply median filter to the acceleration data
    uint16_t filteredData[size];
    for (uint16_t i = 0; i < size; i++)
    {
        // Create a window of filterLength size
        uint16_t window[filterLength];
        for (uint16_t j = 0; j < filterLength; j++)
        {
            // Check if the index is within the bounds of the accelerationData array
            if (i + j < size)
            {
                window[j] = accelerationData[i + j];
            }
        }

        // Apply median filter to the window
        filteredData[i] = medianFilter(window, filterLength);
    }

    // Calculate the product of amplitude and time for consecutive windows
    for (uint16_t i = 0; i < size - windowLength + 1; i+windowLength)
    {
        uint16_t product = 0;
        for (uint16_t j = 0; j < windowLength; j++)
        {
            product += filteredData[i + j];
        }

        // Compare the product with the threshold
        if (product > threshold)
        {
            // Window is classified as active
            return activityClassifier(filteredData, size, 50);
        }
        else
        {
            // Window is classified as inactive
            return REST;
        }
    }
}

#include <complex.h>
#include <math.h>

void fft(uint16_t data[], uint16_t size, uint16_t fftData[])
{
    // Calculate the number of stages
    uint16_t numStages = log2(size);

    // Perform the FFT algorithm
    for (uint16_t stage = 0; stage < numStages; stage++)
    {
        // Calculate the number of butterflies in this stage
        uint16_t numButterflies = pow(2, stage);

        // Calculate the distance between butterflies in this stage
        uint16_t butterflyDistance = size / (2 * numButterflies);

        // Perform the butterflies in this stage
        for (uint16_t butterfly = 0; butterfly < numButterflies; butterfly++)
        {
            // Calculate the indices of the butterflies
            uint16_t index1 = butterfly * butterflyDistance;
            uint16_t index2 = index1 + butterflyDistance;

            // Calculate the twiddle factor
            int16_t twiddleFactorReal = cos(-2 * M_PI * butterfly / (2 * numButterflies)) * (1 << 14);
            int16_t twiddleFactorImag = sin(-2 * M_PI * butterfly / (2 * numButterflies)) * (1 << 14);

            // Perform the butterfly operation
            int16_t butterfly1Real = data[index1].real;
            int16_t butterfly1Imag = data[index1].imag;
            int16_t butterfly2Real = (twiddleFactorReal * data[index2].real - twiddleFactorImag * data[index2].imag) >> 14;
            int16_t butterfly2Imag = (twiddleFactorReal * data[index2].imag + twiddleFactorImag * data[index2].real) >> 14;

            // Update the data array with the butterfly results
            data[index1].real = butterfly1Real + butterfly2Real;
            data[index1].imag = butterfly1Imag + butterfly2Imag;
            data[index2].real = butterfly1Real - butterfly2Real;
            data[index2].imag = butterfly1Imag - butterfly2Imag;
        }
    }

    // Calculate the magnitude of the FFT data
    for (uint16_t i = 0; i < size / 2 + 1; i++)
    {
        fftData[i] = sqrt((data[i].real * data[i].real + data[i].imag * data[i].imag) >> 14);
    }
}

uint8_t activityClassifier(uint16_t accelerationData[], uint8_t size, uint8_t samplingRate) {
    // Using the z axis data compute the FFT and find the largest frequency spike between 1 and 3 Hz

    // Compute the FFT of the acceleration data on the z-axis

    uint16_t fftSize = size / 2 + 1;
    float fftData[fftSize];
    fft(accelerationData[2], size, fftData);

    // Find the largest frequency spike between 1 and 3 Hz
    float maxAmplitude = 0;
    uint16_t maxIndex = 0;
    for (uint16_t i = 1; i < fftSize; i++)
    {
        float frequency = i * (samplingRate / size);
        if (frequency >= 1 && frequency <= 3 && fftData[i] > maxAmplitude)
        {
            maxAmplitude = fftData[i];
            maxIndex = i;
        }
    }

    // Convert the index to frequency
    float maxFrequency = maxIndex * (samplingRate / size);

    // Calculate the step rate per minute
    float stepRate = maxFrequency * 60;

    // Check if the step rate exceeds the walking threshold
    if (stepRate <= WALKRUN_THRESHOLD)
    {
        return WALK;
    }
    // Check if the step rate exceeds the running threshold
    else if (stepRate <= WALKRUN_THRESHOLD)
    {
        return RUN;
    }
    else
    {
        return ERR;
    }
}
