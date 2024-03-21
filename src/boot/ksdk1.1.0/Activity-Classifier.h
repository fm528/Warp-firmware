// Header file for Activity-Classifier.c

#ifndef _ACTIVITY_CLASSIFIER_H_
#define _ACTIVITY_CLASSIFIER_H_


// Function prototypes
uint16_t medianFilter(uint16_t data[], uint16_t size);
// Function to process high pass filtered data and perform step counting
uint8_t processData(uint16_t data[], uint16_t size);


// activity flags i.e. walk, run, stand, liedown

#define WALK  0
#define RUN 1
#define REST 2
#define ERR 0b11111111
#define WALKRUN_THRESHOLD 120

#endif // _ACTIVITY_CLASSIFIER_H_

// end of file