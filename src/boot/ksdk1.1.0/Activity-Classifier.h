// Header file for Activity-Classifier.c

#ifndef _ACTIVITY_CLASSIFIER_H_
#define _ACTIVITY_CLASSIFIER_H_


// Function prototypes
void ActivityClassifier_Init(void);
void ActivityClassifier_Run(void);
void ActivityClassifier_Deinit(void);
uint16_t lowPassFilter(uint16_t data);
uint16_t highPassFilter(uint16_t data);
uint8_t classifyActivity(uint16_t accelerationData[], uint16_t size, uint16_t filterLength, uint16_t windowLength, uint16_t threshold);
uint8_t activityClassifier(uint16_t accelerationData[], uint8_t size, uint8_t samplingRate);


// activity flags i.e. walk, run, stand, liedown

#define WALK  0
#define RUN 1
#define REST 2
#define ERR 0b11111111
#define WALKRUN_THRESHOLD 120

#endif // _ACTIVITY_CLASSIFIER_H_

// end of file