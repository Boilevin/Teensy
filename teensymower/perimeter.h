#ifndef PERIMETER_H
#define PERIMETER_H

#include <Arduino.h>

#define RAW_SIGNAL_SAMPLE_SIZE 32

#define IDX_LEFT   0
#define IDX_CENTER 0
#define IDX_RIGHT  1


class PerimeterClass
{
  public:
    PerimeterClass();
    // set ADC pins
    void begin(byte idx0Pin, byte idx1Pin);

    void printTimedADCInfo(uint8_t adc_num, uint16_t *buffer, uint32_t &delta_time);



    void changeArea(byte areaInMowing);
    const int16_t* getRawSignalSample(byte idx);
    // get perimeter magnitude
    int getMagnitude(byte idx);
    int getSmoothMagnitude(byte idx);
    // inside perimeter (true) or outside (false)?
    boolean isInside();
    boolean isInside(byte idx);
    void resetTimedOut();
    boolean signalTimedOut(byte idx);
    int16_t getSignalMin(byte idx);
    int16_t getSignalMax(byte idx);
    int getSignalAvg(byte idx);
    float getFilterQuality(byte idx);
    void speedTest();
    byte signalCodeNo; // not used yet
    void run();
    int timedOutIfBelowSmag;
    unsigned long timeOutSecIfNotInside;
    
   
   
    unsigned long lastInsideTime[2];
   
 
  private:


    static void adc0_isr();
    static void adc1_isr();



    byte idxPin[2]; // channel for idx
    //int callCounter;
    int16_t mag [2]; // perimeter magnitude per channel
    float smoothMag[2];
    float filterQuality[2];
    int16_t signalMin[2];
    int16_t signalMax[2];
    int signalAvg[2];
    int signalCounter[2];
    //int8_t rawSignalSample[2][RAW_SIGNAL_SAMPLE_SIZE];
    void matchedFilter(byte idx);
    int16_t corrFilter(int8_t *H, int subsample, int M, int16_t *ip, int16_t nPts, float &quality);
    void printADCMinMax(int16_t *samples);
};

extern PerimeterClass Perimeter;

#endif
