#include "perimeter.h"
#include <Arduino.h>
#include <limits.h>
//#include "adcman.h"
//#include "config.h"

#include <ADC.h>
#include <ADC_util.h>


PerimeterClass Perimeter;

//adc var
ADC *adc = new ADC(); // adc object;
#define USE_ADC_0
#define USE_ADC_1
#define BUFFER_SIZE 192  //sigcode 24 * 4 subsample *1.5 to scan on a full signal code
int16_t buffer_ADC_0[BUFFER_SIZE];
int16_t buffer_adc_0_count = 0xffff;
uint32_t delta_time_adc_0 = 0;

int16_t buffer_ADC_1[BUFFER_SIZE];
int16_t buffer_adc_1_count = 0xffff;
uint32_t delta_time_adc_1 = 0;
elapsedMillis timed_read_elapsed;
//end adc var

int subSample = 4;
int16_t *samples;
int sampleCount = 192;//ADCMan.getSampleCount(idxPin[0]);

int8_t sigcode_norm[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };
int8_t sigcode_diff[] = { 1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1, 0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1 };
int sigcode_size = 24;

PerimeterClass::PerimeterClass() {
  //useDifferentialPerimeterSignal = true;
  swapCoilPolarityLeft = false;
  swapCoilPolarityRight = false;
  //read2Coil=true;
  timedOutIfBelowSmag = 50;
  timeOutSecIfNotInside = 15;
  callCounter = 0;
  mag[0] = mag[1] = 0;
  smoothMag[0] = smoothMag[1] = 0;
  filterQuality[0] = filterQuality[1] = 0;
  signalCounter[0] = signalCounter[1] = 0;
  lastInsideTime[0] = lastInsideTime[1] = 0;
}


void PerimeterClass::changeArea(byte areaInMowing) {

}

static void PerimeterClass::adc1_isr() {//this is the main adc1 loop executed each 24 microseconds
  uint16_t adc_val = adc->adc1->readSingle();
  if (buffer_adc_1_count < BUFFER_SIZE) {
    buffer_ADC_1[buffer_adc_1_count++] = adc_val;
    if (buffer_adc_1_count == BUFFER_SIZE) {
      delta_time_adc_1 = timed_read_elapsed;
      Perimeter.matchedFilter(1);
      buffer_adc_1_count = 0;
    }
  }
#if defined(__IMXRT1062__)  // Teensy 4.0
  asm("DSB");
#endif
}

static void PerimeterClass::adc0_isr() { //this is the main adc0 loop executed each 24 microseconds
  //Serial.println(buffer_adc_0_count);
  uint16_t adc_val = adc->adc0->readSingle();
  if (buffer_adc_0_count < BUFFER_SIZE) {
    buffer_ADC_0[buffer_adc_0_count++] = adc_val;
    if (buffer_adc_0_count == BUFFER_SIZE) {
      /*
            for (int i = 0; i < BUFFER_SIZE; i++) {

              Serial.print(" ");
              Serial.println(buffer_ADC_0[i]);
            }
*/
      
      delta_time_adc_0 = timed_read_elapsed;
      Perimeter.matchedFilter(0);
      //buffer_adc_0_count = 0;
      buffer_adc_0_count = 0;

    }
  }
#if defined(__IMXRT1062__)  // Teensy 4.0
  asm("DSB");
#endif
}


void PerimeterClass::begin(byte idx0Pin, byte idx1Pin) {

  idxPin[0] = idx0Pin;
  idxPin[1] = idx1Pin;


  pinMode(idx0Pin, INPUT);
  pinMode(idx1Pin, INPUT);

  Serial.println("Begin setup adc0");
  ///// ADC0 ////
  adc->adc0->setAveraging(1); // set number of averages
  adc->adc0->setResolution(10); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); // change the sampling speed
  //adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  //adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  Serial.println("End setup adc0");
  Serial.println("Try to Start adc0 Timer Interrupt ");
  adc->adc0->stopTimer();
  adc->adc0->startSingleRead(idx0Pin); // call this to setup everything before the Timer starts
  adc->adc0->enableInterrupts(adc0_isr);
  //adc->adc0->startTimer(384); //frequency Hz

  adc->adc0->startTimer(38462); //frequency in Hz
  buffer_adc_0_count = 0;
  Serial.println("adc0 Timer Interrupt Started");



  Serial.println("Begin setup adc1");
  ////// ADC1 /////
  adc->adc1->setAveraging(1); // set number of averages
  adc->adc1->setResolution(10); // set bits of resolution
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed
  Serial.println("End setup adc1");
  Serial.println("Try to Start adc1 Timer Interrupt ");
  adc->adc1->stopTimer();
  adc->adc1->startSingleRead(idx1Pin); // call this to setup everything before the Timer starts
  adc->adc1->enableInterrupts(adc1_isr);
  adc->adc1->startTimer(38462); //frequency in Hz
  timed_read_elapsed = 0;
  buffer_adc_1_count = 0;
  Serial.println("adc1 Timer Interrupt Started");

}



int PerimeterClass::getMagnitude(byte idx) {
  return Perimeter.mag[idx];
}

int PerimeterClass::getSmoothMagnitude(byte idx) {
  return Perimeter.smoothMag[idx];
}





// perimeter V2 uses a digital matched filter
void PerimeterClass::matchedFilter(byte idx) {
  if (idx == 0) {
    samples = buffer_ADC_0 ;//ADCMan.getSamples(idxPin[idx]);      
  }
  else {
    samples = buffer_ADC_1 ;//ADCMan.getSamples(idxPin[idx]);
  }
  
 // Serial.println(buffer_adc_0_count);
  signalMin[idx] = 9999;
  signalMax[idx] = -9999;
  signalAvg[idx] = 0;
  for (int i = 0; i < sampleCount; i++) {
    int16_t v = samples[i];
    // Serial.print(v);
    // Serial.print(",");
    signalAvg[idx] += v;
    signalMin[idx] = min(signalMin[idx], v);
    signalMax[idx] = max(signalMax[idx], v);
  }
  //Serial.println(" ");
   
  signalAvg[idx] = signalAvg[idx]/sampleCount;

  // magnitude for tracking (fast but inaccurate)

  //int16_t sigcode_size = sizeof sigcode_norm;

  int8_t *sigcode = sigcode_norm;
  //if (useDifferentialPerimeterSignal) sigcode = sigcode_diff;
  sigcode = sigcode_diff;
  //sampleCount - sigcode_size * subSample= normalement 96  192-24*4






  mag[idx] = corrFilter(sigcode, subSample, sigcode_size, samples, sampleCount - sigcode_size * subSample , filterQuality[idx]);





  //Serial.println(mag[1]);
  // if ((idx == 0) && swapCoilPolarityLeft) mag[idx] *= -1;
  // if ((idx == 1) && swapCoilPolarityRight) mag[idx] *= -1;
  // smoothed magnitude used for signal-off detection change from 1 % to 5 % for faster detection and possible use on center big area to avoid in/out transition
  smoothMag[idx] = 0.95 * smoothMag[idx] + 0.05 * ((float)abs(mag[idx]));
  //smoothMag[idx] = 0.99 * smoothMag[idx] + 0.01 * ((float)abs(mag[idx]));

  // perimeter inside/outside detection
  if (mag[idx] > 0) {
    signalCounter[idx] = min(signalCounter[idx] + 1, 3);
  } else {
    signalCounter[idx] = max(signalCounter[idx] - 1, -3);
  }
  if (mag[idx] < 0) {
    lastInsideTime[idx] = millis();
  }

  //ADCMan.restartConv(idxPin[idx]);
  if (idx == 0) callCounter++;
}

void PerimeterClass::resetTimedOut() {
  lastInsideTime[0] = millis();
  lastInsideTime[1] = millis();
}

int16_t PerimeterClass::getSignalMin(byte idx) {
  return Perimeter.signalMin[idx];
}

int16_t PerimeterClass::getSignalMax(byte idx) {
  return Perimeter.signalMax[idx];
}

int PerimeterClass::getSignalAvg(byte idx) {
  return Perimeter.signalAvg[idx];
}


float PerimeterClass::getFilterQuality(byte idx) {
  return Perimeter.filterQuality[idx];
}

boolean PerimeterClass::isInside() {

  return (Perimeter.isInside(IDX_LEFT));
  //return (isInside(IDX_LEFT) && isInside(IDX_RIGHT));
}

boolean PerimeterClass::isInside(byte idx) {
  if (abs(mag[idx]) > 600) {
    // Large signal, the in/out detection is reliable.
    // Using mag yields very fast in/out transition reporting.
    return (Perimeter.mag[idx] < 0);
  } else {
    // Low signal, use filtered value for increased reliability
    return (Perimeter.signalCounter[idx] < 0);
  }
}

boolean PerimeterClass::signalTimedOut() {

  return (signalTimedOut(IDX_LEFT) && signalTimedOut(IDX_RIGHT));
}


boolean PerimeterClass::signalTimedOut(byte idx) {
  if (getSmoothMagnitude(idx) < timedOutIfBelowSmag) return true;
  if (millis() - lastInsideTime[idx] > timeOutSecIfNotInside * 1000) return true;
  return false;
}


// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data

int16_t PerimeterClass::corrFilter(int8_t *H, int subsample, int M, int16_t *ip, int16_t nPts, float &quality) {

  //erase array
  //memset(myarray, 0, sizeof(myarray));
  /*
    Serial.println();
    Serial.print("H:");
    Serial.print(*H);
    Serial.print(" Subsample:");
    Serial.print(subsample);
    Serial.print(" M:");
    Serial.print(M);
    Serial.print(" *ip:");
    Serial.print(*ip);
    Serial.print(" nPts:");
    Serial.print(nPts);
    Serial.print(" Quality:");
    Serial.print(quality);
    Serial.println();
  */
  /*
    for (byte i = 0; i <= 23; i++) {
      Serial.print(H[i]);
      Serial.print(",");
    }
    Serial.println();

    for (byte i = 0; i <= 191; i++) {
      Serial.print(ip[i]);
      Serial.print(",");
    }
    Serial.println();
  */

  int16_t sumMax = 0; // max correlation sum
  int16_t sumMin = 0; // min correlation sum
  int16_t Ms = M * subsample; // number of filter coeffs including subsampling

  // compute sum of absolute filter coeffs
  int16_t Hsum = 0;
  for (int16_t i = 0; i < M; i++) Hsum += abs(H[i]);
  Hsum *= subsample;

  // compute correlation
  // for each input value
  for (int16_t j = 0; j < nPts; j++)
  {
    int16_t sum = 0;
    int8_t *Hi = H;
    int8_t ss = 0;
    int16_t *ipi = ip;
    // for each filter coeffs
    for (int16_t i = 0; i < Ms; i++)
    {
      sum += ((int16_t)(*Hi)) * ((int16_t)(*ipi));
      ss++;
      if (ss == subsample) {
        ss = 0;
        Hi++; // next filter coeffs
      }
      ipi++;
    }
    if (sum > sumMax) sumMax = sum;
    if (sum < sumMin) sumMin = sum;
    ip++;
  }
  // normalize to 4095
  sumMin = ((float)sumMin) / ((float)(Hsum * 127)) * 4095.0;
  sumMax = ((float)sumMax) / ((float)(Hsum * 127)) * 4095.0;

  // compute ratio min/max
  if (sumMax > -sumMin) {
    quality = ((float)sumMax) / ((float) - sumMin);
    return sumMax;
  } else {
    quality = ((float) - sumMin) / ((float)sumMax);
    return sumMin;
  }
}
