#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "arduinoFFT.h"
#include <math.h>

#define Bins 8

#define BNO055_SAMPLERATE_DELAY_MS (100)
#define ledPin 13
#define LeftMotor 6
#define RightMotor 5

#define SAMPLES 256
#define SAMPLING_FREQUENCY 25600
#define UltraPin 10


/**************************************************************************/
                        /*Global Variables*/

Adafruit_BNO055 bno = Adafruit_BNO055(55);    // imu object
int angleOffset;  // for initial angle
int desiredAngle; 
long duration, cm;

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const uint16_t samples = 256; // Blocksixe for fft input 
const double samplingFrequency = 25600;
unsigned int sampling_period_us;
unsigned long microseconds; 
double vReal[SAMPLES];
double vImag[SAMPLES];
int freqIdx = 0;    // 0 for 5k, 1 for 5.5K, 2 for 6K ...

int greatest = 0;
// declare vectors for fft magnitude and angle storage
double magnitudeArray[13];
double anglesArray[13]; 
int freqArray[10] = {5000,5500,6000,6500,7000,7500,8000,8500,9000,9500}; 
bool visitedArray[10] = {false,false,false,false,false,false,false,false,false,false};    // 0 for 5k, 1 for 5.5K, 2 for 6K ...
int desiredFreq;

int temp; 
int tempIndex; 
bool angleFound = false;

/**************************************************************************/


/**************************************************************************/
                       /*Function Declarations*/
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);

void initIMU();
void rotateTo(int angle);
int readAngle();

void initMotors();
void forward(int Speed);
void turnLeft();
void turnRight();
void stopRobot();

void doFFT(int angle, int freqIndex);

double MajorPeakModified(double *vD, uint16_t samples, double samplingFrequency, double *Magnitude);
/**************************************************************************/

void setup(void)
{
  Serial.begin(115200);
  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  pinMode(A1,INPUT);
  pinMode(ledPin,OUTPUT);
  pinMode(UltraPin, OUTPUT);
  initMotors();
  initIMU();
}

void loop(void)
{
  int freq_ang, startTime, endTime;
 
  for(int i=0, angleFound = false;i<=360;i=i+20)
  {  
    rotateTo(i);
   // doFFT(i,freqIdx); //delay(2000);    // replace delay to do fft per positions
    desiredFreq = freqArray[freqIdx];
    freq_ang = doFFT_Freq(freqArray[freqIdx]);
    if (freq_ang != -1){
      stopRobot();
      angleFound = true;
      break;
    }
    Serial.print("Frequency angle: ");
    Serial.println(freq_ang);
    angleFound = false; 
  }

  desiredAngle = freq_ang;
  Serial.print("Desired Angle: ");
  Serial.print(desiredAngle);

  Serial.print(" Distance: ");
  Serial.print(getDistance());

  Serial.print(" get_Freq(desiredFreq): ");
  Serial.print(get_Freq(desiredFreq));

  Serial.print(" Desired freq: ");
  Serial.println(desiredFreq);
  
  delay(1000);

  if(angleFound)
  {
    /*Go in desrired angle direction*/
    while(getDistance()>15)// && desiredFreq == get_Freq(desiredFreq))
    {
      Serial.println("While  - Dist > 15");
      if(readAngle() > desiredAngle+5)  // turn left to compensate10.
      {
        turnLeft(); 
        delay(100);
        stopRobot();  
      }
      else if(readAngle() < desiredAngle-5)
      {
        turnRight(); 
        delay(100);
        stopRobot();  
      }
      else
      {
         forward(50); 
         delay(1000);
         stopRobot();
     }      
    }
    if(getDistance()<=15)
    {
      stopRobot();
      delay(1000);
      if(desiredFreq == get_Freq(desiredFreq))
        freqIdx++;  // go to next freq

      if(desiredFreq == 9500)
      {
        digitalWrite(ledPin,HIGH);
        forward(50);
        delay(1000);
        while(1)   // end of task
        {
          stopRobot();
        }
      }

//      while(getDistance()<50)
//      {
//        rotateTo(desiredAngle+30);
//      }
    }
    
  }
  else
  {
    /*do fft again*/
    forward(50); 
    delay(1000);
    stopRobot();
  }
}


/* ---------- IMU Functions ---------- */
void initIMU()
{
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(false); 

  delay(1000);

  sensors_event_t event;
  bno.getEvent(&event);
  
  angleOffset = event.orientation.x;

  Serial.print("\n Angle Offset: ");
  Serial.println(angleOffset);
}

int readAngle()
{
  sensors_event_t event;
  bno.getEvent(&event);
  int expression, ang;
  /* Wait the specified delay before requesting next data */
    delay(BNO055_SAMPLERATE_DELAY_MS);

  expression = (int)event.orientation.x - angleOffset;
  if(expression >= 0)
    return expression;
  else
    return 360+expression;
}

void rotateTo(int angle)
{
  int currangle = readAngle();
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.println(currangle);

  while(currangle!=angle)
  {
    if(currangle < angle+5 && currangle > angle-5)
      break;    // desired angle achieved

   /* Rotatecode here*/
    turnRight();
    delay(150);
    stopRobot();
   // delay(10);
   /* -------------- */
    
   // Take next sample
    currangle = readAngle();
    Serial.print("X: ");
    Serial.println(currangle);
  }
   stopRobot(); //to stop the bot at the desired angle
  /*** Flash LED to indicate desired angle is reached****/
  for(int i=0; i<5;i++)
  {
    digitalWrite(ledPin,HIGH);
    delay(50);
    digitalWrite(ledPin,LOW);
    delay(50); 
  }
}

/* ---------- IMU Functions End ---------- */

/* ---------- Motor Functions ---------- */
void initMotors()
{
  Serial.println("Initializing Motors...");
  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);
  
  int Frequency = 50; // time period for pwm is 20 ms
  analogWriteFrequency(RightMotor, Frequency);
  analogWriteFrequency(LeftMotor, Frequency);
  Serial.println("Initialization Done!");
  delay(1000);
}
void forward(int Speed) {
 int x = map(Speed, 1, 100, 21, 25);
 int y = map(Speed, 1, 100, 17, 13);
 analogWrite(RightMotor, x);
 analogWrite(LeftMotor, y);
}

void turnLeft() {
// analogWrite(RightMotor, 21);
// analogWrite(LeftMotor, 21);
  analogWrite(RightMotor, 22);
  analogWrite(LeftMotor, 22);
}

void turnRight() {
// analogWrite(RightMotor, 17);
// analogWrite(LeftMotor, 17);
  analogWrite(RightMotor, 16);
  analogWrite(LeftMotor, 16);
}

void stopRobot() {
 analogWrite(RightMotor, 0);
 analogWrite(LeftMotor, 0);
}
/* ---------- Motor Functions End ---------- */

/* ---------- ADC and FFT Functions ---------- */

int doFFT_Freq(int freq)
{
 // FFT_RESULT fft_res;
  double FFT_SamplingArray[20]; 

/// Initialize Sampling Array
  for(int l=0;l<20;l++)
    FFT_SamplingArray[l]=0; 

  // sample fft for 20 times 
  for ( int NumTimes = 0; NumTimes < 20; NumTimes++)
  {
    for ( uint16_t i = 0; i < samples; i++)
    {
      microseconds = micros();
      vReal[i] = analogRead(A1);    // output of the microphone is at ADC0_RA
      vImag[i] = 0;
      while(micros()<(microseconds + sampling_period_us)){}
      //delayMicroseconds(15);
     } // end for
      
    // perform fft
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    vReal[0] = 0;
    vReal[1] = 0;
    vReal[2] = 0;

    
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY); 

    Serial.print("peak = ");

    Serial.println(peak);
    delay(75);
   
    if (peak >= freq-50 && peak <= freq+50)
    {
        peak = freq;
        
        return readAngle();
//      FFT_SamplingArray[NumTimes] = Magnitude;
    }
    

   /////////////////////// END FFT ///////////////////////////////////
  
 //  delay(75);  // so mic can stabilize
  
  } // end big for loop for FFT
  return -1;
}

int get_Freq(int freq)
{
 // FFT_RESULT fft_res;
  double FFT_SamplingArray[20]; 

/// Initialize Sampling Array
  for(int l=0;l<20;l++)
    FFT_SamplingArray[l]=0; 

  // sample fft for 20 times 
  for ( int NumTimes = 0; NumTimes < 20; NumTimes++)
  {
    for ( uint16_t i = 0; i < samples; i++)
    {
      microseconds = micros();
      vReal[i] = analogRead(A1);    // output of the microphone is at ADC0_RA
      vImag[i] = 0;
      while(micros()<(microseconds + sampling_period_us)){}
      //delayMicroseconds(15);
     } // end for
      
    // perform fft
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    vReal[0] = 0;
    vReal[1] = 0;
    vReal[2] = 0;

    
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY); 

    Serial.print("peak = ");

    Serial.println(peak);
    delay(75);
   
    if (peak >= freq-50 && peak <= freq+50)
    {
        peak = freq;
        
        return peak;
//      FFT_SamplingArray[NumTimes] = Magnitude;
    }
    

   /////////////////////// END FFT ///////////////////////////////////
  
 //  delay(75);  // so mic can stabilize
  
  } // end big for loop for FFT
  return -1;
}

double MajorPeakModified(double *vD, uint16_t samples, double samplingFrequency, double *Magnitude)
{
  double maxY = 0;
  uint16_t IndexOfMaxY = 0;
  //If sampling_frequency = 2 * max_frequency in signal,
  //value would be stored at position samples/2
  for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
    if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1])) {
      if (vD[i] > maxY) {
        maxY = vD[i];
        IndexOfMaxY = i;
      }
    }
  }
  *Magnitude = vD[IndexOfMaxY];
  double delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
  double interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples - 1);
  if (IndexOfMaxY == (samples >> 1)) //To improve calculation on edge values
    interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
  // returned value: interpolated frequency peak apex
  return (interpolatedX);
}

/* ---------- ADC and FFT Functions Ends ---------- */

/* ----------- Ultrasonic Functions -----------*/
int getDistance()
{  
  delay(100);
  pinMode(UltraPin, OUTPUT);
  digitalWrite(UltraPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltraPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(UltraPin, LOW);

  
  pinMode(UltraPin, INPUT);
  duration = pulseIn(UltraPin, HIGH);

  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
/* ----------- Ultrasonic Functions Ends ----------- */


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
