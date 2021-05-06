// READ BEFORE EDIT!!!!
// The dection mechanism is very sensitive to the
// sampling frequency, composition of the signal, settings of the threshold.

// for the SD card
#include "SdFat.h"
#include "RingBuf.h"

// For the digital accelerometer
#include "MPU9250.h"

// For the interrupts (from the MPU9250)
#include <avr/io.h>
#include <avr/interrupt.h>

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

// Interval between points for 1000 ksps.
#define LOG_INTERVAL_USEC 250

#define ANALOG_INTERVAL_USEC 100

// calculating the sampling frequency from the LOG_INTERVAL_USEC
// 0.000001 is converting interval to usec
#define SAMPLING_HZ 1/(0.000001*LOG_INTERVAL_USEC)

// time for logging in minutes
#define LOG_PERIOD 10
// Size to log 10 byte lines at 25 kHz for more than ten minutes.
#define LOG_FILE_SIZE 10*SAMPLING_HZ*60*LOG_PERIOD

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// This does not really make sence for us with 10 000 Hz fs. But makes
// the buffer twice as big so that is ... good I guess.
#define RING_BUF_CAPACITY 450*512
#define LOG_FILENAME "SdioLogger.csv"
#define LOG_FILENAME_NAME "test_"
#define LOG_FIMENAME_EXT ".csv"

//mpu9250 accelerometer interrupt pin
#define MPU_INT_PIN 9

FsFile root;
SdFs sd;
FsFile file;


// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

// SD card variables
size_t n = 0;
String fileName = String();
int fileCountOnSD = 0;

// Max RingBuf used bytes. Useful to understand RingBuf overrun.
size_t maxUsed = 0;

// Min spare micros in loop.
int32_t minSpareMicros = INT32_MAX;

// punch detection algo variables
int16_t ay1 = 0;
int16_t ay2 = 0;
float his1 = 0;
float his2 = 0;
float hisma1 = 1000000;
float hisma2 = 1000000;
float lambda = 0.99;
float lambda2 = 0.9995;
byte label1,label2 = 0;
uint32_t punchThr1,punchThr2 = 7000000;


uint32_t logTime = 0;
uint32_t analogLogTime = 0;
int count1 = 0;
int count2 = 0;
uint32_t cumVal1 = 0;
uint32_t cumVal2 = 0;
int punchCount = 0;

bool sigBelowThr1 = true;
bool sigBelowThr2 = true;

bool buzzState = false;
uint32_t buzzerTime = 0;
uint16_t tic = 0;
uint16_t toc = 0;
int16_t data[3];

//flags
bool punchFlag1 = false;
bool punchFlag2 = false;
bool buzzFlag1 = false;
bool buzzFlag2 = false;
bool breachFlag1 = false;
bool breachFlag2 = false;
bool gotNewDigitalFlag = false;
bool gotNewAnalogFlag = false;
bool digiDataReadyFlag = false;
bool analogDataReadyFlag = false;
bool stopButtonFlag = false;

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(SPI,10);
int status;

// interrupt variables
volatile bool digiTimeFlag = false;
volatile bool analogTimeFlag = false;

void mpuDataReadyInt(){
  digiTimeFlag = true;
}

// setting analog data ready flag
void analogTimeInt(){
  analogTimeFlag = true;
}


// pins
byte buzzer = 2;
byte startBtn = 3;
byte analogAccel1 = 23;
byte analogAccel2 = 22;
byte button2 = 3;

// interrupt timer for analog data setup
IntervalTimer analogTimer; 


void buttonPressed(){
  if(digitalRead(button2) == LOW){
    stopButtonFlag = true;
  }
}

void labelSetter(){
  if(punchFlag1 == true){
    // writting the digital data from MPU9250 to the SD card
    label1 = 1;
    punchCount = punchCount + 1;
    buzzFlag1 = true;
  } else {
    label1 = 0;
  }

  if(punchFlag2 == true){
    // writting the digital data from MPU9250 to the SD card
    label2 = 1;
    punchCount = punchCount + 1;
    buzzFlag2 = true;
  } else {
    label2 = 0;
  }

}

void deviceInitSound() {
  tone(buzzer, 2300);
  delay(200);
  noTone(buzzer);
  tone(buzzer, 2380);
  delay(200);
  noTone(buzzer);
  tone(buzzer, 2450);
  delay(200);
  noTone(buzzer);
}

void sdCardStart() {
  tone(buzzer, 2700);
  delay(100);
  noTone(buzzer);
  tone(buzzer, 2700);
  delay(100);
  noTone(buzzer);
}

void startButtonPush() {
  while (digitalRead(startBtn) != LOW);
}

void buzzerBeep() {
  if (buzzFlag1) {
    if (buzzState == false) {
      buzzState = true;
      tone(buzzer, 2200);
      buzzerTime = millis();
    } else {
      if ((millis() - buzzerTime) > 30) {
        noTone(buzzer);
        buzzState = false;
        buzzFlag1 = false;
      }
    }
  }

  if (buzzFlag2) {
    if (buzzState == false) {
      buzzState = true;
      tone(buzzer, 2300);
      buzzerTime = millis();
    } else {
      if ((millis() - buzzerTime) > 30) {
        noTone(buzzer);
        buzzState = false;
        buzzFlag2 = false;
      }
    }
  }
}

void analogSigProcessing(){
  // squaring signal makes it non-negative and increase the hight ot the spikes
    // hand #1
    int32_t sum1 = sq(ay1);
    // hand #2
    int32_t sum2 = sq(ay2);
    
    // exponentialy weighted average mage from acceleration signal
    
    // hand #1
    float MA1 = lambda * his1 + (1 - lambda) * sum1;
    his1 = MA1;

    // hand #2
    float MA2 = lambda * his2 + (1 - lambda) * sum2;
    his2 = MA2;

    // exponentialy weighted average for threshold, it is smoothing the acceleration data and
    // attempts to make dynamic threshold
    
    // hand #1
    float MA_thr1 = lambda2 * hisma1 + (1 - lambda2) * MA1;
    hisma1 = MA_thr1;

    // hand #2
    float MA_thr2 = lambda2 * hisma2 + (1 - lambda2) * MA2;
    hisma2 = MA_thr2;

    // dynamically shifts the threshold above the signal so it is less sensitive to noise.
    
    // hand #1
    float thr1 = MA_thr1 * 3;
    // hand #2
    float thr2 = MA_thr2 * 3;

    // if signal breaches the Threshold from the bottom up
    // hand #1
    
    if (MA1 > thr1) {
    
      if (sigBelowThr1 == true) {

        // breach from the bottom up
        sigBelowThr1 = false;
        breachFlag1 = true;

      } 
    } else {
      // MA1 < thr1
      if (sigBelowThr1 == false) {
        // signal just breached the thr from up downward
        sigBelowThr1 = true;
        breachFlag1 = false;
      }
    }

    // hand #2
    if (MA2 > thr2) {

      if (sigBelowThr2 == true) {

        // breach from the bottom up
        sigBelowThr2 = false;
        breachFlag2 = true;

      } 
    } else {
      // MA2 < thr2
      if (sigBelowThr2 == false) {
        // signal just breached the thr from up downward
        sigBelowThr2 = true;
        breachFlag2 = false;
      }
    }

    // Accumulating the values of MA while breached above the thr
    // hand #1
    if (breachFlag1 == true) {
      if (count1 < 100) {
        cumVal1 += MA1;
        count1++;
      } else {
        // breachFlag1 == true && count1 >= 100

        if (cumVal1 > punchThr1) {
          // Punch is detected, disabling the accumulation of samples and detection algo
          breachFlag1 = false;
          punchFlag1 = true;
        } else {
          // accumulated value was smaller than punchThr
          Serial.print("CumVal1: ");
          Serial.println(cumVal1 / 1000000);
          breachFlag1 = false;
        }
      }
    } else {
      // breachFlag == false, so we are below the thr
      count1 = 0;
      cumVal1 = 0;
    }

    // hand #2
    if (breachFlag2 == true) {
      
      if (count2 < 100) {
        cumVal2 += MA2;
        count2++;
      } else {
        // breachFlag2 == true && count2 >= 100

        if (cumVal2 > punchThr2) {
          // Punch is detected, disabling the accumulation of samples and detection algo
          breachFlag2 = false;
          punchFlag2 = true;
          
        } else {
          // accumulated value was smaller than punchThr
          Serial.print("CumVal2: ");
          Serial.println(cumVal2 / 1000000);
          breachFlag2 = false;
        }
      }
    } else {
      // breachFlag == false, so we are below the thr
      count2 = 0;
      cumVal2 = 0;
    }
}

// Write accel data into ring Buffer
void ringBuffWrite(){
// Print acceleration into RingBuf.
  rb.print(data[0]);
  rb.write(',');
  rb.print(data[1]);
  rb.write(',');
  rb.print(data[2]);
  rb.write(',');
  rb.print(label1);
  rb.write(',');
  rb.println(label2);
}

// function which gets Data from the MPU9250 and saves it to the SD card
void logData() {
 
    // Amount of data in ringBuf.
    n = rb.bytesUsed();

    // if file is near to full
    if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
      Serial.println("File full - quiting.");
      
      // Write any RingBuf data to file.
      rb.sync();
      file.truncate();
      file.rewind();
      Serial.print("fileSize: ");
      Serial.println((uint32_t)file.fileSize());
      Serial.print("maxBytesUsed: ");
      Serial.println(maxUsed);
      Serial.print("minSpareMicros: ");
      Serial.println(minSpareMicros);
      file.close();
    }
    
    if (n > maxUsed) {
      maxUsed = n;
    }
    
    if (n >= 512 && !file.isBusy()) {
      // Not busy only allows one sector before possible busy wait.
      // Write one sector from RingBuf to file.
      if (512 != rb.writeOut(512)) {
        Serial.println("writeOut failed");
        while(1){}
      }
    }
    
    // Time for next point.
    logTime += LOG_INTERVAL_USEC;
    int32_t spareMicros = logTime - micros();
    
    if (spareMicros < minSpareMicros) {
      minSpareMicros = spareMicros;
    }
    
    if (spareMicros <= 0) {
      Serial.print("Rate too fast ");
      Serial.println(spareMicros);
      while(1){}
    }  
  
    // Print acceleration into RingBuf.
    ringBuffWrite();
    if(punchFlag1 == true){
      punchFlag1 = false;
      punchCount = punchCount + 1;
      buzzFlag1 = true;
    }

    if(punchFlag2 == true){
      punchFlag2 = false;
      punchCount = punchCount + 1;
      buzzFlag2 = true;
    }
    
    if (rb.getWriteError()) {
      // Error caused by too few free bytes in RingBuf.
      Serial.println("WriteError");
      while(1){}
    }
}

void clearSerialInput() {
  for (uint32_t m = micros(); micros() - m < 10000;) {
    if (Serial.read() >= 0) {
      m = micros();
    }
  }
}

// Loops through the filenames on SD card and returns the next available filename.

//Beep at the end of the signal
void stopBeep(){
  tone(buzzer, 2200);
  delay(100);
  noTone(buzzer);
  tone(buzzer, 2400);
  delay(100);
  noTone(buzzer);
  
}

void countSdFiles(FsFile dir, int numTabs, int& fileCount) {
   while(true) {
     
     FsFile entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       break;
     }
     if (entry.isDirectory()) {
       countSdFiles(entry, numTabs+1, fileCount);
     }else {
        fileCount++;
     }
     entry.close();
   }
}


void setup() {
  // pin Settings
  pinMode(buzzer, OUTPUT);
  pinMode(startBtn, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  
  Serial.begin(9600);
  while (!Serial) {}
  // setting 12 bit resolution
  
  analogReadResolution(12);
  // Go faster or log more channels.  ADC quality will suffer.
  // analogReadAveraging(1);
  
  // setting of the buzzer pin to OUTPUT

  deviceInitSound();
  // Serial.println("Push button to start");
  //startButtonPush();
  // Serial.println("Starting the loop.");
  tic = millis();
  
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
  }

  // SD card preparation
   root = sd.open("/");
   countSdFiles(root, 0, fileCountOnSD);
   
  // finding the next possible filename
  fileName += LOG_FILENAME_NAME;
  fileName += fileCountOnSD-2;
  fileName +=LOG_FIMENAME_EXT;
  fileName = String(fileName);
  Serial.print("Next filename is: ");
  Serial.println(fileName);
  
  // Open or create file - truncate existing file.
  if (!file.open(fileName.c_str(), O_RDWR | O_CREAT | O_TRUNC)) {
    Serial.println("open failed\n");
    return;
  }

  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
    Serial.println("preAllocate failed\n");
    file.close();
    return;
  }

  // initialize the RingBuf.
  rb.begin(&file);
  //Serial.println("Type any character to stop");


  //MPU9250 setup
  status = IMU.mybegin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // setting the params for the MPU
  IMU.setAccelRange(IMU.ACCEL_RANGE_8G);
  attachInterrupt(MPU_INT_PIN,mpuDataReadyInt,RISING);
  IMU.enableDataReadyInterrupt();
  

  // beginning of the analog Timer
  analogTimer.begin(analogTimeInt, 100);
  analogTimer.priority(150);  
  logTime = micros();
}

void loop() {
  while(stopButtonFlag == false){
  // check if its time for digital data (takes up 13 usec)
  if(digiTimeFlag == true){
    //Serial.write(',');
    //Serial.print(digiTimeFlag);
    //Serial.write(',');
    // record the digital data
    IMU.readAccel(data);
    noInterrupts();
    digiTimeFlag = false;
    interrupts();
    digiDataReadyFlag = true; 
  }
 

  // check if its time for analog data (16 usec)
  if(analogTimeFlag == true){
    // record tha analog data (only axis pointing in the direction of the punch)
    ay1 = analogRead(analogAccel1) - 2045;
    ay2 = analogRead(analogAccel2) - 2045;

    noInterrupts();
    analogTimeFlag = false;
    interrupts();
    analogDataReadyFlag = true;
  }

 // this is no kidding 1-2 usecs
  if(analogDataReadyFlag == true){ 
    analogDataReadyFlag = false;
    /*Analog Data Punch Processing
      Sensor is placed on the glove with
      x-axis pointing to the left,
      z-axis up from the glove,
      y-axis back, away from the fist so extending the arm causes negative acceleration
      retraction of landing of the punch causes positive spike. */
      analogSigProcessing();
    
  }
  
  // setting labels for the data
  labelSetter();
  // saving the digital data to the sd card (10-12 usecs)
  if(digiDataReadyFlag == true){
    digiDataReadyFlag = false;

    // save the data to the SD card
    logData();  
  }

   // buzz with the buzzer if punch was detected
   buzzerBeep();

   // check if stop button was not pressed
   buttonPressed();
  }
  stopBeep();
  rb.sync();
  file.truncate();
  file.rewind();
  Serial.print("fileSize: ");
  Serial.println((uint32_t)file.fileSize());
  Serial.print("maxBytesUsed: ");
  Serial.println(maxUsed);
  Serial.print("minSpareMicros: ");
  Serial.println(minSpareMicros);
  file.close();
  while(1){}
}
