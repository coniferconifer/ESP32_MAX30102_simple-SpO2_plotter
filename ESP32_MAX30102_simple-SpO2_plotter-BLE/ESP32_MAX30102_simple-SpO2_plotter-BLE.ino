/*
  #  simple SpO2 plotter for MH-ET LIVE MAX30102 breakout board and ESP32 devkit-C
     SpO2 for Heart rate monitor application.
     This program sends SpO2 as a HRM of BLE standart service

  Using Sparkfun MAX3010X library
  https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library

  ESP32_MAX30102_simple-SpO2_plotter-BLE.ino
  by coniferconifer Copyright 2020
  LICENSED under Apache License 2.0

  Version 1.3

  Heart rate moniter is added.

  Shows SpO2 and the user's heart beat/rate on Arduino's serial plotter.
  No display hardware is required.
  works as a BLE peripheral.
  LED blinks with blips if LED with some pull down resister is attached on LED_INDICATOR port.

  This program should not be used for medical purposes.
  I wrote this to learn how SpO2 can be measured and pay tributes for the inventors.

  Pulse oximetry was developed in 1972, by Takuo Aoyagi and Michio Kishi,
  bioengineers, at Nihon Kohden in Japan.
  https://ethw.org/Takuo_Aoyagi

  Since MH-ET LIVE MAX30102 breakout board seems outputting IR and RED swapped
  when Sparkfun's library is used.

  red = particleSensor.getFIFOIR();
  ir = particleSensor.getFIFORed();
  is used in my code. If you have Sparkfun's MAX30105 breakout board , try to
  correct these lines.

  ## what's new
  - Heart rate monitor by zero crossing at falling edge is added.
  - LED Heart beat indicator on GPIO_15 , LED connected to GPIO_15 via pull down resister.(3.3kOhm for ex.)
  - BEEP piezo speaker on GPIO_12
  - when GPIO_4 is pull down to ground , this program sends Heart Rate to BLE client.

  ## Tips:
  SpO2 is calculated as R=((square root means or Red/Red average )/((square root means of IR)/IR average))
  SpO2 = -23.3 * (R - 0.4) + 100;
  // taken from a graph in http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf

  ## Instructions:
  0) Install Sparkfun's MAX3010X library
  1) Load code onto ESP32 with MH-ET LIVE MAX30102 board
  2) Put MAX30102 board in plastic bag and insulates from your finger.
     and attach sensor to your finger tip
  3) Run this program by pressing reset botton on ESP32
  4) Wait for 3 seconds and Open Arduino IDE Tools->'Serial Plotter'
     Make sure the drop down is set to 115200 baud
  5) Search the best position and presure for the sensor by watching
     the blips on Arduino's serial plotter
     I recommend to place LED under the backside of nail and wrap you
     finger and the sensor by rubber band softly.

  5) Checkout the SpO2 and blips by seeing serial Plotter
     100%,95%,90%,85% SpO2 lines are always drawn on the plotter

  ## Hardware Connections (Breakoutboard to ESP32 DevkitC):
  -VIN = 3.3V
  -GND = GND
  -SDA = 21 (or SDA)
  -SCL = 22 (or SCL)
  -INT = Not connected

  Use #define TFT_DISPLAY for ILI9341 320x240 display
  ILI9342 TFT display to ESP32 DevkitC
  -TFT_MISO = 19
  -TFT_MOSI = 23
  -TFT_SCLK = 18
  -TFT_CS  = 5  // Chip select control pin
  -TFT_DC  = 17 // Data Command control pin
  -TFT_RST = 16 // Reset pin (could connect to RST pin)
  -TOUCH_CS = 2 //not used
  ## Trouble Shooting:
  Make sure to solder jumper on 3V3 side.
  if you forget this, I2C does not work and can not find MAX30102.
  says "MAX30102 was not found. Please check wiring/power."

*/
#include <Wire.h>
#include "MAX30105.h" //sparkfun MAX3010X library
MAX30105 particleSensor;

//CUSTOM DEFINITION
#define TFT_DISPLAY // for 320x240 ILI9341 TFT display via eSPI
#define MAX30105 //if you have Sparkfun's MAX30105 breakout board , try #define MAX30105
#define BLE


#ifdef TFT_DISPLAY
#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
TFT_eSPI tft = TFT_eSPI();
#endif

//HARDWARE DEFINITION
#define LED_SOUND_INDICATOR
#define LEDPORT         GPIO_NUM_15
#define SPO2_HRM_SWITCH GPIO_NUM_4
#define SPEAKER         GPIO_NUM_12
#define BOOTSOUND 440 //Hz
#define BLIPSOUND 440*2 //Hz A
// beep sounder
#define LEDC_CHANNEL_2     2
#define LEDC_TIMER_13_BIT  13
#define LEDC_BASE_FREQ     5000

#ifdef BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

byte SpO2_data[8] = {0b00000110, 0, 0, 0, 0, 0, 0, 0}; //8bit SpO2 data , no extended data , defined in 0x2A37
bool _BLEClientConnected = false;
#define SpO2Service BLEUUID((uint16_t)0x180D)
BLECharacteristic SpO2MeasurementCharacteristics(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor SpO2Descriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      _BLEClientConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      _BLEClientConnected = false;
    }
};
/*
  BLE peripheral (server) initialization
*/
void Init_BLE_as_HeartRateMonitor() {
  //  Serial.println("Initializing...BLE as a heart rate monitor service 0x2A37");
  //  Serial.println("Smartphone application for smartphone can be used to monitor SpO2 as bpm");
  BLEDevice::init("simple SpO2 plotter");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pSpO2 = pServer->createService(SpO2Service);
  pSpO2->addCharacteristic(&SpO2MeasurementCharacteristics);
  SpO2Descriptor.setValue("use % instead of BPM");
  SpO2MeasurementCharacteristics.addDescriptor(&SpO2Descriptor);
  SpO2MeasurementCharacteristics.addDescriptor(new BLE2902());
  pServer->getAdvertising()->addServiceUUID(SpO2Service);
  pSpO2->start();
  pServer->getAdvertising()->start();// Start advertising
}
#endif
double avered = 0;//DC component of RED signal
double aveir = 0;//DC component of IR signal
double sumirrms = 0; //sum of IR square
double sumredrms = 0; // sum of RED square
unsigned int i = 0; //loop counter
#define SUM_CYCLE 100
int Num = SUM_CYCLE ; //calculate SpO2 by this sampling interval
double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component

#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in Arduino serial plotter at the same time
#define SAMPLING 1 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 50000 // if ir signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0
#define MAX_SPO2 100.0
#define MIN_SPO2 80.0

#ifdef TFT_DISPLAY
void display(float ir_forGraph, double Ebpm, double ESpO2, unsigned int loopCnt) {
#define LCD_WIDTH 320
#define LCD_HIGHT 240
#define BOTTOM_IR_SIGNAL 85.0
#define SCALE_FOR_PULSE 20.0
  unsigned int x; unsigned int y; int temp;
  x = loopCnt % LCD_WIDTH;
  temp = (int)((ir_forGraph - BOTTOM_IR_SIGNAL) * SCALE_FOR_PULSE);
  y = constrain(temp, 0, (LCD_HIGHT / 2) - 1);
//  Serial.printf(",%d , %d \r\n", x, y);
  tft.fillRect(x, LCD_HIGHT / 2, 30 , LCD_HIGHT, TFT_BLACK);
  tft.drawLine(x, LCD_HIGHT - 1, x, LCD_HIGHT - y, TFT_YELLOW);
  if (loopCnt % Num == 0) {
    tft.setCursor(30, 30);
    if ((int)ESpO2 == (int)MIN_SPO2) {
      tft.fillRect(0, 20, LCD_WIDTH, LCD_HIGHT / 3, TFT_BLUE);
      tft.print("No Finger");
    } else {
      tft.print("SpO2="); tft.print((int)ESpO2); tft.print(" BPM="); tft.print((int)Ebpm);
      tft.print("  ");// in case SpO2>=100 and BMP>=100 , delete last "0" when SpO2<100 or BMP<100
    }
  }
}
#endif
void setup()
{
#ifdef TFT_DISPLAY
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextSize(3);
  tft.fillRect(0, 20, LCD_WIDTH, LCD_HIGHT / 3, TFT_BLUE);
#endif

  Serial.begin(115200);
  Serial.println("Initializing...");
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    while (1);
  }

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
#ifdef BLE
  Init_BLE_as_HeartRateMonitor();
#endif
  pinMode(SPO2_HRM_SWITCH, INPUT_PULLUP);
#ifdef LED_SOUND_INDICATOR
  pinMode(LEDPORT, OUTPUT);
  digitalWrite(LEDPORT, HIGH);

  pinMode(SPEAKER, OUTPUT);
  ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT) ;
  ledcAttachPin(SPEAKER, LEDC_CHANNEL_2) ;
  tone(BOOTSOUND); delay(500); noTone();
#endif
}

void tone(int freq)
{
  ledcWriteTone(LEDC_CHANNEL_2, freq) ;
}
void noTone()
{
  ledcWriteTone(LEDC_CHANNEL_2, 0.0) ;
}
//
// Heart Rate Monitor by interval of zero crossing at falling edge
// max 180bpm - min 45bpm
#define FINGER_ON 50000 // if ir signal is lower than this , it indicates your finger is not on the sensor
#define LED_PERIOD 100 // light up LED for this period in msec when zero crossing is found for filtered IR signal
#define MAX_BPS 180
#define MIN_BPS 45
double HRM_estimator( double fir , double aveir)
{
  static double fbpmrate = 0.95; // low pass filter coefficient for HRM in bpm
  static uint32_t crosstime = 0; //falling edge , zero crossing time in msec
  static uint32_t crosstime_prev = 0;//previous falling edge , zero crossing time in msec
  static double bpm = 60.0;
  static double ebpm = 60.0;
  static double eir = 0.0; //estimated lowpass filtered IR signal to find falling edge without notch
  static double firrate = 0.85; //IR filter coefficient to remove notch , should be smaller than frate
  static double eir_prev = 0.0;


  // Heart Rate Monitor by finding falling edge
  eir = eir * firrate + fir * (1.0 - firrate); //estimated IR : low pass filtered IR signal
  if ( ((eir - aveir) * (eir_prev - aveir) < 0 ) && ((eir - aveir) < 0.0)) { //find zero cross at falling edge
    crosstime = millis();//system time in msec of falling edge
    //Serial.print(crosstime); Serial.print(","); Serial.println(crosstime_prev);
    if ( ((crosstime - crosstime_prev ) > (60 * 1000 / MAX_BPS)) && ((crosstime - crosstime_prev ) < (60 * 1000 / MIN_BPS)) ) {
      bpm = 60.0 * 1000.0 / (double)(crosstime - crosstime_prev) ; //get bpm
      //   Serial.println("crossed");
      ebpm = ebpm * fbpmrate + (1.0 - fbpmrate) * bpm;//estimated bpm by low pass filtered
#ifdef LED_SOUND_INDICATOR
      if (aveir > FINGER_ON) {
        digitalWrite(LEDPORT, HIGH);
        tone(BLIPSOUND - (100.0 - ESpO2) * 10.0); //when SpO2=80% BLIPSOUND drops 200Hz to indicate anormaly
      }
#endif
    } else {
      //Serial.println("faild to find falling edge");
    }
    crosstime_prev = crosstime;
  }
  eir_prev = eir;
#ifdef LED_SOUND_INDICATOR
  if (millis() > (crosstime + LED_PERIOD)) {
    if ( aveir > FINGER_ON ) {
      digitalWrite(LEDPORT, LOW);
      noTone();
    }
  }
#endif
  return (ebpm);
}

unsigned int loopCnt = 0;
void loop()
{
  uint32_t ir, red ;//raw data
  double fred, fir; //floating point RED ana IR raw values
  double SpO2 = 0; //raw SpO2 before low pass filtered
  double Ebpm;//estimated Heart Rate (bpm)

  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) {//do we have new data

#ifdef MAX30105
    red = particleSensor.getFIFORed(); //Sparkfun's MAX30105
    ir = particleSensor.getFIFOIR();  //Sparkfun's MAX30105
#else
    red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
#endif
    i++; loopCnt++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level

    Ebpm = HRM_estimator(fir, aveir); //Ebpm is estimated BPM

    if ((i % SAMPLING) == 0) {//slow down graph plotting speed for arduino Serial plotter by decimation
      if ( millis() > TIMETOBOOT) {
        float ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
        float red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        //trancation to avoid Serial plotter's autoscaling
        if ( ir_forGraph > MAX_SPO2) ir_forGraph = MAX_SPO2;
        if ( ir_forGraph < MIN_SPO2) ir_forGraph = MIN_SPO2;
        if ( red_forGraph > MAX_SPO2 ) red_forGraph = MAX_SPO2;
        if ( red_forGraph < MIN_SPO2 ) red_forGraph = MIN_SPO2;
        //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
        if ( ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
#define PRINT
#ifdef PRINT
        //Serial.print(bpm);// raw Heart Rate Monitor in bpm
        //Serial.print(",");
        Serial.print(Ebpm);// estimated Heart Rate Monitor in bpm
        Serial.print(",");
        //        Serial.print(Eir - aveir);
        //        Serial.print(",");
        Serial.print(ir_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(","); Serial.print(red_forGraph); // to display pulse wave at the same time with SpO2 data
        Serial.print(",");
        Serial.print(ESpO2); //low pass filtered SpO2
        Serial.print(","); Serial.print(85.0); //reference SpO2 line
        Serial.print(","); Serial.print(90.0); //warning SpO2 line
        Serial.print(","); Serial.print(95.0); //safe SpO2 line
        Serial.print(","); Serial.println(100.0); //max SpO2 line

#else
        Serial.print(fred); Serial.print(",");
        Serial.print(avered); Serial.println();
        //    Serial.print(fir);Serial.print(",");
        //   Serial.print(aveir);Serial.println();
#endif
#ifdef TFT_DISPLAY
        display(ir_forGraph, Ebpm, ESpO2, loopCnt);
#endif
      }
    }
    if ((i % Num) == 0) {
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      //#define MAXIMREFDESIGN
#ifdef MAXIMREFDESIGN
      //https://github.com/MaximIntegratedRefDesTeam/RD117_ARDUINO/blob/master/algorithm.h
      //uch_spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
      SpO2 = -45.060 * R * R + 30.354 * R + 94.845 ;
      //      SpO2 = 104.0 - 17.0*R; //from MAXIM Integrated https://pdfserv.maximintegrated.com/en/an/AN6409.pdf
#else
#define OFFSET 0.0
      SpO2 = -23.3 * (R - 0.4) + 100 - OFFSET ; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      if (SpO2 > 100.0 ) SpO2 = 100.0;
#endif
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
#ifdef BLE

      if ( ir < FINGER_ON) {
        ESpO2 = MINIMUM_SPO2; //indicator for finger detached
      }
      if ( digitalRead(SPO2_HRM_SWITCH) == LOW) {
        SpO2_data[1] = (byte)Ebpm;
      } else {
        SpO2_data[1] = (byte)ESpO2;
      }
      SpO2_data[2] =  0x00;
      SpO2MeasurementCharacteristics.setValue(SpO2_data, 2);
      SpO2MeasurementCharacteristics.notify();
#endif
      sumredrms = 0.0; sumirrms = 0.0; i = 0;//reset mean square at every interval
      break;
    }
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    //Serial.println(SpO2);
  }

}
