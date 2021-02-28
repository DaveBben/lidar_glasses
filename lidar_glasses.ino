#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <LIDARLite_v3HP.h>
#include <Wire.h>
#include <ArducamSSD1306.h>    // Modification of Adafruit_SSD1306 for ESP8266 compatibility
#include <Adafruit_GFX.h>   // Needs a little change in original Adafruit library (See README.txt file)

#define FAST_I2C
#define DEBUG 0 // Switch debug output on and off by 1 or 0

#define OLED_RESET  16  // Pin 15 -RESET digital signal

#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16



#if DEBUG
#define PRINT(s)        \
  {                     \
    Serial.print(F(s)); \
  }
#define PRINTLN(s)        \
  {                       \
    Serial.println(F(s)); \
  }
#define PRINTF(s, format)    \
  {                          \
    Serial.print(s, format); \
  }

#else
#define PRINT(s)
#define PRINTLN(s)
#define PRINTF(s, format)
#endif

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
LIDARLite_v3HP myLidarLite;
ArducamSSD1306 display(OLED_RESET); // FOR I2C

sensors_event_t mEvent;

bool finishedFirstMeasurement = false;

float sphericalStart[3] = {0, 0, 0};
float sphericalEnd[3] = {0, 0, 0};

float cartesianStart[3] = {0, 0, 0};
float cartesianEnd[3] = {0, 0, 0};

byte sample_counter = 0;
const byte SAMPLE_SZIE = 7;

const byte BUTTON_PIN = 2;

volatile unsigned long lastTimePressed = 0;
volatile boolean pressed = false;

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled

uint8_t getSystemCalibration()
{
  uint8_t sys = 0;
  bno.getCalibration(&sys,NULL,NULL,NULL);
  return sys;
}

void displayText(char *message){
    // SSD1306 Init
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(20,20);
  display.println(message);
  display.display();
}

void setup()
{

 display.begin();  // Switch OLED
 displayText("Lidar Glasses");

#if DEBUG
  Serial.begin(9600);
#endif

  PRINTLN("Lidar Measurement Glasses");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    PRINTLN("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();
#ifdef FAST_I2C
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
#endif

  // Configure the LidarLite internal parameters so as to lend itself to
  // various modes of operation by altering 'configure' input integer to
  // anything in the range of 0 to 5. See LIDARLite_v3HP.cpp for details.
  myLidarLite.configure(0);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonPressed, LOW);

  PRINTLN("Configuration complete");
}

void buttonPressed()
{
  unsigned long currentTime = millis();
  if ((currentTime - lastTimePressed > 50) && !pressed)
  {
    lastTimePressed = currentTime;
    pressed = true;
  }
}

uint16_t getDistance()
{
    myLidarLite.waitForBusy();
    myLidarLite.takeRange();
    myLidarLite.waitForBusy();

    return myLidarLite.readDistance();
}

float calculatePointDistance(float coordinatesOne[], float coordinatesTwo[])
{

  float x1 = coordinatesOne[0];
  float y1 = coordinatesOne[1];
  float z1 = coordinatesOne[2];
  float x2 = coordinatesTwo[0];
  float y2 = coordinatesTwo[1];
  float z2 = coordinatesTwo[2];

  float distance = sqrt(sq(x2 - x1) + sq(y2 - y1) + sq(z2 - z1));

  return distance;
}

float toInches(float cm){
  return cm/2.54;
}

float degreeToRadians(float degree)
{
  return degree * (PI / 180.0);
}

void sphericalToCartesian(float sphericalPoints[], float cartesianPoints[])
{

  float pan = sphericalPoints[0];
  float tilt = sphericalPoints[1];
  float distance = sphericalPoints[2];

  float x = distance * cos(degreeToRadians(pan)) * cos(degreeToRadians(tilt)); //x value
  float y = distance * cos(degreeToRadians(tilt)) * sin(degreeToRadians(pan)); //y value
  float z = distance * sin(degreeToRadians(tilt));                             //z value

  cartesianPoints[0] = x;
  cartesianPoints[1] = y;
  cartesianPoints[2] = z;
}

void sanitizeData(float datasetOne[], float datasetTwo[])
{
  int panDifference = abs(datasetTwo[0]) - abs(datasetOne[0]);
  int tiltDifference = abs(datasetTwo[1]) - abs(datasetOne[1]);
  if (abs(panDifference) > abs(tiltDifference))
  {
    datasetOne[1] = 0.0;
    datasetTwo[1] = 0.0;
    PRINTLN("This is a horiztonal measurement");
  }
  else
  {
    datasetOne[0] = 0.0;
    datasetTwo[0] = 0.0;
    PRINTLN("This is a vertical measurement");
  }
}

void clearArray(float inputArray[], byte arraySize)
{
  for (byte i = 0; i < arraySize; i++)
  {
    inputArray[i] = NULL;
  }
}

void printArray(float inputArray[], byte arraySize)
{
  for (byte i = 0; i < arraySize; i++)
  {
    PRINTF(inputArray[i], DEC);
    PRINT(",");
  }
  PRINTLN("");
}

void calculateAverages(float inputArray[], float outputArray[], byte sampleSize)
{
  outputArray[0] = inputArray[0] / sampleSize;
  outputArray[1] = inputArray[1] / sampleSize;
  outputArray[2] = inputArray[2] / sampleSize;
}

void loop()
{
  uint16_t distance = toInches(getDistance());
  uint8_t calibrationStatus = getSystemCalibration();

  if (calibrationStatus != 3 || distance == 0)
  {
    displayText("Not Ready");
    PRINTLN("System needs to be calibrated before measurement taken");
    PRINT("Distance Reading at: ");
    PRINTF(distance,DEC);
    PRINTLN("");
    PRINT("IMU Calibration is: ");
    PRINTF(calibrationStatus,DEC);
    PRINTLN("");
  }
  else
  {
    bno.getEvent(&mEvent);
    if (pressed)
    {
       
      if (!finishedFirstMeasurement)
      {

        if (sample_counter < SAMPLE_SZIE)
        {
          sphericalStart[0] = sphericalStart[0] + mEvent.orientation.x;
          sphericalStart[1] = sphericalStart[1] + mEvent.orientation.y;
          sphericalStart[2] = sphericalStart[2] + distance;
          sample_counter++;
        }
        else
        {
          calculateAverages(sphericalStart, sphericalStart, SAMPLE_SZIE);
          printArray(sphericalStart, 3);
          sample_counter = 0;
          pressed = false;
          finishedFirstMeasurement = true;
        }
      }
      else
      {

        if (sample_counter < SAMPLE_SZIE)
        {
          sphericalEnd[0] = sphericalEnd[0] + mEvent.orientation.x;
          sphericalEnd[1] = sphericalEnd[1] + mEvent.orientation.y;
          sphericalEnd[2] = sphericalEnd[2] + distance;
          sample_counter++;
        }
        else
        {
          calculateAverages(sphericalEnd, sphericalEnd, SAMPLE_SZIE);
          printArray(sphericalEnd, 3);

          sanitizeData(sphericalStart, sphericalEnd);

          printArray(sphericalStart, 3);
          printArray(sphericalEnd, 3);

          sphericalToCartesian(sphericalStart, cartesianStart);
          sphericalToCartesian(sphericalEnd, cartesianEnd);

          printArray(cartesianStart, 3);
          printArray(cartesianEnd, 3);

          float distance = calculatePointDistance(cartesianStart, cartesianEnd);
          
          PRINT("Distance is ");
          PRINTF(distance,DEC);
          
          char result[8];
          dtostrf(distance,6,2,result);
          
          displayText(result);

          delay(10000);

          clearArray(sphericalStart, 3);
          clearArray(sphericalEnd, 3);
          pressed = false;
          finishedFirstMeasurement = false;

          sample_counter = 0;
        }
      }
    }else{
      if(!finishedFirstMeasurement){
        displayText("Start Measure");
      }else{
         displayText("Stop Measure");
      }
      
    }
  }
}
