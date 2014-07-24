//yellow 4, green 5, tape higher #
//#include <Adafruit_GPS.h>
//#if ARDUINO >= 100
//#include <SoftwareSerial.h>
//#else
//#endif
//#if ARDUINO >= 100
//SoftwareSerial mySerial(6, 5);
//#else
//NewSoftSerial mySerial(1, 0);
//#endif
//Adafruit_GPS GPS(&mySerial);
#include <SFE_BMP180.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <OneWire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define GPSECHO  true
#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
//SFE_BMP180 pressure;
const int buttonPin = 7;
int DS18S20_Pin = 2;
int HIH4030_Pin = A0; //analog pin 0
int HMC6352SlaveAddress = 0x42;
int HMC6352ReadAddress = 0x41;
int GPSCounter = 0;
int bTick = 0;
int mNum = 0; 
int buttonState = 0;
int updateRate = 200;
int headingValue;
int timer = 0;
char cArray1[150] = {
};
int cNum = 1;
boolean GPSMode = false;
boolean layer1 = false;
boolean layer2 = false;
boolean usingInterrupt = false;
OneWire ds(DS18S20_Pin);
void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); 
  Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); 
  Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); 
  Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  Serial.println(" lux");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value); 
  Serial.println(" lux");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  //  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); 
  Serial.println("Auto");
  Serial.print  ("Timing:       "); 
  Serial.println("101 ms");
  Serial.println("------------------------------------");
}
void setup(void) 
{
  //Serial.begin(115200);
  Serial.begin(9600);
  Serial.println("Light Sensor Test"); 
  Serial.println("");   
  HMC6352SlaveAddress = HMC6352SlaveAddress >> 1;
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  pinMode(buttonPin, INPUT);     
  //GPS.begin(9600);
  /* Initialise the sensor */
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Setup the sensor gain and integration time */
  configureSensor();

  /* We're ready to go! */
  Wire.begin();
}
void loop(void) 
{
  display.setTextSize(1.9);
  display.setTextColor(WHITE);  
  sensors_event_t event;
  tsl.getEvent(&event);
  buttonState = digitalRead(buttonPin);
  if (buttonState==HIGH){
    mNum++;
    GPSCounter++;
  }
  if(buttonState!=HIGH)
  {
    GPSCounter=0;
  }
 // if(GPSCounter>=10)
  //{
   // GPSMode=true;
  //}
//  if(GPSMode==true)
//  {
//    useInterrupt(true);
//    delay(100);
//    useInterrupt(false);
//    display.setCursor(0,0);
//    display.print("GPS Location");
//    display.setCursor(0,16);
//    display.print("acquired!");
//    display.display();
//    display.clearDisplay();
//    GPSMode=false;
//    delay(1500);
//  }
//  else
//  {
    if(mNum>5)
      mNum=0;
    if(mNum==0)//temperature
    {
      float temperature = getTemp() - 2;
      float temperature2 = (temperature*2)+30;
      float temperature3 = temperature + 273.15;
      display.setCursor(0,0);
      if(temperature<-500)
      {
        display.print("Sensor busted");
        display.display();
        display.clearDisplay();
      } 
      else
      {
        display.println("Temperature:");
        Serial.println("Temperature:"); 
        display.setCursor(75,0);
        display.print(temperature);
        Serial.print(temperature);
        display.setCursor(110,0);
        display.print("C");
        display.setCursor(10,16);
        display.print(temperature3);
        display.setCursor(50,16);
        display.print("K");
        display.setCursor(75,16);
        display.println(temperature2);
        display.setCursor(110,16);
        display.print("F");
        display.display();
        display.clearDisplay();
      }
    }
    else if(mNum==1)//relative humidity
    {
      float temperature = getTemp() - 2;
      float relativeHumidity = getHumidity(temperature);
      display.setCursor(0,0);
      display.println("Humidity:");
      display.setCursor(55,0);
      display.print(relativeHumidity);

      display.setCursor(90,0);
      display.print("%");
      display.display();
      display.clearDisplay();
    }
    else if(mNum==2)//brightness or light level
    {
      display.setCursor(0,0);
      display.println("Brightness:");
      if(event.light!=0)
      {
        bTick=0;
      }
      if(bTick>=25)
      {
        display.clearDisplay();
        display.print("Sensor probs busted");
        display.display();
        display.clearDisplay();
      } 
      else if(event.light == 0)
      {
        display.setCursor(70,0);
        display.print("Really");
        display.setCursor(70,16);
        display.print("bright");
        display.display();
        display.clearDisplay();
        bTick++;
      }
      else if((int)event.light/1000 != 0)
      {
        display.setCursor(70,0);
        display.print(abs((int)event.light));
        display.setCursor(70,16);
        display.print("lux");
        display.display();
        display.clearDisplay();
      }
      else {
        display.setCursor(70,0);
        display.print(abs((int)event.light));
        display.setCursor(90,0);
        display.print("lux");
        display.display();
        display.clearDisplay();
      }
    }
    else if(mNum==3)//compass direction
    {
      Wire.beginTransmission(HMC6352SlaveAddress);
      Wire.write(HMC6352ReadAddress);
      Wire.endTransmission();
      delay(6);
      Wire.requestFrom(HMC6352SlaveAddress, 2);
      byte MSB = Wire.read();
      byte LSB = Wire.read();
      float headingSum = (MSB << 8) + LSB; //(MSB / LSB sum)
      float headingInt = headingSum / 10; 
      display.setCursor(0,0);
      display.println("Direction:");
      display.setCursor(70,0);
      if(headingInt>337.5)
      {
        display.print("N");
      }
      else if(headingInt>292.5)
      {
        display.print("NW");
      }
      else if(headingInt>247.5)
      {
        display.print("W");
      }
      else if(headingInt>202.5)
      {
        display.print("SW");
      }
      else if(headingInt>157.5)
      {
        display.print("S");
      }
      else if(headingInt>112.5)
      {
        display.print("SE");
      }
      else if(headingInt>67.5)
      {
        display.print("E");
      }
      else if(headingInt>22.5)
      {

        display.print("NE");
      }
      else display.print("N");
      display.setCursor(30,17);
      display.print(headingInt);
      display.setCursor(70,17);
      display.print("degrees");
      display.display();
      display.clearDisplay();
    }
//    else if(mNum==4)
//    {
//      double P, T;
//      T = getTemp();
//      char status = pressure.startPressure(3);
//      if (status != 0)
//      {
//        delay(status);
//        status = pressure.getPressure(P,T);
//        display.setCursor(0,0);
//        display.print("Absolute pressure:");
//        display.setCursor(70,0);
//        display.print((int)P);
//        display.setCursor(100,0);
//        display.print("mb");
//        display.setCursor(70,16);
//        display.print((int)(P*0.0295333727));
//        display.setCursor(100,16);
//        display.print("Hg");
//        display.display();
//        display.clearDisplay();
//      }
//    }
    else if(mNum==4)//weather predictions
    {
      display.setCursor(0,0);
      float temperature = getTemp()-2;
      float temperature2 = (temperature*2)+30;
      float relativeHumidity = getHumidity(temperature);
      display.print("Rain not likely");
      display.setCursor(0,16);
      display.println("Feels like");
      display.setCursor(65,16);
      display.println(heatIndex(temperature2, relativeHumidity));
      display.setCursor(97,12);
      display.println("o");
      display.setCursor(105,16);
      display.print("F");

      display.display();
      display.clearDisplay();
    }
    else if(mNum==5)
    {
      for(int x=0;x>128;x++)
        for(int y=0;y>32;y++)
          display.drawPixel(x,y,BLACK);
      display.display();
      display.clearDisplay();
    }
//  }
  delay(updateRate);
}
float heatIndex(float fahrenheit, float rHumidity)
{
  float hIndex = -42.379 + (2.04901523*fahrenheit) + 
    (10.14333127*rHumidity) + (-.22475541*fahrenheit*rHumidity) +
    (-.00683783*(fahrenheit*fahrenheit)) + (-.05481717*(rHumidity*rHumidity)) +
    (.00122874*(fahrenheit*fahrenheit)*rHumidity) + (.00085282*fahrenheit*(rHumidity*rHumidity)) +
    (-.00000199*(fahrenheit*fahrenheit)*(rHumidity*rHumidity));
  return hIndex;
} 
float getHumidity(float degreesCelsius){
  //caculate relative humidity
  float supplyVolt = 5.0;

  // read the value from the sensor:
  int HIH4030_Value = analogRead(HIH4030_Pin);
  float voltage = HIH4030_Value/1023. * supplyVolt; // convert to voltage value

  // convert the voltage to a relative humidity
  // - the equation is derived from the HIH-4030/31 datasheet
  // - it is not calibrated to your individual sensor
  //  Table 2 of the sheet shows the may deviate from this line
  float sensorRH = 161.0 * voltage / supplyVolt - 25.8;
  float trueRH = sensorRH / (1.0546 - 0.0026 * degreesCelsius); //temperature adjustment 

  return trueRH;
}
float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}
//SIGNAL(TIMER0_COMPA_vect) {
//  char c = GPS.read();
//  if(layer1)
//    if(c==(char)71)
//      layer2=true; 
//  if(c==(char)36)
//  {
//    layer1=true;
//    cArray1[0]=c;
//  }
//  if(layer2)
//  {
//    if(timer<150){
//      // if you want to debug, this is a good time to do it!
//      if (GPSECHO && c) {
//#ifdef UDR0
//        UDR0 = c;
//        cArray1[cNum]=c;
//        cNum++;  
//        timer++;
//        // writing direct to UDR0 is much much faster than Serial.print 
//        // but only one character can be written at a time. 
//#endif
//      }
//    }
//  }
//  if(timer>1000)
//    timer = 500;
//  if(cNum>=150)
//  {
//    layer1=false;
//    layer2=false;
//    c=0;
//  }
//}
//
//void useInterrupt(boolean v) {
//  if (v) {
//    // Timer0 is already used for millis() - we'll just interrupt somewhere
//    // in the middle and call the "Compare A" function above
//    OCR0A = 0xAF;
//    TIMSK0 |= _BV(OCIE0A);
//    usingInterrupt = true;
//  } 
//  else {
//    // do not call the interrupt function COMPA anymore
//    TIMSK0 &= ~_BV(OCIE0A);
//    usingInterrupt = false;
//  }
//}

