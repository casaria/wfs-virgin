/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/casaria3/Documents/workbench/wfs-virgin/wfs-virgin/src/wfs-virgin.ino"

/*
 * Project wfs-virgin
 * Description:
 * Author:
 * Date:
 */

// This #include statement was automatically added by the Particle IDE.
#include <DS18B20.h>

// This #include statement was automatically added by the Particle IDE.
#include <Adafruit_PCA9685.h>

//#include <PCF8574.h>

// This #include statement was automatically added by the Particle IDE.
#include <CASARIA_MCP3428.h>

#include <application.h>
#include <spark_wiring_i2c.h>

// MCP3428 I2C
//address is 0x68(104)
int cmdSetDamper(String command);
int cmdTXV2(String command);
int cmdTXV1(String command);
int cmdDefrost1(String command);
int cmdStopDefrost1(String command);
int cmdStopDefrost2(String command);
int cmdDefrost2(String command);
int cmdTempReset(String command);
void relayOff8574();
void relayOn8574();
void PerformDefrost2();
void PerformDefrost1();
void setup();
void publishDebug();
double getTemp(uint8_t addr[8]);
void publishData();
void publishPressure();
void publishStatus();
int ModulateDamper();
void relayOp(int relay, int op);
byte bitop(byte b1, byte b2, int op);
void turnOnRelay(int relay);
void turnOffRelay(int relay);
void toggleRelay(int relay);
void Relay8574(byte bitnum, boolean value);
void loop();
void getMCP();
String relayCmd(String acommand);
int triggerRelay(String command);
#line 25 "c:/Users/casaria3/Documents/workbench/wfs-virgin/wfs-virgin/src/wfs-virgin.ino"
#define Addr 0x68

// This #include statement was automatically added by the Particlee IDE.
#include <CASARIA_MCP23017.h>

#include <math.h>

//#include "DS18.h"

/*
TODO
- BME/BMP280
- Compressor 1
- Runtime using Tstamp
- issue with fan, crash, bettee Watchd dog
    - MQTT with SSL/TLS Qos2, locaL FALLBACK SERVER3
    
    
    */

#define damper2_1 0
#define damper2_2 1
#define damper1_1 2
#define damper1_2 3
#define txv1 4
#define txv2 8

#define RelayFan1 2 //if ((InReverse1) && (msReverse1 > 10000) && (psi[4]>320)) {
#define RelayCompr1 1
#define RelayCool1 3
#define RelayBypassDamper1 4

#define RelayFan2 7
#define RelayCompr2 8
#define RelayCool2 5
#define RelayBypassDamper2 6

static boolean InReverse1 = 0;
static boolean InReverse2 = 0;
static boolean HaltTemp = 0;
static boolean SwitchDefrost1 = 0;
static boolean SwitchDefrost2 = 0;

static int DefrostLevel1 = 0;
static int DefrostLevel2 = 0;

const int MAXRETRY = 3;
const int pinOneWire = D4;
const int pinLED = D7;

const uint32_t msPressureSampleTime = 400; //
const uint32_t msRelaySampleTime = 1900;   //
const uint32_t msTempSampleTime = 3000;
const uint32_t msPublishTime = 8000;          //30000
const uint32_t msPublishTime2 = 3000;         //30000
const uint32_t msPublishTime3 = 2400;         //30000
const uint32_t msCompressor1Startlock = 5000; //30000
const uint32_t msCompressor2Startlock = 5000; //30000
const uint32_t defrostTimer = 200000;  //900000
static uint32_t msDefrost1 = 0;
static uint32_t msDefrost2 = 0;

uint32_t msStartReverse1;
uint32_t msStartReverse2;

static int cmdPosArray[6];

// SYSTEM_THREAD(ENABLED);

// declare a global watchdog instance
// reset the system after 15 seconds if the application is unresponsive
ApplicationWatchdog wd(20000, System.reset, 1024);
// Initialize objects from the lib

const int nSENSORS = 12;

DS18B20 ds18b20(pinOneWire);
retained uint8_t sensorAddresses[8][nSENSORS];
float celsius[nSENSORS] = {NAN, NAN};
float temp;

//double celsius[nSENSORS] = {};
//uint8_t addr[8][12];

CASARIA_MCP3428 mcp1(0x68);
CASARIA_MCP3428 mcp2(0x6E);

//DS18 sensor(pinOneWire);

Adafruit_PCA9685 damper = Adafruit_PCA9685(0x40, true); // Use the default address, but also turn on debugging

CASARIA_MCP23017 relays; //not used

// PCF8574 I2C address is 0x24 (36)
#define Addr8574 0x24
static byte relayMap = 0xFF;

int b1status = 0;
int b2status = 0;
byte error;
int8_t mcp1address;
long Raw_adc[8];
float psi[8];
const int psiFS[8]{5000, 300, 300, 0, 500, 0, 0, 0};

//HP2,   LP2

int cmdSetDamper(String command)
{
  // Wire.reset();
  String valueString;
  for (int i = 0; i < 4; i++)
  {
    valueString = command.substring(i * 4, (i * 4 + 4));
    cmdPosArray[i] = valueString.toInt();
    //TXV2 PWM output is 8 (skipped 3)
    if (cmdPosArray[i] < 4096)
      damper.setVal(i + ((i < 6) ? 0 : 3), cmdPosArray[i]);
  }
  return 1;
}

int cmdTXV2(String command)
{
  int TXVpos;
  TXVpos = command.toInt();
  if (TXVpos <= 4096)
  {
    damper.setVal(4, TXVpos);
  }
  return TXVpos;
}

int cmdTXV1(String command)
{
  int TXVpos;
  TXVpos = command.toInt();
  if (TXVpos <= 4096)
  {
    damper.setVal(8, TXVpos);
  }
  return TXVpos;
}

int cmdDefrost1(String command)
{

  DefrostLevel1 = command.toInt();
  return DefrostLevel1;
}

int cmdStopDefrost1(String command)
{
  turnOnRelay(RelayCool1);
  turnOnRelay(RelayFan1);
  turnOnRelay(RelayBypassDamper1);
  InReverse1 = 0;
  //msDefrost2 = millis() + defrostTimer -100000;
  return 0;
}

int cmdStopDefrost2(String command)
{
  turnOnRelay(RelayCool2);
  turnOnRelay(RelayFan2);
  turnOnRelay(RelayBypassDamper2);
  InReverse2 = 0;
  return 0;
}

int cmdDefrost2(String command)
{

  DefrostLevel2 = command.toInt();
  return DefrostLevel2;
}

int cmdTempReset(String command)
{
  HaltTemp = TRUE;
  relayOff8574();
  delay(2000);
  //find all temp sensors
  ds18b20.resetsearch();
  delay(200); // initialise for sensor search
  for (int i = 0; i < nSENSORS; i++)
  {                                     // try to read the sensor addre
    ds18b20.search(sensorAddresses[i]); // and if available store
    delay(300);
    celsius[i] = 0;
  }
  HaltTemp = FALSE;
  relayOn8574();
  return 0;
}

void relayOff8574()
{
  Wire.begin();
  //Wire.reset();
  Wire.beginTransmission(Addr8574);
  // Select GPIO as input
  relayMap = 0xff;

  Wire.write(relayMap); //Wire.write(0xff);

  // All relafys turn off
  // Stop I2C transmission
  Wire.endTransmission();
}
void relayOn8574()
{
  Wire.begin();
  //Wire.reset();
  Wire.beginTransmission(Addr8574);
  // Select GPIO as input
  relayMap = 0x00;
  Wire.write(relayMap); //Wire.write(0xff);

  // All relafys turn off
  // Stop I2C transmission
  Wire.endTransmission();
}

void PerformDefrost2() {
    static int defstate = 0;
    static uint32_t msStartTime = 0;
    static int damper1 = damper.readPWMOn(damper2_1);
    static int damper2 = damper.readPWMOn(damper2_2);

    switch(defstate)
    {
    case 0:
      msStartTime = millis();
      turnOffRelay(RelayCool2);
      turnOffRelay(RelayFan2);
      turnOffRelay(RelayBypassDamper2);
      InReverse2 = !InReverse2;
      if (InReverse2)
        msStartReverse2 = millis();
      if (DefrostLevel2 >= 3)
      {
        msStartTime = msStartTime - 500000;
    
      }
            //wait for dampers
     // damper.setVal(damper1_2, 500);
     // damper.setVal(damper1_1, 500);
      defstate++;
      break;
      case 1:

      if (millis() > (msStartTime + 15000))
        defstate++;
      break;
      case 2:
      turnOnRelay(RelayFan2);
      if (millis() > (msStartTime + 45000))
        defstate++;
      break;
      case 3:
      turnOnRelay(RelayFan2);
      turnOnRelay(RelayCool2);
      defstate++;
      break;
     case 4:
      if (millis() > (msStartTime + 25000))
        defstate++;
      break;
      case 5:
        turnOnRelay(RelayBypassDamper2);
        defstate++; 
      break;
      case 6:
        if (millis() > (msStartTime + 45000))
            defstate++;

      break;
      case 7:
      
        damper.setVal(damper2_2, 2800);
        damper.setVal(damper2_1, 3000);

         DefrostLevel = 0;
         defstate = 0;

      break;

    default:
      break;
   }
}

void PerformDefrost1() {
    static int defstate = 0;
    static uint32_t msStartTime = 0;
    static int damper1 = damper.readPWMOn(damper1_1);
    static int damper2 = damper.readPWMOn(damper1_2);

    switch(defstate)
    {
    case 0:
      msStartTime = millis();
      turnOffRelay(RelayCool1);
      turnOffRelay(RelayFan1);
      turnOffRelay(RelayBypassDamper1);
      InReverse1 = !InReverse1;
      if (InReverse1)
        msStartReverse1 = millis();
      if (DefrostLevel1 >= 3)
      {
        msStartTime = msStartTime - 500000;
    
      }
            //wait for dampers
     // damper.setVal(damper1_2, 500);
     // damper.setVal(damper1_1, 500);
      defstate++;
      break;
      case 1:

      if (millis() > (msStartTime + 15000))
        defstate++;
      break;
      case 2:
      turnOnRelay(RelayFan1);
      if (millis() > (msStartTime + 45000))
        defstate++;
      break;
      case 3:
      turnOnRelay(RelayFan1);
      turnOnRelay(RelayCool1);
      defstate++;
      break;
     case 4:
      if (millis() > (msStartTime + 25000))
        defstate++;
      break;
      case 5:
        turnOnRelay(RelayBypassDamper1);
        defstate++; 
      break;
      case 6:
        if (millis() > (msStartTime + 45000))
            defstate++;

      break;
      case 7:
      
        damper.setVal(damper1_2, 2800);
        damper.setVal(damper1_1, 3000);

         DefrostLevel1 = 0;
         defstate = 0;

      break;

    default:
      break;
    }
}

void setup()
{
  cmdStopDefrost1("abort");
  cmdStopDefrost2("abort");
  relayOff8574();
  delay(5000);
  //find all temp sensors
  ds18b20.resetsearch();
  delay(200); // initialise for sensor search
  for (int i = 0; i < nSENSORS; i++)
  {                       -              // try to read the sensor addre
    ds18b20.search(sensorAddresses[i]); // and if available store
    delay(300);
    celsius[i] = 0;
  }

  //relays.setAddress(0x20);
  //relays.setRelays(16);
  //relays.setOutputs(0x00, 0x00);

  // relays.setOutput(4);
  //relays.init();

  //If this is a 32 channel relay board, the A0 address jumper is ALWAYS set on the second chipset, so should never be set here on the first

  //Particle.function("Te mpReset", cmdTempReset);
  Particle.function("TXV1", cmdTXV1);
  //Particle.function("TXV2", cmdTXV2);
  //Particle.function("DEFROST1", cmdDefrost1);
  //Particle.function("DEFROST2", cmdDefrost2);

  // Particle.function("SetDamper", cmdSetDamper);
  // Particle.variable("Bank_1", b1status);
  // Particle.variable("Bank_2", b2status);

  //Particle.variable("Input Status", b4status);
  //relays.turnOffAllRelays();
  pinMode(pinLED, OUTPUT);

  //delay(500);
  // Start serial communication and set baud rate = 9600
  //Serial.begin(9600);
  // Set variable

  damper.begin();          // This calls Wire.begin()
  damper.setPWMFreq(1500); // Maximum PWM frequency is 1600

  damper.setVal(damper2_2, 2800);
  damper.setVal(damper2_1, 3000);
  damper.setVal(damper1_2, 2800);
  damper.setVal(damper1_1, 3000);
  damper.setVal(txv1, 3500); //default TXV2
  damper.setVal(txv2, 3500); //default TXV

  // Initialise I2C communication as MASTER
  //Wire.begin();
  // Start I2C Transmission
  //Wire.beginTransmission(Addr);
  // Selct configuration command
  // Continuous conversion mode, Channel-1, 12-bit resolution
  //Wire.write(0x10);
  // Stop I2C Transmission
  //Wire.endTransmission();
  Wire.begin();
  //Wire.reset();
  Wire.beginTransmission(Addr8574);
  // Select GPIO as input
  relayMap = 0xFF;
  Wire.write(relayMap); //Wire.write(0xff);
  Relay8574(1, 1);

  // All relafys turn off
  // Stop I2C transmission
  Wire.endTransmission();
}

void publishDebug()
{
  // char szInfo[200];
}

double getTemp(uint8_t addr[8])
{

  double _temp;
  int i = 0;

  do
  {
    _temp = ds18b20.getTemperature(addr);
  } while (!ds18b20.crcCheck() && MAXRETRY > i++);

  if (i < MAXRETRY)
  {
    //_temp = ds18b20.convertToFahrenheit(_temp);
    // Serial.println(_temp);
  }
  else
  {
    _temp = -99;
    // Serial.println("Invalid reading");
  }
  return _temp;
}

void publishData()
{
  char szInfo[200];
  snprintf(szInfo, sizeof(szInfo), "%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f", celsius[0], celsius[1], celsius[2], celsius[3],
           celsius[4], celsius[5], celsius[6], celsius[7], celsius[8], celsius[9], celsius[10], celsius[11]);
  Particle.publish("dsTEMPS", szInfo, PRIVATE);

  /*

  for (int i = 0; i < nSENSORS; i++) {   //ry to read the sen-sor addre
    snprintf(szInfo, sizeof(szInfo), "%x %x %x %x %x %x %x %x %x %x %x %x", sensorAddresses[0],sensorAddresses[1], sensorAddresses[2], sensorAddresses[3],
      sensorAddresses[4], sensorAddresses[5], sensorAddresses[6], sensorAddresses[7], sensorAddresses[8],sensorAddresses[9],sensorAddresses[10],sensorAddresses[11]);
    Particle.publish("dsAddress", szInfo, PRIVATE);
  
  }
  */
}

void publishPressure()
{
  char szInfo[200];

  //  snprintf(szInfo, sizeof(szInfo), "%d&%d&%d&%d&%d&%d&%d&%d", Raw_adc[0], Raw_adc[1], Raw_adc[2], Raw_adc[3], Raw_adc[4], Raw_adc[5], Raw_adc[6], Raw_adc[7]);
  // Particle.publish("dsAnalogin",szInfo, PRIVATE );
  snprintf(szInfo, sizeof(szInfo), "%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f&%.2f", psi[0], psi[1], psi[2], psi[3], psi[4], psi[5], psi[6], psi[7]);
  Particle.publish("ds_psi", szInfo, PRIVATE);
}

void publishStatus()
{
}

/*
void printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (sensor.crcError()) {
    Serial.print("CRC Error ");
  }

  // Print the sensor type
  const char *type;
  switch(sensor.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  Serial.print(type);1

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  Serial.printf(
    " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
  );

  // Print the raw sensor data
  uint8_t data[9];
  sensor.data(data);
  Serial.printf(
    " data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]
  );
}
*/

/*
void getTemp(){
  static int i=0;


  if (sensor.read()) {
        
    // Do something cool with the temperature
    if (i>11) i=0;
    celsius[i]= sensor.celsius();
      sensor.addr(addr[i]);
    i++;
    printDebugInfo();

  // If sensor.read() didn't return true you can try again later
  // This next block helps debug what's wrong.
  // It's not needed for the sensor to work properly
  } else {
    // Once all sensors have been read you'll get searchDone() == true
    // Next time read() is called the first sensor is read again
    i++;
    if (sensor.searchDone()) {
      Serial.println("No more addresses.");
      // Avoid excessive printing when no sensors are connected
      delay(250);

    // Something went wrong
    } else {
      printDebugInfo();
    }
  }
    
}
*/
int ModulateDamper()
{
  static int16_t dutyCycle = 800;
  static int16_t damperinc = 100;
  dutyCycle += damperinc; // Edit this if you want to edit the brightness step size
  if (dutyCycle > 4095)
  {
    dutyCycle = 4095;
    damperinc = -damperinc;
  }
  if (dutyCycle <= 800)
  {
    dutyCycle = 800;
    damperinc = -damperinc;
  }
  damper.setVal(0, dutyCycle);
  return dutyCycle;
}

void relayOp(int relay, int op)
{
  if (relay > 8)
  {
  }
  else
  {
    byte rbit = (1 << (relay - 1));
    Wire.beginTransmission(Addr8574);
    // Select GPIO as input
    //(toggle) ? Wire.write(0x55) : Wire.write(0xAA);
    //Wire.write(0x00);
    // Stop I2C transmission
    relayMap = bitop(relayMap, rbit, op);
    Wire.write(relayMap);
    Wire.endTransmission();
  }
}
byte bitop(byte b1, byte b2, int op)
{
  switch (op)
  {
  case 1:
    return b1 | b2;
  case 2:
    return b1 & ~b2;
  case 3:
    return b1 ^ b2;
  }
  return 0;
}

void turnOnRelay(int relay)
{
  relayOp(relay, 2);
}
void turnOffRelay(int relay)
{
  relayOp(relay, 1);
}
void toggleRelay(int relay)
{
  relayOp(relay, 3);
}

void Relay8574(byte bitnum, boolean value)
{
  static bool toggle;

  Wire.beginTransmission(Addr8574);
  // Select GPIO as input
  //(toggle) ? Wire.write(0x55) : Wire.write(0xAA);
  //Wire.write(0x00);
  // Stop I2C transmission
  Wire.write(0x00);
  relayMap = 0x00;
  Wire.endTransmission();
  toggle = !toggle;
}

void loop()
{
  static uint32_t msTempSample = 0;
  static uint32_t msPressureSample = 0;
  static uint32_t msRelaySample = 0;
  static uint32_t msPublish = 0;
  static uint32_t msPublish2 = 0;
  static uint32_t msPublish3 = 0;
  static uint32_t msReverse1 = 0;
  static uint32_t msReverse2 = 0;

  static uint32_t now;

  if (msDefrost1 == 0)
    msDefrost1 = now + defrostTimer * 1.8;

  if (msDefrost2 == 0)
    msDefrost2 = now + defrostTimer * 1.4;

  wd.checkin();

  now = millis();

  if (now - msRelaySample >= msRelaySampleTime)
  {

    msRelaySample = millis();
  }

  if (now - msDefrost1 >= defrostTimer)
  {
    cmdDefrost1("4");
    msDefrost1 = millis();
  }

  if (now - msDefrost2 >= defrostTimer)
  {
    cmdDefrost2("4");
    msDefrost2 = millis();
  }

  if (now - msTempSample >= msTempSampleTime)
  {
    if (!HaltTemp)
    {
      for (int i = 0; i < nSENSORS; i++)
      {
        temp = getTemp(sensorAddresses[i]);

        if (!isnan(temp))
        {
          celsius[i] = temp;
        }
        else
        {
          temp = getTemp(sensorAddresses[i]);
          if (!isnan(temp))
            celsius[i] = temp;
        }

        wd.checkin();
      }
    }
    msTempSample = millis();
  }

  if (now - msPressureSample >= msPressureSampleTime)
  {
    getMCP();
    msPressureSample = millis();
    msReverse1 = millis() - msStartReverse1;
    msReverse2 = millis() - msStartReverse2;

    wd.checkin();
    /*if ((InReverse1) && (msReverse1 > 12000) && (psi[4] > 290))
    {
      //if ( (psi[4]>300)) {
      cmdStopDefrost1("OFF");
    }
    if ((InReverse2) && (msReverse2 > 12000) && (psi[0] > 290))
    {
      //  if ( (psi[0]>300)) {
      cmdStopDefrost2("OFF");
    } */
  }
  
  
  if (DefrostLevel1) PerformDefrost1();
  if (DefrostLevel2) PerformDefrost2();

  if (now - msPublish2 >= msPublishTime2)
  {
    msPublish2 = millis();
    publishPressure();
    //publishStatus();
    //publishDebug();
  }

  if (now - msPublish3 >= msPublishTime3)
  {

    msPublish3 = millis();
  }

  if (now - msPublish >= msPublishTime)
  {

    publishData();
    //ModulateDamper();
    msPublish = millis();
  }
  wd.checkin(); // resets the AWDT count


}

void getMCP()
{

  // Distributed with a free-will license.
  // Use it any way you want, prof it or free, provided it fits in the licenses of its associated works.
  // MCP3428
  // This code is designed to work with the MCP3428_I2CADC I2C Mini Module available from ControlEverything.com.
  // https://www.controleverything.com/content/Analog-Digital-Converters?sku=MCP3428_I2CADC#tabs-0-product_tabset-2

  byte error;
  int8_t address;

  address = mcp1.devAddr;
  // The i2c_scanner uses the  return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {

    for (int i = 1; i <= 4; i++)
    {
      // MCP3428 is configured to channel i with 12 bits resolution, continuous mode and gain defined to 1
      // This arrangement of the mentioned paarmeters can be changed as per convenience
      mcp1.SetConfiguration(i, 16, 1, 2);
      Raw_adc[i - 1] = mcp1.readADC();
      // Note that the library waits for a complete conversion
      psi[i - 1] = ((float(Raw_adc[i - 1])) - 5813) / (29390 - 5813) * psiFS[i - 1];
      // raw_adc = raw_adc * LSB(1 mV)/PGA for PGA = 1;       // 12-bit Resolution
      // raw_adc = raw_adc * LSB(250 µV)/PGA for PGA = 1;     // 14-bit Resolution
      // raw_adc = raw_adc * LSB(62.5 µV)/PGA for PGA = 1;    // 16-bit Resolution
    } //calibration/linearizatiom
      //CALIBRATION
      // correct zero offset 
      if (psi[0] < 600) {
        psi[0] = psi[0] + 15;
      }
  }  else
  { //errors occurred
  }

  address = mcp2.devAddr;
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {
    for (int i = 4; i <= 8; i++)
    {
      // MCP3428 is configured to channel i with 12 bits resolution, continuous mode and gain defined to 1
      // This arrangement of the mentioned paarmeters can be changed as per convenience
      mcp2.SetConfiguration(i, 16, 1, 2);

      // Note that the library waits for a complete conversion
      Raw_adc[i - 1] = mcp2.readADC();
      psi[i - 1] = ((float(Raw_adc[i - 1])) - 5813) / (29390 - 5813) * psiFS[i - 1];
    }
    //calibration/linearizatiom
    //CALIBRATION
    // correct zero offset error
  }
  else
  { //errors occurred
  }
}

String relayCmd(String acommand)
{
  int relay = 0;
  int bank = 0;
  int op = 0;
  int test = 0;
  bool all = false;
  int p = acommand.indexOf(" ");

  bool relay_next = false;
  String word;
  if (p > -1)
  {
    while (acommand.length() > 0)
    {
      if (p > -1)
      {
        word = acommand.substring(0, p);
        acommand = acommand.substring(p + 1);
        //preservedCommand = command.substring(0, p);
        p = acommand.indexOf(" ");
      }
      else
      {
        word = acommand;
        // command = "";
        //preservedCommand  = word;
      }

      if (word.equalsIgnoreCase("on") || word.equalsIgnoreCase("activate"))
      {
        if (op == 0 || bank > 0)
          op = 2;
      }
      if (word.equalsIgnoreCase("off") || word.equalsIgnoreCase("deactivate"))
      {
        op = 1;
      }
      if (word.equalsIgnoreCase("toggle") || word.equalsIgnoreCase("flip"))
      {
        op = 3;
      }
      if (relay_next)
      {

        if (test == 0)
        {
          test = word.toInt();
        }
        if (test > 0)
        {
          relay = test;
          relay_next = false;
        }
      }

      if (word.equalsIgnoreCase("relay") || word.equalsIgnoreCase("output"))
      {
        relay_next = true;
      }

      if (word.equalsIgnoreCase("bank"))
      {
        all = true;
      }
      if (word.equalsIgnoreCase("all"))
      {
        all = true;
      }
    }
  }
  if (all)
  {
    return acommand;
  }
  else
  {
    if (bank > 0)
      relay += ((bank - 1) * 8);
    relays.relayOp(relay, op);
    return "";
  }
}

int triggerRelay(String command)
{

  //relays.relayTalk(relayCmd(command));
  relays.relayTalk(command);
  return 1;
}
