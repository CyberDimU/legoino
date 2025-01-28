/**************************************************************************/
#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include <TFT_eSPI.h>  // Graphics and font library for ST7789 driver chip
//#include <SPI.h>
#include <images.h>

#include "Lpf2Hub.h"
#include <PNGdec.h>


String NameHub;
byte VerHub;

#define PIN_POWER_ON 15
#define light_pin 0
#define speed_pin 14

#define pinbat ADC1_CHANNEL_3   //inner power (usb/batt)
#define pinX ADC1_CHANNEL_1 //pin 2 on the board
#define pinY ADC1_CHANNEL_2 //pin 3 on the board

adc_atten_t atten = ADC_ATTEN_DB_12;


boolean FoundHUb = false;

byte state_light;
byte state_speed;
byte power_strlight = 0;


int RSSI = 0;
int modelbattery = 0;
float pultbattery = 0;
int16_t gyroX=0;
int16_t gyroY=0;
int16_t gyroZ=0;
int metter_spd;
int metter_str;

//timers - for del
unsigned long bat_time;
unsigned long tick_str;
unsigned long tick_drw;
unsigned long tick_drwS;

float oldbattery_voltage = 0;
byte ols = 1;
byte oss = 1;

int vref = 1100;


TFT_eSPI tft = TFT_eSPI(170, 320);  // Invoke library, pins defined in User_Setup.h


PNG png;  // PNG decoder instance

#define MAX_IMAGE_WIDTH_audi 135    // Adjust for your images
#define MAX_IMAGE_WIDTH_porshe 135  // Adjust for your images

#define MAX_IMAGE_WIDTH_light_off 32  // Adjust for your images
#define MAX_IMAGE_WIDTH_light_on 32   // Adjust for your images
#define MAX_IMAGE_WIDTH_max_speed 31  // Adjust for your images
#define MAX_IMAGE_WIDTH_std_speed 31  // Adjust for your images

int16_t xpos = 50;
int16_t ypos = 200;

int16_t xpos_light = 50;
int16_t ypos_light = 200;

int16_t xpos_speed = 50;
int16_t ypos_speed = 200;



// create a hub instance
Lpf2Hub myMoveHub;

bool isHwVersionAvailable = false;
bool isFwVersionAvailable = false;
bool isBatteryTypeAvailable = false;



bool isInitialized = false;

byte portA = (byte)ControlPlusHubPort::A;
byte portB = (byte)ControlPlusHubPort::B;
byte portC = (byte)ControlPlusHubPort::C;
byte portD = (byte)ControlPlusHubPort::D;

void setup() {
  pinMode(PIN_POWER_ON, OUTPUT);  //set to work TFT if use battery
  digitalWrite(PIN_POWER_ON, HIGH); //set to work TFT if use battery

  Serial.begin(115200);
  
  pinMode(light_pin, INPUT);
  pinMode(speed_pin, INPUT);
  
  ols = digitalRead(light_pin);
  oss = digitalRead(speed_pin);

  state_light = 0;
  state_speed = 2;  //0000 0010 - slowly

  pinMode(TFT_BL, OUTPUT);     // TTGO T-Display enable Backlight pin 4
  digitalWrite(TFT_BL, HIGH);  // T-Display turn on Backlight
  tft.init();
  tft.fillScreen(TFT_BLACK);

  tft.setRotation(0);
  tft.setTextSize(1);
  tft.setTextColor(TFT_GREEN);
  tft.drawCentreString(F("READY"), 85, 20, 4);
  tft.drawCentreString(F("TO MOVE "), 85, 42, 4);

  tft.setTextColor(TFT_NAVY);
  tft.drawCentreString(F("Press BIND"), 85, 82, 4);
  tft.drawCentreString(F("on the car"), 85, 104, 4);

  tft.setTextColor(TFT_RED);
  tft.drawCentreString(F("And reset transmiter"), 85, 144, 2);
  tft.drawCentreString(F("if needed..."), 85, 160, 2);

  isInitialized = false;
  bat_time = 0;
  tick_str = 0;
  tick_drw = 0;
  tick_drwS = 0;
  power_strlight = 0;

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(pinbat, atten);
  adc1_config_channel_atten(pinX, atten);
  adc1_config_channel_atten(pinY, atten);




  myMoveHub.init();  // initalize the MoveHub instance
}


void hubPropertyChangeCallback(void *hub, HubPropertyReference hubProperty, uint8_t *pData) {
  Lpf2Hub *myHub = (Lpf2Hub *)hub;
  

  //Serial.print("HubProperty: ");
  //Serial.println((byte)hubProperty, DEC);

  if (hubProperty == HubPropertyReference::RSSI) {
    
    RSSI = int(myHub->parseRssi(pData));
    //drawInfo();
    return;
  }


  if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE) {
    modelbattery = int(myHub->parseBatteryLevel(pData));
    drawModelBattery();
    return;
  }

  if (hubProperty == HubPropertyReference::FW_VERSION) {
    Version version = myHub->parseVersion(pData);
    Serial.print("FWVersion: ");
    Serial.print(version.Major);
    Serial.print("-");
    Serial.print(version.Minor);
    Serial.print("-");
    Serial.print(version.Bugfix);
    Serial.print(" Build: ");
    Serial.println(version.Build);
    isFwVersionAvailable = true;
    return;
  }

  if (hubProperty == HubPropertyReference::HW_VERSION) {
    Version version = myHub->parseVersion(pData);
    Serial.print("HWVersion: ");
    Serial.print(version.Major);
    Serial.print("-");
    Serial.print(version.Minor);
    Serial.print("-");
    Serial.print(version.Bugfix);
    Serial.print(" Build: ");
    Serial.println(version.Build);
    isHwVersionAvailable = true;
    return;
  }
}

// callback function to handle updates of sensor values
void portValueChangeCallback(void *hub, byte portNumber, DeviceType deviceType, uint8_t *pData) {
  Lpf2Hub *myHub = (Lpf2Hub *)hub;
  static unsigned long TimerGyro=0;

  if (deviceType == DeviceType::TECHNIC_MEDIUM_HUB_TILT_SENSOR) {
     unsigned long tektimeG = millis();
     if (tektimeG-TimerGyro>300)                 
     {
      TimerGyro = tektimeG;
      gyroX = myHub->parseControlPlusHubTiltSensorX(pData);
      gyroY = myHub->parseControlPlusHubTiltSensorY(pData);
      gyroZ = myHub->parseControlPlusHubTiltSensorZ(pData);
      if (VerHub == 8)  //x<->z
      {
        gyroX = gyroX + gyroZ;
        gyroZ = gyroX - gyroZ;
        gyroX = gyroX - gyroZ;
      }
     
      //drawInfo();
     } 

  }
}


// main loop
void loop() {
  esp_adc_cal_characteristics_t adc_chars;

  //Get the battery voltage in the remote control
  unsigned long new_bat_time = millis();
  //tft.setTextColor(TFT_WHITE,TFT_BLACK);
  //tft.drawString(String(new_bat_time)+"  ", 20, 300, 2);
   //digitalWrite(TFT_BL, HIGH);  // T-Display turn on Backlight

if ((new_bat_time - tick_drwS) > 1000)
  {
  tick_drwS = new_bat_time;
 
tft.setTextColor(TFT_WHITE,TFT_BLACK);
tft.drawCentreString("      Steering: "+String(metter_str)+" Engine: "+String(metter_spd)+"      ",85,300,2);
//Serial.println("      Steering: "+String(metter_str)+" Engine: "+String(metter_spd));
  }

  if (((new_bat_time - bat_time) > 2000) || (bat_time == 0)) {
      bat_time = new_bat_time;
    
    uint32_t tin = adc1_get_raw(pinbat);
    

    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, vref, &adc_chars);
    uint32_t intbattery_voltage = esp_adc_cal_raw_to_voltage(tin, &adc_chars) * 2; //The partial pressure is one-half
    pultbattery = float(intbattery_voltage)/1000;      
    //Serial.println("bat: "+String(tin)+"  "+String(intbattery_voltage)+"   "+String(battery_voltage));

    if (pultbattery != oldbattery_voltage) {
      oldbattery_voltage = pultbattery;
      if (pultbattery > 4.30) {
        tft.drawBitmap(60, 0, bat_charge_16x16, 16, 16, TFT_RED, TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
      } else {
        tft.drawBitmap(60, 0, bat_16x16, 16, 16, TFT_NAVY, TFT_BLACK);
        tft.setTextColor(TFT_NAVY, TFT_BLACK);
      }
      tft.drawString(String(pultbattery), 80, 0, 2);
    }
  }


  if ((!myMoveHub.isScanning()) && (!FoundHUb))  //If HUB is not found, we will start the search again
  {
    myMoveHub.init();
  }

  if (!FoundHUb)  //If HUB is not found, we will pause so as not to overload the board
  {
    delay(60);
  }



  if (myMoveHub.isConnected()) {
    drawInfo();
    if (VerHub == 8) {
      NewHubMove();
    } else {
      OldHubMove();
    }

  }

  if (myMoveHub.isConnecting()) {

    FoundHUb = true;
    myMoveHub.connectHub();


    if (myMoveHub.isConnected() && !isInitialized) {
      Serial.println("Connected to HUB");
      tft.fillScreen(TFT_BLACK);
      bat_time = 0;

      //Not all HUBs return a name, so we'll get it next.
      //delay(50); //needed because otherwise the message is to fast after the connection procedure and the message will get lost
      //myMoveHub.activateHubPropertyUpdate(HubPropertyReference::ADVERTISING_NAME, hubPropertyChangeCallback);

      delay(50);
      myMoveHub.activateHubPropertyUpdate(HubPropertyReference::BATTERY_VOLTAGE, hubPropertyChangeCallback);

      //delay(50);
      //myMoveHub.activateHubPropertyUpdate(HubPropertyReference::BUTTON, hubPropertyChangeCallback);

      delay(50);
      myMoveHub.activateHubPropertyUpdate(HubPropertyReference::RSSI, hubPropertyChangeCallback);

      delay(50);
      myMoveHub.activatePortDevice((byte)MoveHubPort::TILT, portValueChangeCallback);

      //delay(50);
      //myMoveHub.activatePortDevice((byte)MoveHubPort::CURRENT, portValueChangeCallback);

      delay(50);
      myMoveHub.activatePortDevice((byte)MoveHubPort::VOLTAGE, portValueChangeCallback);

      delay(50);
      myMoveHub.activatePortDevice((byte)ControlPlusHubPort::TILT, portValueChangeCallback);

      delay(50);
      myMoveHub.activatePortDevice((byte)ControlPlusHubPort::GYRO, portValueChangeCallback);

      NameHub = MegaTrim(myMoveHub.getHubName().c_str());

      drawName();

      VerHub = byte(myMoveHub.getHubType());
      //Serial.println(VerHub);

     if (VerHub == 8) {
        myMoveHub.calibrateTechnicMoveSteering();
     }
     drawModel(); 
     delay(50);
         

      isInitialized = true;
    } else {
      Serial.println("Failed to connect to HUB");
    }
  }

  // FW/HW version properties and battery type need a explicit request to get the values back because they does not change over time
  if (myMoveHub.isConnected() && !isFwVersionAvailable) {
    myMoveHub.requestHubPropertyUpdate(HubPropertyReference::FW_VERSION, hubPropertyChangeCallback);
    delay(100);
  }
  if (myMoveHub.isConnected() && !isHwVersionAvailable) {
    myMoveHub.requestHubPropertyUpdate(HubPropertyReference::HW_VERSION, hubPropertyChangeCallback);
    delay(100);
  }
  if (myMoveHub.isConnected() && !isBatteryTypeAvailable) {
    myMoveHub.requestHubPropertyUpdate(HubPropertyReference::BATTERY_TYPE, hubPropertyChangeCallback);
    delay(100);
  }

  // if connected, print out continously the hub property values

}  // End of loop



void OldHubMove() {
  int spd;
  int spds;
  //int x = analogRead(37);  //Engine
  //int y = analogRead(36);  //Steering
  int x = adc1_get_raw(pinX);
  int y = adc1_get_raw(pinY);
    

  /*Serial.print("X:");
  Serial.print(x);
  Serial.print("  Y:");
  Serial.println(y);
  delay(10);*/

  if (x > 2030) {
    spd = constrain(map(x, 2030, 2905, 0, 100),0,100);
    
    myMoveHub.setBasicMotorSpeed(portA, spd);
    myMoveHub.setBasicMotorSpeed(portB, spd);

  } else if (x < 1990) {
    spd = map(x, 1990, 1515, 0, -100);
    myMoveHub.setBasicMotorSpeed(portA, spd);
    myMoveHub.setBasicMotorSpeed(portB, spd);
  } else {
    myMoveHub.stopBasicMotor(portA);
    myMoveHub.stopBasicMotor(portB);
    spd = 0;
  }

  if (y > 2030)  //Влево
  {
    spds = map(y, 2030, 4095, 0, -100);
    myMoveHub.setAbsoluteMotorPosition(portD, 100, spds);
  } else if (y < 1990) {
    spds = map(y, 1990, 10, 0, 100);
    myMoveHub.setAbsoluteMotorPosition(portD, 100, spds);
  } else {
    //Установить в 0
    myMoveHub.setAbsoluteMotorPosition(portD, 100, 0);
    spds=0 ;
  }

metter_str = spds;
metter_spd = spd; 

drawmymetter();

}

void NewHubMove() {
 
//  int x = analogRead(pinX);  //Engine
//  int y = analogRead(pinY);  //Steering
int x = adc1_get_raw(pinX);
int y = adc1_get_raw(pinY);
  
/*
  Serial.print("X:");
 Serial.print(x);
 Serial.print("  Y:");
 Serial.println(y);
  delay(100);
*/

  byte lightclick = digitalRead(light_pin);
  byte speedclick = digitalRead(speed_pin);
  byte flag = 0;

  if ((lightclick == 0) && (ols == 1)) {
    //Serial.println("lightclick");
    unsigned long duration = pulseIn(light_pin, HIGH, 2000000);
    //Serial.println(duration);
    if (duration==0) 
    {
      tft.init();
      drawName();
                  
      tft.resetViewport();
      drawModelBattery();                                           
      drawModel();
      
    }
    else 
    {
    state_light = state_light ^ 4;  //0000 0100
    flag = 1;}
    ols = 0;
  }

  if ((speedclick == 0) && (oss == 1)) {
    //Serial.println("speedclick");
    state_speed = state_speed ^ 2;  //0000 0010
      
    //Serial.println(state_speed);
    flag = 1;
    oss = 0;
  }

  ols = digitalRead(light_pin);
  oss = digitalRead(speed_pin);

  if (flag == 1) {
    porshe_light_speed_drw();
  }

  byte light = 0;
  byte spd = 0;
  byte spds = 0;
  byte estop = 0;

  if (x > 2030) {
    spd = constrain(map(x, 2030, 2920, 0, 127),0,127);
    //if (spd > 127) { spd = 127; }
    metter_spd = map(spd,0,127,0,100);
  
  } else if (x < 1990) {
    spd = constrain(map(x, 1990, 1515, 255, 128),128,255);
    metter_spd = map(spd,255,128,0,-100);
  } else {
    spd = 0;
    metter_spd = 0;
  }

  if (y > 2030)  //Влево
  {
    left_blink();
    spds = constrain(map(y, 2030, 4095, 255, 128),128,255);
    metter_str = map(spds,255, 128, 0, -100);

  } else if (y < 1990) {
    right_blink();
    spds = constrain(map(y, 1990, 10, 0, 127),0,127);
    metter_str = map(spds, 0 ,127, 0, 100);

  } else {
    spds = 0;
    metter_str = 0;
    myMoveHub.setTechnicLed(B001001, 0);
  }

  /* Serial.print("spd: ");
    Serial.println(spd);
    Serial.print("spds: ");
    Serial.println(spds);
  */

  light = (state_light ^ state_speed);

  if ((x < 2260) && (x > 2230)) {
    estop = 1;  //stop
  } else {
    estop = 0;
  }

  light = light | estop;

  myMoveHub.setTechicMoveMotorSpeed(spd, spds, light);

  drawmymetter();
}


void drawmymetter()
{
 
unsigned long tektime= millis();
 static int oprspd;
 static int oprstr;
int prstr;
int prspd;

 if (((tektime - tick_drw) > 500)&&((oprstr=!prstr)||(oprspd=!prspd)))
 
 {
  tick_drw = tektime;
  prstr = map(metter_str, -100, 100, -72, 71);
  tft.fillRoundRect(14,160,151,15,1,TFT_NAVY);
 // tft.fillRoundRect(93,160,74, 15,1,TFT_NAVY);
  tft.fillRoundRect(87+prstr,160,7,15,1,TFT_RED);

 if (metter_spd>0)
 {
  prspd = constrain(map(metter_spd,0,100, 0, 60),0,60);
 }
 else
 {
   prspd = constrain(map(metter_spd,0,-100, 0, -35),-35,0);
 }

tft.fillRoundRect(150,55,15,60,1,TFT_GREEN);
tft.fillRoundRect(150,115,15,40,1,TFT_RED);

tft.fillRoundRect(150,114-prspd,15,10,1,TFT_NAVY);



oprspd = prspd;
oprstr = prstr;
 }       

}



void right_blink() {
  unsigned long tektime = millis();
  if ((tektime - tick_str) > 900) {
    tick_str = tektime;
    power_strlight = power_strlight ^ 48;
    myMoveHub.setTechnicLed(B000001, 0);
    myMoveHub.setTechnicLed(B001000, power_strlight);
    //Serial.println("L" + String(power_strlight));
  }
}

void left_blink() {
  unsigned long tektime = millis();
  if ((tektime - tick_str) > 900) {
    tick_str = tektime;
    power_strlight = power_strlight ^ 48;
    myMoveHub.setTechnicLed(B001000, 0);
    myMoveHub.setTechnicLed(B000001, power_strlight);
    //Serial.println("R" + String(power_strlight));
  }
}


void pngDraw(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH_audi];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos, ypos + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}

void pngDraw_light(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH_audi];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos_light, ypos_light + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}

void pngDraw_speed(PNGDRAW *pDraw) {
  uint16_t lineBuffer[MAX_IMAGE_WIDTH_audi];
  png.getLineAsRGB565(pDraw, lineBuffer, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  tft.pushImage(xpos_speed, ypos_speed + pDraw->y, pDraw->iWidth, 1, lineBuffer);
}


void porshe_light_speed_drw()

{
  xpos_speed = 124;
  ypos_speed = 245;

  xpos_light = 20;
  ypos_light = 190;

  if (state_speed == 2) {
    int16_t rc = png.openFLASH((uint8_t *)std_speed, sizeof(std_speed), pngDraw_speed);
    if (rc == PNG_SUCCESS) {
      tft.startWrite();
      uint32_t dt = millis();
      rc = png.decode(NULL, 0);
      tft.endWrite();
      // png.close(); // not needed for memory->memory decode
    } else {
      //Failed to get image
      //Serial.println("Failed to get image");
    }
  }

  else {
    int16_t rc = png.openFLASH((uint8_t *)max_speed, sizeof(max_speed), pngDraw_speed);
    if (rc == PNG_SUCCESS) {
      tft.startWrite();
      uint32_t dt = millis();
      rc = png.decode(NULL, 0);
      tft.endWrite();
      // png.close(); // not needed for memory->memory decode
    } else {
      //Failed to get image
      //Serial.println("Failed to get image");
    }
  }

  if (state_light == 4) {
    int16_t rc = png.openFLASH((uint8_t *)light_off, sizeof(light_off), pngDraw_light);
    if (rc == PNG_SUCCESS) {
      tft.startWrite();
      uint32_t dt = millis();
      rc = png.decode(NULL, 0);
      tft.endWrite();
      // png.close(); // not needed for memory->memory decode
    } else {
      //Failed to get image
      //Serial.println("Failed to get image");
    }

  } else {
    int16_t rc = png.openFLASH((uint8_t *)light_on, sizeof(light_on), pngDraw_light);
    if (rc == PNG_SUCCESS) {
      tft.startWrite();
      uint32_t dt = millis();
      rc = png.decode(NULL, 0);
      tft.endWrite();
      // png.close(); // not needed for memory->memory decode
    } else {
      //Failed to get image
      //Serial.println("Failed to get image");
    }
  }
}

String MegaTrim(const String instr)
{
  //String врNameHub = myMoveHub.getHubName().c_str();
String outres;
      
      for (int i=0;i<sizeof(instr);i++)
      {
        char ccc = instr[i];
        if (ccc>0) {outres = outres+ccc;}
      }
      outres.trim();
      return(outres);
}

void drawModel()

{

if (VerHub == 8) {

        int16_t rc = png.openFLASH((uint8_t *)porshe, sizeof(porshe), pngDraw);
        if (rc == PNG_SUCCESS) {
          xpos = 20;
          ypos = 195;
          tft.startWrite();
          uint32_t dt = millis();
          rc = png.decode(NULL, 0);
          tft.endWrite();
          // png.close(); // not needed for memory->memory decode
        } else {
          //Failed to get image
          //Serial.println("Failed to get image");
        }

        delay(300);

        //tft.fillRoundRect(150,55,15,60,1,TFT_GREEN);
        //tft.fillRoundRect(150,115,15,40,1,TFT_RED);

        //tft.fillRoundRect(15,160,146,15,1,TFT_NAVY);
        //tft.fillRoundRect(93,160,73,15,1,TFT_NAVY);
        //tft.fillRoundRect(87,160,8,15,1,TFT_RED);
        
        //tft.fillRoundRect(150,120,15,40,1,TFT_RED);



        porshe_light_speed_drw();

      }

      else {
        int16_t rc = png.openFLASH((uint8_t *)audi, sizeof(audi), pngDraw);
        if (rc == PNG_SUCCESS) {
          xpos = 20;
          ypos = 195;
          tft.startWrite();
          uint32_t dt = millis();
          rc = png.decode(NULL, 0);
          tft.endWrite();
          // png.close(); // not needed for memory->memory decode
        } else {
          //Failed to get image
          //Serial.println("Failed to get image");
        }
      }


}

void drawModelBattery()
{

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawBitmap(130, 0, bat_16x16, 16, 16, TFT_GREEN);
  tft.drawString(String(modelbattery)+"  ", 147, 0, 2);
}

void drawName()
{
      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.drawCentreString(NameHub, 85, 20, 4);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
}


void drawInfo()
{
static unsigned long disp_timeGyro=0;
static unsigned long disp_RSSI=0;
static unsigned long disp_modelBattery=0;

unsigned long tektime = millis();


    if ((tektime - disp_timeGyro) > 300) {
      disp_timeGyro = tektime;

      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("E:" + String(gyroX) + "        ", 35, 60, 4);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("FB:" + String(gyroY) + "    ", 35, 90, 4);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("LR:" + String(gyroZ) + "    ", 35, 120, 4);
    }

if ((tektime - disp_RSSI)>500)
{
  disp_RSSI = tektime;

tft.setTextColor(TFT_WHITE, TFT_BLACK);
tft.drawBitmap(0, 0, antena_16x16, 16, 16, TFT_WHITE, TFT_BLACK);


if ((RSSI) > -46) {
      tft.drawBitmap(18, 0, level4_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    else if (((RSSI) < -45) && (RSSI > -66)) {
      tft.drawBitmap(18, 0, level3_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    else if (((RSSI) < -65) && (RSSI > -76)) {
      tft.drawBitmap(18, 0, level2_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    else if (((RSSI) < -75) && (RSSI >= 85)) {
      tft.drawBitmap(18, 0, level1_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    if (RSSI < -85) {
      tft.drawBitmap(18, 0, level0_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }
}

if ((tektime - disp_modelBattery) > 300) {
      disp_modelBattery = tektime;
      drawModelBattery();
}
}