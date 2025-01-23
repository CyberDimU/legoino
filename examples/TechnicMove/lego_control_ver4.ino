/**************************************************************************/

String NameHub;
byte VerHub;

#define light_pin 0
#define speed_pin 35

byte state_light;
byte state_speed;

byte ols = 1;
byte oss = 1;

byte oldspd = 0;
byte power_strlight = 0;

boolean FoundHUb = false;

#include <TFT_eSPI.h>  // Graphics and font library for ST7735 driver chip

#include <SPI.h>
#include <images.h>

// pinouts from https://github.com/Xinyuan-LilyGO/TTGO-T-Display
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23
#define TFT_BL 4

#define ADC_PIN 34
int vref = 1100;
float oldbattery_voltage = 0;

TFT_eSPI tft = TFT_eSPI(135, 240);  // Invoke library, pins defined in User_Setup.h

#include "Lpf2Hub.h"
#include <PNGdec.h>

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

unsigned long bat_time;
unsigned long disp_time;
unsigned long tick_str;

bool isInitialized = false;
int spd = 0;

byte portA = (byte)ControlPlusHubPort::A;
byte portB = (byte)ControlPlusHubPort::B;
byte portC = (byte)ControlPlusHubPort::C;
byte portD = (byte)ControlPlusHubPort::D;


void hubPropertyChangeCallback(void *hub, HubPropertyReference hubProperty, uint8_t *pData) {
  Lpf2Hub *myHub = (Lpf2Hub *)hub;

  //Serial.print("HubProperty: ");
  //Serial.println((byte)hubProperty, DEC);

  if (hubProperty == HubPropertyReference::RSSI) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawBitmap(0, 0, antena_16x16, 16, 16, TFT_WHITE, TFT_BLACK);

    int siglevel = int(myHub->parseRssi(pData));

    if ((siglevel) > -46) {
      tft.drawBitmap(18, 0, level4_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    else if (((siglevel) < -45) && (siglevel > -66)) {
      tft.drawBitmap(18, 0, level3_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    else if (((siglevel) < -65) && (siglevel > -76)) {
      tft.drawBitmap(18, 0, level2_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    else if (((siglevel) < -75) && (siglevel >= 85)) {
      tft.drawBitmap(18, 0, level1_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    if (int(myHub->parseRssi(pData)) < -85) {
      tft.drawBitmap(18, 0, level0_16x16, 16, 16, TFT_WHITE, TFT_BLACK);
    }

    return;
  }


  if (hubProperty == HubPropertyReference::BATTERY_VOLTAGE) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawBitmap(95, 0, bat_16x16, 16, 16, TFT_GREEN);
    tft.drawString(String(myHub->parseBatteryLevel(pData), DEC), 111, 0, 2);

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

  if (deviceType == DeviceType::TECHNIC_MEDIUM_HUB_TILT_SENSOR) {

    unsigned long tektime = millis();

    if ((tektime - disp_time) > 300) {
      disp_time = tektime;

      int x = myHub->parseControlPlusHubTiltSensorX(pData);
      int y = myHub->parseControlPlusHubTiltSensorY(pData);
      int z = myHub->parseControlPlusHubTiltSensorZ(pData);

      if (VerHub == 8)  //x<->z
      {
        x = x + z;
        z = x - z;
        x = x - z;
      }

      /* Serial.print("Tilt X: ");
    Serial.print(x, DEC);
    Serial.print(" Y: ");
    Serial.print(y, DEC);
    Serial.print(" Z: ");
    Serial.println(z, DEC);
    */

      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("E:" + String(x) + "   ", 35, 60, 4);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("FB:" + String(y) + "   ", 35, 90, 4);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);
      tft.drawString("LR:" + String(z) + "   ", 35, 120, 4);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(37, INPUT);
  pinMode(36, INPUT);

  pinMode(light_pin, INPUT_PULLUP);
  pinMode(speed_pin, INPUT_PULLUP);

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
  tft.drawCentreString("READY", 67, 20, 4);
  tft.drawCentreString("TO MOVE ", 67, 42, 4);

  tft.setTextColor(TFT_NAVY);
  tft.drawCentreString("Press BIND", 67, 82, 4);
  tft.drawCentreString("on the car", 67, 104, 4);

  tft.setTextColor(TFT_RED);
  tft.drawCentreString("And reset transmiter", 67, 144, 2);
  tft.drawCentreString("if needed...", 67, 160, 2);

  isInitialized = false;
  bat_time = 0;
  disp_time = 0;
  tick_str = 0;
  power_strlight = 0;
  myMoveHub.init();  // initalize the MoveHub instance
}

// main loop
void loop() {

  //Get the battery voltage in the remote control
  unsigned long new_bat_time = millis();
  if (((new_bat_time - bat_time) > 2000) || (bat_time == 0)) {
    bat_time = new_bat_time;
    uint16_t v = analogRead(ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    if (battery_voltage != oldbattery_voltage) {
      oldbattery_voltage = battery_voltage;
      if (battery_voltage > 4.30) {
        tft.drawBitmap(44, 0, bat_charge_16x16, 16, 16, TFT_RED, TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);
      } else {
        tft.drawBitmap(44, 0, bat_16x16, 16, 16, TFT_NAVY, TFT_BLACK);
        tft.setTextColor(TFT_NAVY, TFT_BLACK);
      }
      tft.drawString(String(battery_voltage), 62, 0, 2);
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

      NameHub = myMoveHub.getHubName().c_str();
      //Serial.println(NameHub);

      tft.setTextColor(TFT_BLUE, TFT_BLACK);
      tft.drawCentreString(NameHub, 67, 20, 2);
      tft.setTextColor(TFT_WHITE, TFT_BLACK);

      VerHub = byte(myMoveHub.getHubType());
      //Serial.println(VerHub);

      if (VerHub == 8) {
        delay(50);
        myMoveHub.calibrateTechnicMoveSteering();

        int16_t rc = png.openFLASH((uint8_t *)porshe, sizeof(porshe), pngDraw);
        if (rc == PNG_SUCCESS) {
          xpos = 0;
          ypos = 155;
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

        porshe_light_speed_drw();

      }

      else {
        int16_t rc = png.openFLASH((uint8_t *)audi, sizeof(audi), pngDraw);
        if (rc == PNG_SUCCESS) {
          xpos = 0;
          ypos = 155;
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
  int x = analogRead(37);  //Engine
  int y = analogRead(36);  //Steering

  /*Serial.print("X:");
  Serial.print(x);
  Serial.print("  Y:");
  Serial.println(y);
  delay(10);*/

  if (x > 1890) {
    int spd = map(x, 1890, 2770, 0, 100);
    if (spd > 100) {
      spd = 100;
    }
    myMoveHub.setBasicMotorSpeed(portA, spd);
    myMoveHub.setBasicMotorSpeed(portB, spd);
  } else if (x < 1880) {
    int spd = map(x, 1880, 1410, 0, -100);
    myMoveHub.setBasicMotorSpeed(portA, spd);
    myMoveHub.setBasicMotorSpeed(portB, spd);
  } else {
    myMoveHub.stopBasicMotor(portA);
    myMoveHub.stopBasicMotor(portB);
  }

  if (y > 1880)  //Влево
  {
    int spds = map(y, 1880, 4095, 0, -100);
    myMoveHub.setAbsoluteMotorPosition(portD, 100, spds);
  } else if (y < 1875) {
    int spds = map(y, 1875, 0, 0, 100);
    myMoveHub.setAbsoluteMotorPosition(portD, 100, spds);
  } else {
    //Установить в 0
    myMoveHub.setAbsoluteMotorPosition(portD, 100, 0);
  }
}

void NewHubMove() {
  int x = analogRead(37);  //Engine
  int y = analogRead(36);  //Steering

  /*Serial.print("X:");
 Serial.print(x);
 Serial.print("  Y:");
 Serial.println(y);
  delay(10);*/

  byte lightclick = digitalRead(light_pin);
  byte speedclick = digitalRead(speed_pin);
  byte flag = 0;

  if ((lightclick == 0) && (ols == 1)) {
    //Serial.println("lightclick");
    state_light = state_light ^ 4;  //0000 0100
    flag = 1;
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

  if (x > 1890) {
    spd = map(x, 1891, 2770, 0, 127);
    if (spd > 127) { spd = 127; }
  } else if (x < 1880) {
    spd = map(x, 1880, 1410, 255, 128);
  } else {
    spd = 0;
  }

  if (y > 1930)  //Влево
  {
    left_blink();
    spds = map(y, 1930, 4095, 255, 128);
  } else if (y < 1870) {
    right_blink();
    spds = map(y, 1875, 0, 0, 127);

  } else {
    spds = 0;
    myMoveHub.setTechnicLed(B001001, 0);
  }

  /*Serial.print("spd: ");
    Serial.println(spd);
    Serial.print("spds: ");
    Serial.println(spds);
    */

  light = (state_light ^ state_speed);

  if ((x < 1955) && (x > 1835)) {
    estop = 1;  //stop
  } else {
    estop = 0;
  }

  light = light | estop;

  myMoveHub.setTechicMoveMotorSpeed(spd, spds, light);
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
  xpos_speed = 104;
  ypos_speed = 205;

  xpos_light = 0;
  ypos_light = 150;

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