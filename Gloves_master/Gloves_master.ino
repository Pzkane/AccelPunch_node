/*
  Accelerometer readings for both hands (acceleration + tilt angle [no yaw])
  Uses parts of modified code from Carbon Aeronautics [https://github.com/CarbonAeronautics/Part-XV-1DKalmanFilter/blob/main/ArduinoCode] for
    angle and Kalman filter calculations.
*/

#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <DFRobot_LIS2DW12.h>
#include <Wire.h>

const char* ssid     = "AccelPunch-AP";         // The SSID (name) of the Wi-Fi network you want to connect to
const char* password = "accelpunch";     // The password of the Wi-Fi network

bool disconnected = false;

//When using I2C communication, use the following program to construct an object by DFRobot_LIS2DW12_I2C
/*!
 * @brief Constructor 
 * @param pWire I2c controller
 * @param addr  I2C address(0x18/0x19)
 */
TwoWire wire;
DFRobot_LIS2DW12_I2C acceL(&wire,0x18);
DFRobot_LIS2DW12_I2C acceR(&wire,0x18);

IPAddress local_IP(192, 168, 43, 2);
IPAddress gateway(192, 168, 43, 1);
IPAddress subnet(255, 255, 0, 0);

int server_port = 8266;
WiFiServer server(server_port);
WiFiClient client;

void print_wifi_info() {
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
  Serial.print("Gateway IP address:\t");
  Serial.println(WiFi.gatewayIP());         // Send the IP address of the ESP8266 to the computer
  Serial.print("Mask:\t");
  Serial.println(WiFi.subnetMask());         // Send the IP address of the ESP8266 to the computer
}

uint32_t LoopTimer;
float RateRollL, RatePitchL, RateYawL, RateRollR, RatePitchR, RateYawR;
float RateCalibrationRollL, RateCalibrationPitchL, RateCalibrationYawL, RateCalibrationRollR, RateCalibrationPitchR, RateCalibrationYawR;
int RateCalibrationNumber;
float KalmanAngleRollL=0, KalmanAngleRollR=0, KalmanUncertaintyAngleRollL=2*2, KalmanUncertaintyAngleRollR=2*2,
      KalmanAnglePitchL=0, KalmanAnglePitchR=0, KalmanUncertaintyAnglePitchL=2*2, KalmanUncertaintyAnglePitchR=2*2,
      Kalman1DOutputL[]={0,0},
      Kalman1DOutputR[]={0,0};
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement, float* Kalman1DOutput) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}

void print_accelerometers_to_client(WiFiClient& client, bool dry_run = false) {
  wire.begin(D1, D2);
  int16_t accLX = acceL.readAccX(),
          accLY = acceL.readAccY(),
          accLZ = acceL.readAccZ();
  wire.begin(D5, D6);
  int16_t accRX = acceR.readAccX(),
          accRY = acceR.readAccY(),
          accRZ = acceR.readAccZ();
  float AngleRollL = atan(accLY / sqrt(accLX * accLX + accLZ * accLZ)) * 1 / (3.142/180),
        AnglePitchL = -atan(accLX / sqrt(accLY * accLY + accLZ * accLZ)) * 1 / (3.142/180),
        AngleRollR = atan(accRY / sqrt(accRX * accRX + accRZ * accRZ)) * 1 / (3.142/180),
        AnglePitchR = -atan(accRX / sqrt(accRY * accRY + accRZ * accRZ)) * 1 / (3.142/180);

  // Kalman filter calibration
  // Left Hand
  RateRollL-=RateCalibrationRollL;
  RatePitchL-=RateCalibrationPitchL;
  RateYawL-=RateCalibrationYawL;
  kalman_1d(KalmanAngleRollL, KalmanUncertaintyAngleRollL, RateRollL, AngleRollL, Kalman1DOutputL);
  KalmanAngleRollL=Kalman1DOutputL[0]; 
  KalmanUncertaintyAngleRollL=Kalman1DOutputL[1];
  kalman_1d(KalmanAnglePitchL, KalmanUncertaintyAnglePitchL, RatePitchL, AnglePitchL, Kalman1DOutputL);
  KalmanAnglePitchL=Kalman1DOutputL[0]; 
  KalmanUncertaintyAnglePitchL=Kalman1DOutputL[1];

  // Right Hand
  RateRollR-=RateCalibrationRollR;
  RatePitchR-=RateCalibrationPitchR;
  RateYawR-=RateCalibrationYawR;
  kalman_1d(KalmanAngleRollR, KalmanUncertaintyAngleRollR, RateRollR, AngleRollR, Kalman1DOutputR);
  KalmanAngleRollR=Kalman1DOutputR[0]; 
  KalmanUncertaintyAngleRollR=Kalman1DOutputR[1];
  kalman_1d(KalmanAnglePitchR, KalmanUncertaintyAnglePitchR, RatePitchR, AnglePitchR, Kalman1DOutputR);
  KalmanAnglePitchR=Kalman1DOutputR[0]; 
  KalmanUncertaintyAnglePitchR=Kalman1DOutputR[1];
  
  if (dry_run) return;

  client.print("GLOVES:");
  // client.print("xL: ");
  //Read the acceleration in the x direction
  client.print(accLX);
  client.print(":");
  // client.print(" mg \tyL: ");
  //Read the acceleration in the y direction
  client.print(accLY);
  client.print(":");
  // client.print(" mg \tzL: ");
  //Read the acceleration in the z direction
  client.print(accLZ);
  
  client.print(":");
  // client.print(" mg \txR: ");
  //Read the acceleration in the x direction
  client.print(accRX);
  client.print(":");
  // client.print(" mg \tyR: ");
  //Read the acceleration in the y direction
  client.print(accRY);
  client.print(":");
  // client.print(" mg \tzR: ");
  //Read the acceleration in the z direction
  client.print(accRZ);
  // client.println(" mg");

  client.print(":"); client.print(AngleRollL); client.print(":"); client.print(AnglePitchL);
  client.print(":"); client.print(AngleRollR); client.print(":"); client.println(AnglePitchR);
}

void setup() {
  // |-------------Accel Config------------------
  Serial.begin(9600);         // Start the Serial communication to send messages to the computer
  wire.begin(D1, D2);

  while(!acceL.begin()){
     Serial.println("LEFT: Communication failed, check the connection and I2C address setting when using I2C communication.");
     delay(1000);
  }
  
  Serial.print("chip id (LEFT): ");
  Serial.println(acceL.getID(),HEX);

  //Chip soft reset
  acceL.softReset();
  //Set whether to collect data continuously
  acceL.continRefresh(true);
  
  /**！
    Set the sensor data collection rate:
               eRate_0hz           /<Measurement off>/
               eRate_1hz6          /<1.6hz, use only under low-power mode>/
               eRate_12hz5         /<12.5hz>/
               eRate_25hz          
               eRate_50hz          
               eRate_100hz         
               eRate_200hz         
               eRate_400hz       /<Use only under High-Performance mode>/
               eRate_800hz       /<Use only under High-Performance mode>/
               eRate_1k6hz       /<Use only under High-Performance mode>/
               eSetSwTrig        /<The software triggers a single measurement>/
  */
  acceL.setDataRate(DFRobot_LIS2DW12::eRate_800hz);
  
  /**！
    Set the sensor measurement range:
                   e2_g   /<±2g>/
                   e4_g   /<±4g>/
                   e8_g   /<±8g>/
                   e16_g  /< ±16g>/
  */
  acceL.setRange(DFRobot_LIS2DW12::e16_g);
  
  /**！
    Filter settings:
           eLPF (Low pass filter)
           eHPF (High pass filter)
  */
  acceL.setFilterPath(DFRobot_LIS2DW12::eLPF);
  /**！
    Set bandwidth：
        eRateDiv_2  /<Rate/2 (up to Rate = 800 Hz, 400 Hz when Rate = 1600 Hz)>/
        eRateDiv_4  /<Rate/4 (High Power/Low power)>*
        eRateDiv_10 /<Rate/10 (HP/LP)>/
        eRateDiv_20 /< Rate/20 (HP/LP)>/
  */
  acceL.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  /**！
   Set power mode:
       eHighPerformance_14bit         /<High-Performance Mode,14-bit resolution>/
       eContLowPwr4_14bit             /<Continuous measurement,Low-Power Mode 4(14-bit resolution)>/
       eContLowPwr3_14bit             /<Continuous measurement,Low-Power Mode 3(14-bit resolution)>/
       eContLowPwr2_14bit             /<Continuous measurement,Low-Power Mode 2(14-bit resolution)/
       eContLowPwr1_12bit             /<Continuous measurement,Low-Power Mode 1(12-bit resolution)>/
       eSingleLowPwr4_14bit           /<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution)>/
       eSingleLowPwr3_14bit           /<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution)>/
       eSingleLowPwr2_14bit           /<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution)>/
       eSingleLowPwr1_12bit           /<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution)>/
       eHighPerformanceLowNoise_14bit /<High-Performance Mode,Low-noise enabled,14-bit resolution>/
       eContLowPwrLowNoise4_14bit     /<Continuous measurement,Low-Power Mode 4(14-bit resolution,Low-noise enabled)>/
       eContLowPwrLowNoise3_14bit     /<Continuous measurement,Low-Power Mode 3(14-bit resolution,Low-noise enabled)>/
       eContLowPwrLowNoise2_14bit     /<Continuous measurement,Low-Power Mode 2(14-bit resolution,Low-noise enabled)>/
       eContLowPwrLowNoise1_12bit     /<Continuous measurement,Low-Power Mode 1(12-bit resolution,Low-noise enabled)>/
       eSingleLowPwrLowNoise4_14bit   /<Single data conversion on demand mode,Low-Power Mode 4(14-bit resolution),Low-noise enabled>/
       eSingleLowPwrLowNoise3_14bit   /<Single data conversion on demand mode,Low-Power Mode 3(14-bit resolution),Low-noise enabled>/
       eSingleLowPwrLowNoise2_14bit   /<Single data conversion on demand mode,Low-Power Mode 2(14-bit resolution),Low-noise enabled>/
       eSingleLowPwrLowNoise1_12bit   /<Single data conversion on demand mode,Low-Power Mode 1(12-bit resolution),Low-noise enabled>/
  */
  acceL.setPowerMode(DFRobot_LIS2DW12::eHighPerformanceLowNoise_14bit);

  // Transition to other 2 pins
  wire.begin(D5, D6);
  while(!acceR.begin()){
     Serial.println("RIGHT: Communication failed, check the connection and I2C address setting when using I2C communication.");
     delay(1000);
  }
  Serial.print("chip id (RIGHT): ");
  Serial.println(acceR.getID(),HEX);
  acceR.softReset();
  acceR.continRefresh(true);
  acceR.setDataRate(DFRobot_LIS2DW12::eRate_800hz);
  acceR.setRange(DFRobot_LIS2DW12::e16_g);
  acceR.setFilterPath(DFRobot_LIS2DW12::eLPF);
  acceR.setFilterBandwidth(DFRobot_LIS2DW12::eRateDiv_4);
  acceR.setPowerMode(DFRobot_LIS2DW12::eHighPerformanceLowNoise_14bit);
  // --------------Accel Config-----------------|

  // |--------------Kalman state config----------
  // Serial.println("Calibrating Kalman tilt ...");
  // short iters = 2000;
  // for (RateCalibrationNumber=0; RateCalibrationNumber<iters; ++RateCalibrationNumber) {
  //   print_accelerometers_to_client(client, true);
  //   RateCalibrationRollL+=RateRollL;
  //   RateCalibrationPitchL+=RatePitchL;
  //   RateCalibrationYawL+=RateYawL;
  //   RateCalibrationRollR+=RateRollR;
  //   RateCalibrationPitchR+=RatePitchR;
  //   RateCalibrationYawR+=RateYawR;
  //   delay(1);
  // }
  // RateCalibrationRollL/=iters;
  // RateCalibrationPitchL/=iters;
  // RateCalibrationYawL/=iters;
  // RateCalibrationRollR/=iters;
  // RateCalibrationPitchR/=iters;
  // RateCalibrationYawR/=iters;
  // Serial.println("Calibration done!");
  // --------------Kalman state config----------|

  // |--------------Wifi Config------------------
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  Serial.println('\n');
  
  WiFi.begin(ssid, password);             // Connect to the network
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
  }

  Serial.println("Connection established!");  
  print_wifi_info();

  server.begin();
  // --------------Wifi Config-------------------|

  // LoopTimer=micros();
}



void loop() { 
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.println("No active connection.");
    disconnected = true;
  }

  if (disconnected) {
    Serial.println("Connection established!");
    print_wifi_info();
    disconnected = false;
  }

  // Setup socket connection
  client = server.available();
  
  if (client) {
    Serial.println("Socket acquired from client");

    if(client.connected())
    {
      Serial.println("Client connected.");
    }
    
    while(client.connected()){     
      while(client.available() > 0){
        // Read data from the connected client
        Serial.write(client.read()); 
      }
      print_accelerometers_to_client(client);
      // Send Data to connected client
      while(Serial.available() > 0)
      {
        client.write(Serial.read());
      }
      delay(50);
      // while (micros() - LoopTimer < 8000);
      // LoopTimer=micros();
    }
    client.stop();
    Serial.println("Client disconnected.");
  }
}