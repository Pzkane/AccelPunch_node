#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <LIS3DHTR.h>
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
LIS3DHTR<TwoWire> acceBag;

IPAddress local_IP(192, 168, 43, 3);
IPAddress gateway(192, 168, 43, 1);
IPAddress subnet(255, 255, 0, 0);

int server_port = 8267;
WiFiServer server(server_port);

void print_wifi_info() {
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());         // Send the IP address of the ESP8266 to the computer
  Serial.print("Gateway IP address:\t");
  Serial.println(WiFi.gatewayIP());         // Send the IP address of the ESP8266 to the computer
  Serial.print("Mask:\t");
  Serial.println(WiFi.subnetMask());         // Send the IP address of the ESP8266 to the computer
}

void setup() {
  digitalWrite(LED_BUILTIN, LOW);
  // |--------------Accel Config------------------
  wire.begin(D1, D2);
  acceBag.begin(wire,LIS3DHTR_ADDRESS_UPDATED);

  while(!acceBag){
     Serial.println("BAG: Communication failed, check the connection and I2C address setting when using I2C communication.");
     delay(1000);
  }
  
  Serial.print("chip id (LEFT): ");
  Serial.println(acceBag.getDeviceID(),HEX);
  // acceBag.openTemp();
  acceBag.setPowerMode(POWER_MODE_NORMAL);

  acceBag.setOutputDataRate(LIS3DHTR_DATARATE_400HZ);

  acceBag.setFullScaleRange(LIS3DHTR_RANGE_16G);
  // --------------Accel Config------------------|

  // |--------------Wifi Config-------------------
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("STA Failed to configure");
  }

  Serial.begin(9600);         // Start the Serial communication to send messages to the computer
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
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);

  // Setup done
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);
}

void print_accelerometers_to_client(WiFiClient& client) {
  wire.begin(D1, D2);
  client.print("BAG:");
  //Read the acceleration in the x direction
  client.print((int)acceBag.getAccelerationX()*1000);
  client.print(":");
  //Read the acceleration in the y direction
  client.print((int)acceBag.getAccelerationY()*1000);
  client.print(":");
  //Read the acceleration in the z direction
  client.print((int)acceBag.getAccelerationZ()*1000);
  client.print(":");
  //Read the temperature
  client.println(acceBag.getTemperature());
}

void loop() { 
  while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.println("no active connection");
    disconnected = true;
  }

  if (disconnected) {
    Serial.println("Connection established!");
    print_wifi_info();
    disconnected = false;
  }

  // Setup socket connection
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("Socket acquired from client");

    if(client.connected())
    {
      Serial.println("Client connected");
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
    }
    client.stop();
    Serial.println("Client disconnected");
  } else {
    Serial.println("Waiting for socket connection ...");
    delay(1000);
  }
}