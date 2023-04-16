/**
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Master >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the Serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/


#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#include<stdlib.h>
#include<stdio.h>
#include<math.h>
// Global copy of slave
esp_now_peer_info_t slave;
#define M_PI 3.14
#define CHANNEL 2
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
typedef struct message {
  float buton;
  float w0;
  float w1;
  float w2;
  float w3;
  float con_mode;
} message;
message but_values;
message joy_values;
float speedd = 100;
int wheel_angles[4] = {135, 225, 315, 45};
int i, j, k;
int buttons[7] = {25, 33, 32, 26, 14, 27,12};
int but_mode[2] = {18, 19};
float vel_x, vel_y;
int pot_x =  36;
int pot_y = 39;
// Init ESP Now with fallback
float vel[4] = {0, 0, 0, 0};
float offset_x = -24;
float offset_y = -20;

float x = 0, y = 0;
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    /* A additional feature that could have been added was to print
      the esp initialization has failed and it needs to restart again with a count*/
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave()
{
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL); // Scan only on one channel
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));
  if (scanResults == 0)
  {
    Serial.println("No WiFi devices in AP Mode found");
  }
  else
  {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS)
      {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
     // delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0)
      {
        //SSID of interest
    //    Serial.println("Found a Slave.");
    //    Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        //Get BSSID => Mac Address of the Slave
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) )
        {
          for (int ii = 0; ii < 6; ++ii )
          {
            slave.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slave.channel = CHANNEL; // pick a channel
        slave.encrypt = 0; // no encryption
        slaveFound = 1;
        break;
      }
    }
  }

  if (slaveFound)
  {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if ( exists) {
      //Slave already paired.
      Serial.println("Already Paired");
      return true;
    }
    else
    {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK)
      {
        // Pair success
        Serial.println("Pair success");
        return true;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_ARG)
      {
        Serial.println("Invalid Argument");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_FULL)
      {
        Serial.println("Peer list full");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_NO_MEM)
      {
        Serial.println("Out of memory");
        return false;
      }
      else if (addStatus == ESP_ERR_ESPNOW_EXIST)
      {
        Serial.println("Peer Exists");
        return true;
      }
      else
      {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  }
  else
  {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer()
{
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK)
  {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT)
  {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  }
  else if (delStatus == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
double tans(double a, double b)
{
  if (a >= 0 && b >= 0)
    return atan(a / b);
  else if (a < 0 && b >= 0)
    return 2 * M_PI + atan(a / b);
  else return M_PI + atan(a / b);
}
void setup() {
  Serial.begin(115200);

  for (i = 0; i < 7; i++)
  {
    pinMode(buttons[i], INPUT_PULLUP);
  }
  for (i = 0; i < 2; i++)
  {
    pinMode(but_mode[i], INPUT_PULLUP);
  }
  pinMode(36, INPUT);
  pinMode(39, INPUT);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (slave.channel == CHANNEL)
  { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    bool isPaired = manageSlave();
    if (isPaired)
    {
      if (digitalRead(but_mode[0]) == 0)
      {
        sendControls();
      }
      else if (digitalRead(but_mode[1]) == 0)
      {
        joystick();
      }
    }
    else
    {
      // slave pair failed
      Serial.println("Slave pair failed!");
    }
  }
  else {
    // No slave found to process
  }

  // wait for 3seconds to run the logic again
  delay(50);
}
void sendControls() {
  const uint8_t *peer_addr = slave.peer_addr;
  but_values.con_mode = 0;
  for (i = 0; i < 7; i++) {
    int val = digitalRead(buttons[i]);
    Serial.println(digitalRead(buttons[i]));

    if (val == 0) {
      but_values.buton = i + 1;
      break;
    }
    else {
      but_values.buton = 100;
    }
  }
  Serial.println(but_values.buton);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &but_values, sizeof(but_values));

  if (result == ESP_OK) {
    //  Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

}

void joystick()
{
  const uint8_t *peer_addr = slave.peer_addr;
  Serial.println("inside the joystik");
  for (i = 0; i < 7; i++) {
    int val = digitalRead(buttons[i]);
   Serial.println(digitalRead(buttons[i]));

    if (val == 0)
    {
      joy_values.buton = i + 1;
      break;
    }
    else
    {
      joy_values.buton = -1;
    }
  }
//  double jx = analogRead(39);
//  double jy = analogRead(36);
//  x = map(jx, 0, 4095, -250, 250) - offset_x;//+3*(100-abs(x))/100;
//  y = map(jy, 0, 4095, -250, 250) - offset_y;//-1*(100-abs(y))/100;
//
//  Serial.print("X:coordinate ");
//  Serial.print(x);
//  Serial.print("    ");
//   Serial.print("Y:coordinate ");
//  Serial.print(y);
//  Serial.print("    ");
//  double dist = sqrt(x * x + y * y);
//  speedd = map(dist, 0, 141.4, 0, 100);
//  double ang = tans(y, x) + M_PI / 2;
//  ang  = (ang * 180) / M_PI;
//
//  if (ang >= 350 && ang <= 370)
//  {
//    ang = 360;
//  }
//  if (ang >= 170 && ang <= 200)
//  {
//    ang = 180;
//  }
//  if (ang >= 260 && ang <= 280)
//  {
//    ang = 270;
//  }
//  if (ang >= 360)
//  {
//    ang = ang - 360;
//  }
//  Serial.print("Angle : ");
//  Serial.println(ang);
//  double vel_x = speedd * sin((ang * M_PI) / 180);
//  double vel_y = speedd * cos((ang * M_PI) / 180);
//  Serial.println(vel_x);
//  Serial.println(vel_y);
//  for (int i = 0; i < 4; i++)
//  {
//    vel[i] = vel_x * cos(wheel_angles[i] * M_PI / 180) + vel_y * sin(wheel_angles[i] * M_PI / 180);
//    vel[i] = constrain(vel[i], -50, 50);
//    if (vel[i] <= 20 && vel[i] >= -20) {
//      vel[i] = 0;
//    }
//  }
for(int i = 0;i<4;i++){
  vel[i] = 0;
}
  joy_values.w0 = vel[0];
  joy_values.w1 = vel[1];
  joy_values.w2 = vel[2];
  joy_values.w3 = vel[3];
  joy_values.con_mode = 1;
  for (int i = 0; i < 4; i++)
  {
    Serial.print(vel[i]);
    Serial.print("    ");
  }
  Serial.println("");
  Serial.println(vel_x);
  Serial.println(vel_y);
  Serial.println(joy_values.con_mode);
  Serial.println(joy_values.w0);
  Serial.println(joy_values.w1);
  Serial.println(joy_values.w2);
  Serial.println(joy_values.w3);
  Serial.println(joy_values.buton);
  esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &joy_values, sizeof(joy_values));
  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
}
