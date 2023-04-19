#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>  // only for esp_wifi_set_channel()
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

esp_now_peer_info_t slave;

#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

void ScanForSlave() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);  // Scan only on one channel
  // reset on each scan
  bool slaveFound = 0;
  memset(&slave, 0, sizeof(slave));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found ");
    Serial.print(scanResults);
    Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        Serial.println("Found a Slave.");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" [");
        Serial.print(BSSIDstr);
        Serial.print("]");
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];
        if (6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x", &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])) {
          for (int ii = 0; ii < 6; ++ii) {
            slave.peer_addr[ii] = (uint8_t)mac[ii];
          }
        }

        slave.channel = CHANNEL;  // pick a channel
        slave.encrypt = 0;        // no encryption

        slaveFound = 1;
        // we are planning to have only one slave in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (slaveFound) {
    Serial.println("Slave Found, processing..");
  } else {
    Serial.println("Slave Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

bool manageSlave() {
  if (slave.channel == CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("Slave Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(slave.peer_addr);
    if (exists) {
      // Slave already paired.
      //    Serial.println("Already Paired");
      return true;
    } else {
      // Slave not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&slave);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No slave found to process
    Serial.println("No Slave found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(slave.peer_addr);
  Serial.print("Slave Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  // Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

//int modePin[2] = { 19, 18 };
int button[7] = { 32, 33, 25, 26, 27, 14, 12 };

//double x = 0, y = 0;
//int vel[4];
//int prevVel[4] = {0, 0, 0, 0};
//int max_speed=50;
//int button_speed = 50;
//double angle;

typedef struct message {
  //int mode;
  //int pwm[4];
  int button;
};
message send;

void setup() {
  Serial.begin(115200);
  delay(500);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL ");
  Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  for (int i = 0; i < 7; i++)
    pinMode(button[i], INPUT_PULLUP);

  //for (int i = 0; i < 2; i++)
    //pinMode(modePin[i], INPUT_PULLUP);
}
/*
double tans(double a, double b) {
  if(b<0) return M_PI+atan(a/b);
  else return 2*M_PI + atan(a/b);
}

void leftRotate() {
  vel[0] = button_speed;
  vel[1] = -1*button_speed;
  vel[2] = button_speed;
  vel[3] = -1*button_speed;
}

void rightRotate() {
  vel[0] = -1*button_speed;
  vel[1] = button_speed;
  vel[2] = -1*button_speed;
  vel[3] = button_speed;
}

void calculateVelocity() {

   int wheel_angle[4] = { 315, 45, 135, 225 };

  int joyX, joyY;

  joyX = analogRead(39);
  joyY = analogRead(36);

  x = map(joyX, 0, 4095, -100, 100) + 11*(100-abs(x))/100;
  y = map(joyY, 0, 4095, 100, -100) - 10*(100-abs(y))/100;

  Serial.print(x);
  Serial.print(" ");  
  Serial.print(y);
  Serial.print(" ");  
  
  angle = tans(x, y);
Serial.print(angle*180/3.14);
  Serial.print(" ");
  double distance = sqrt(x * x + y * y);
  double speed = map(distance, 0, 141.4, 0, max_speed);

  if (send.mode == 1) {
    switch (send.button) {
      case 0: angle = 3*M_PI/2; speed = button_speed; break;
      case 1: angle = M_PI; speed = button_speed; break;
      case 2: angle = 0; speed = button_speed; break;
      case 3: angle = M_PI/2; speed = button_speed; break;
    }
  }

  double vel_x = speed * cos(angle);
  double vel_y = speed * sin(angle);

  for (int i = 0; i < 4; i++)
    vel[i] = vel_x * cos(wheel_angle[i] * M_PI / 180) + vel_y * sin(wheel_angle[i] * M_PI / 180);

    switch (send.button) {
      case 4: rightRotate(); break;
      case 5: leftRotate(); break;
    }

  for(int i=0; i<4; i++)
    if(vel[i]>100)
      vel[i] = prevVel[i];
  

  for (int i = 0; i < 4; i++)
    send.pwm[i] = vel[i];

    for (int i = 0; i < 4; i++)
    prevVel[i] = vel[i];
}*/

void loop() {

  ScanForSlave();

  if (slave.channel == CHANNEL) {
    bool isPaired = manageSlave();
    if (isPaired) {
      const uint8_t *peer_addr = slave.peer_addr;
      
      send.button = -1;
     /* for (int i = 0; i < 2; i++)
        if (digitalRead(modePin[i]) == LOW)
          send.mode = i;
*/
      for (int i = 0; i < 7; i++){
        if (digitalRead(button[i]) == LOW)
          {send.button = i;}}
          
            Serial.println(send.button);
          

 /*     //calculateVelocity();
Serial.println("");
Serial.print(send.mode);
Serial.print("  ");
Serial.print(send.button);
Serial.print("  ");
for(int i=0; i<4; i++)
{
  Serial.print(send.pwm[i]);
  Serial.print("  ");
}*/
      esp_err_t result = esp_now_send(peer_addr, (uint8_t *)&send, sizeof(send));
    }
  } else {
    // slave pair failed
    Serial.println("Slave pair failed!");
  }
  
}
