#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x08, 0xB6, 0x1F, 0x3D, 0x05, 0xE0};

// Structure example to send data
// Must match the receiver structure
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

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Init Serial Monitor
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
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
double tans(double a, double b)
{
  if (a >= 0 && b >= 0)
    return atan(a / b);
  else if (a < 0 && b >= 0)
    return 2 * M_PI + atan(a / b);
  else return M_PI + atan(a / b);
}
void loop() {
  // Set values to send
      if (digitalRead(but_mode[0]) == 0)
      {
        sendControls();
      }
      else if (digitalRead(but_mode[1]) == 0)
      {
        joystick();
      }
  
  // Send message via ESP-NOW
//  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
//   
//  if (result == ESP_OK) {
//    Serial.println("Sent with success");
//  }
//  else {
//    Serial.println("Error sending the data");
//  }
//  delay(2000);
}
void sendControls() {
  //const uint8_t *peer_addr = slave.peer_addr;
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
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &but_values, sizeof(but_values));

  if (result == ESP_OK) {
    //  Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

}

void joystick()
{
//  const uint8_t *peer_addr = slave.peer_addr;
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
//  Serial.println(vel_x);
//  Serial.println(vel_y);
  Serial.println(joy_values.con_mode);
//  Serial.println(joy_values.w0);
//  Serial.println(joy_values.w1);
//  Serial.println(joy_values.w2);
//  Serial.println(joy_values.w3);
  Serial.println(joy_values.buton);
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joy_values, sizeof(joy_values));
  if (result == ESP_OK)
  {
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }
}
