#include <esp_now.h>
#include <WiFi.h>
#define CHANNEL 1
/*Motor driver initialization */
int dir[4] = {26, 19, 22, 33};
int pwm[4] = {25, 21, 23, 32};
int speedd = 50;
int i, j, k;
/* Initializing the encoder */
//int inputA = 36;
//int inputB = 39;
int dire[2] = {5, 2};
int pwme[2] = {4, 15};
//String encoder_dir="";
//String enter="0";
//float counter = 0;
//float encoder_counts = 7181.00;
//float distance_moved=0.0, ratio=0.0;
//double dist = 0;
/* Initializing the structure to recieve  it to the  ESP-32 using espnow protocol*/
int con = 1123;
float butoon;
typedef struct message {
  float buton;
  float w0;
  float w1;
  float w2;
  float w3;
  float con_mode;
} message;
message but;
int dirs[4] = {false, true, true, true};
float vel[4] = {0, 0, 0, 0};
// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result)
  {
    Serial.println("AP Config failed.");
  }
  else
  {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
  for (i = 0; i < 4; i++)
  {
    pinMode(dir[i], OUTPUT);
  }
  for (i = 0; i < 4; i++)
  {
    pinMode(pwm[i], OUTPUT);
  }
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  /*Setting up the coniguration of Encoders */
  pinMode(inputA, INPUT_PULLUP);
  pinMode(inputB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(inputA), record_data1, RISING);
  attachInterrupt(digitalPinToInterrupt(inputB), record_data2, RISING);
  pinMode(dire[0], OUTPUT);
  pinMode(dire[1], OUTPUT);
  pinMode(pwme[0], OUTPUT);
  pinMode(pwme[1], OUTPUT);
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  memcpy(&but, data, sizeof(but));
  Serial.print("Byte Recieved: ");
  Serial.println(data_len);
  Serial.println(but.con_mode);
  butoon = but.buton;
  con = but.con_mode;
  vel[0] = but.w0;
  vel[1] = but.w1;
  vel[2] = but.w2;
  vel[3] = but.w3;
  Serial.println(con);
  Serial.println(vel[0]);
  Serial.println(vel[1]);
  Serial.println(vel[2]);
  Serial.println(vel[3]);
  Serial.println(butoon);

}

void loop() {
  // Chill
  if (con == 0) {
    if (butoon == 1) {
      forward();
    }
    else if (butoon == 2) {
      back();
    }
    else if (butoon == 3) {
      left();
    }
    else if (butoon == 4) {
      right();
    }
    else if (butoon == 5) {
      left_rotate();
    }
    else if (butoon == 6) {
      right_rotate();
    }
    else {
      stops();
    }
  }
  else if (con == 1)
  {
    if (butoon == 1)
    {
      // Write the code
      for (int i = 0; i < 2; i++)
      {
        digitalWrite(dire[i], HIGH);
        digitalWrite(pwme[i], 20);
        delay(500);
      }

    }
    else if (butoon == 2)
    {
      // write the code
      digitalWrite(dire[i], LOW);
      digitalWrite(pwme[i], 20);
      delay(500);
    }
    else {

      joy();
    }
  }
}
void forward()
{

  for (i = 0; i < 4; i++)
  {
    digitalWrite(dir[i], LOW);
    analogWrite(pwm[i], speedd);
  }
  digitalWrite(dir[3], HIGH);
  Serial.println("Forward");
}
void back()
{
  for (i = 0; i < 4; i++)
  {
    digitalWrite(dir[i], HIGH);
    analogWrite(pwm[i], speedd);
  }
  digitalWrite(dir[3], LOW);

  Serial.println("Back");
}
void left()
{
  digitalWrite(dir[0], HIGH);
  digitalWrite(dir[1], LOW);
  digitalWrite(dir[2], HIGH);
  digitalWrite(dir[3], HIGH);
  for (i = 0; i < 4; i++)
  {
    analogWrite(pwm[i], speedd);
  }

  Serial.println("Left");
}
void right()
{
  digitalWrite(dir[0], LOW);
  digitalWrite(dir[1], HIGH);
  digitalWrite(dir[2], LOW);
  digitalWrite(dir[3], LOW);
  for (i = 0; i < 4; i++)
  {
    analogWrite(pwm[i], speedd);
  }

  Serial.println("Right");
}
void left_rotate()
{
  digitalWrite(dir[0], LOW);
  digitalWrite(dir[1], HIGH);
  digitalWrite(dir[2], HIGH);
  digitalWrite(dir[3], HIGH);
  for (i = 0; i < 4; i++)
  {
    analogWrite(pwm[i], 50);
  }
}
void right_rotate()
{
  digitalWrite(dir[0], HIGH);
  digitalWrite(dir[1], LOW);
  digitalWrite(dir[2], LOW);
  digitalWrite(dir[3], LOW);
  for (i = 0; i < 4; i++)
  {
    analogWrite(pwm[i], 50);
  }
}
void stops()
{
  for (i = 0; i < 4; i++)
  {
    analogWrite(pwm[i], 0);
  }
}

void joy() {

  int directions[4] = {false, false, false, false};
  for (i = 0; i < 4; i++)
  {
    directions[i] = dirs[i];
    if (vel[i] < 0)
    { Serial.println("here");
      vel[i] *= -1;
      directions[i] = !dirs[i];
      // Serial.println(directions[i]);
    }
  }

  for (i = 0; i < 4; i++)
  {
    digitalWrite(dir[i], directions[i]);
    analogWrite(pwm[i], vel[i]);
    //Serial.println(vel[i]);
    // Serial.println(dir[i]);
    //Serial.println(directions[i]);
  }
  Serial.println("-------------------");
  delay(700);
}
}
