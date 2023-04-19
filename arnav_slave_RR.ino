#include <esp_now.h>
#include <WiFi.h>
#define CHANNEL 1
#include<Servo.h>

Servo s;

void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave_1";
  bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL ");
    Serial.println(WiFi.channel());
  }
}

//int dir[4] = { 26, 19, 22, 33 };
//int pwm[4] = { 25, 21, 23, 32 };

typedef struct message {
  //int mode;
  //int pwm[4];
  int button;
};
message recieved;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  memcpy(&recieved, data, sizeof(recieved));
  Serial.print("Byte Recieved: ");
  Serial.println(data_len);
}

void setup() {
  s.attach(19);
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");

  /*for (int i = 0; i < 4; i++)
    pinMode(dir[i], OUTPUT);

  for (int i = 0; i < 4; i++)
    pinMode(pwm[i], OUTPUT);*/

  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: ");
  Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
}
/*
void moveBot() {

  int dirs[4] = { false, false, false, true };
  int vel[4];
  int directions[4];

  for (int i = 0; i < 4; i++) {
    if (recieved.pwm[i] < 0) {
      vel[i] = -recieved.pwm[i];
      directions[i] = !dirs[i];
    } else {
      vel[i] = recieved.pwm[i];
      directions[i] = dirs[i];
    }
    Serial.print(vel[i]);
    Serial.print("  ");
  }
  Serial.println("");

  for (int i = 0; i < 4; i++) {
    digitalWrite(dir[i], directions[i]);
    analogWrite(pwm[i], vel[i]);
  }
}*/

void leadscrew(int butoon)
{
  int d_pin = 26;
  int p_pin = 25;
  int speed = 1;
  if (butoon == 1)
  {
    // Write the code

      digitalWrite(26, HIGH);
      analogWrite(25, 100);
      Serial.println("forward");
      
   delay(100);       

        digitalWrite(26, LOW);
        analogWrite(25, 0);
        Serial.println("stop");
    
  }
  else if (butoon == 0)
  {
    
      digitalWrite(26, LOW);
      analogWrite(25, 100);
      Serial.println("backwards");
    
    delay(200);

        digitalWrite(26, LOW);
        analogWrite(25, 0);
        Serial.println("stop");
      
  }
}

void shoot(int op)
{
 /* int d_pin[2] = {1,2};
  int p_pin[2] = {3,4};
  int speed[2] = {127, 175};

  for(int i=0; i<2; i++)
    {
      digitalWrite(d_pin[i], HIGH);
      analogWrite(p_pin[i], speed[op]);
    }*/
}

void servo_0()
{
  s.write(180);
  Serial.println("0");
}

void servo_90()
{
  Serial.println("90");
  s.write(75);
}
void loop() {

  //moveBot();
  //if(recieved.mode==0)
  switch(recieved.button)
  {
    case 0: shoot(0); Serial.println("Shoooting close range"); break;
    case 1: leadscrew(0); Serial.println("Lead Screw going down"); break;
    case 2: leadscrew(1); Serial.println("Lead Screw going up"); break;
    case 3: shoot(1); Serial.println("Shooting long range"); break;
    case 4: servo_90();break;
    case 5: servo_0();break;
    case 6: ;
  }
}
