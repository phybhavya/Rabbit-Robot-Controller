#include <esp_now.h>
#include <WiFi.h>
#define CHANNEL 1
float dir[3] = {25,33,32};
float pwm[3] = {27,14,12};
#define spd 50
int i,con;
  float forw;
  float back;
  float lef;
  float righ;
  float butoon;
typedef struct message{
  float buton;
//  float backward;
//  float left;
//  float right;
  float w0;
  float w1;
  float w2;
//  float con_mode;
  } message;
  message but;
bool wheel_dir[3]={true, true, true};
float vel[3] ={0,0,0};
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
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESPNow/Basic/Slave Example");
 for (i=0;i<3;i++)
 {
  pinMode(dir[i],OUTPUT);
 }
  for (i=0;i<3;i++)
 {
  pinMode(pwm[i],OUTPUT);
 }
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
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
  butoon = but.buton;
//  back = but.backward;
//  lef = but.left;
//  righ = but.right;
//  con = but.con_mode;
  vel[0] = but.w0;
  vel[1] = but.w1;
  vel[2] = but.w2;
  Serial.println(butoon);
  //  Serial.print("Last Packet Recv Data: "); Serial.println(*data);
//  Serial.println("");
//if(butoon == 1){
//  aage();
//}
//else if(butoon == 2){
//  piche();
//}
//else if(butoon ==3){
//  daye();
//}
//else if(butoon == 4){
//  baye();
//}
//else{
//  stop();
//}
}

void loop() {
  // Chill
//if(con == 0){
if(butoon == 1){
  aage();
}
else if(butoon == 2){
  piche();
}
else if(butoon ==3){
  daye();
}
else if(butoon == 4){
  baye();
}
else{
  stop();
}
 }
// else if(con == 1){
//  joy();
// }
//}
void aage(){
  digitalWrite(dir[1],LOW);
  digitalWrite(dir[2],LOW);
  analogWrite(pwm[1],50);
  analogWrite(pwm[2],50);
  analogWrite(pwm[0],0);
  Serial.println("Forward");
  }
void piche(){
  digitalWrite(dir[1],HIGH);
  digitalWrite(dir[2],HIGH);
  analogWrite(pwm[1],50);
  analogWrite(pwm[2],50);
  analogWrite(pwm[0],0);
  Serial.println("backward");  
}

void daye(){
  digitalWrite(dir[0],HIGH);
  digitalWrite(dir[1],HIGH);
  digitalWrite(dir[2],LOW);
  analogWrite(pwm[1],50);
  analogWrite(pwm[2],50);
  analogWrite(pwm[0],50);  
  Serial.println("right");
}

void baye(){
  digitalWrite(dir[0],LOW);
  digitalWrite(dir[1],LOW);
  digitalWrite(dir[2],HIGH);
  analogWrite(pwm[1],50);
  analogWrite(pwm[2],50);
  analogWrite(pwm[0],50);   
  Serial.println("left");
}
void stop(){
  analogWrite(pwm[1],0);
  analogWrite(pwm[2],0);
  analogWrite(pwm[0],0);  
}
void joy(){
   for(i=0;i<3;i++)
  {
    if(vel[i]<0)
    {
      vel[i]*=-1;
      digitalWrite(dir[i], !wheel_dir[i]);
      analogWrite(pwm[i], vel[i]);
    }

    else
    {
      digitalWrite(dir[i], wheel_dir[i]);
      analogWrite(pwm[i], vel[i]);
    }
  }
  
}
