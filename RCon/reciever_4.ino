//include all libraries
#include<Servo.h>
#include<esp_now.h>
#include<WiFi.h>

  /* Initializing of structure which will recieve the data */
  typedef struct message{
  uint8_t con_mode;
  int16_t motor[4];
  uint8_t custom_function[8];
   }message;
    message phase;
// this is where we give name to a particular structure
//you need to change 'example' with whatever name and dont forget it to change in OnDataRec function
  /* End */

  /* Intializing of pins  */
uint8_t dir[4] = {26,33,5,2};
uint8_t pwm[4] = {25,32,4,15};
uint8_t conveyor  = 0;
uint8_t servopin  = 18;
uint8_t shooting_dir[2] = {27,12};
uint8_t shooting_pwm[2] = {14,13};
uint8_t leadscrew_dir[2] = {22,19};
uint8_t leadscrew_pwm[2] = {23,21};
uint8_t sense = 34;
uint8_t lim = 39;
Servo servo1;
  /* End */
  /* Initialising Variables*/
  int16_t vel[4] = {0};
  uint8_t functions[8] = {1};
  uint8_t controller_mode = 0;
  int i = 0;
  int dirs[4] = {false, true, true, true};
  /* END*/
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: ");
  Serial.println(macStr);
  memcpy(&phase, incomingData, sizeof(phase));//Change the 'example' with the name of the structure intialized
  Serial.print("Byte Recieved: ");
  Serial.println(len);
  for(i = 0;i<4;i++){
    vel[i] = phase.motor[i];
    Serial.print(vel[i]);
    Serial.print("\t");
  }
  Serial.println(" ");
  for(i = 0;i<8;i++){
    functions[i] = phase.custom_function[i];
    Serial.println(functions[i]);
  }  

  controller_mode = phase.con_mode;
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//Initialize Baudrate
  //Set Device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
/* Initialize all the pins */
for(i =0;i<4;i++){
  pinMode(dir[i],OUTPUT);
  pinMode(pwm[i],OUTPUT);
}
servo1.attach(18);
for(i =0;i<2;i++){
  pinMode(shooting_dir[i],OUTPUT);
  pinMode(shooting_pwm[i],OUTPUT);
  pinMode(leadscrew_dir[i],OUTPUT);
  pinMode(leadscrew_pwm[i],OUTPUT);
 }
 pinMode(lim,INPUT);
/* End */  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // put your main code here, to run repeatedly:
motor_vel();
function_vel();
}
void motor_vel(){
  for(i =0;i<4;i++)
  {
    if(vel[i]<0)
    {
     digitalWrite(dir[i],!dirs[i]);
     analogWrite(pwm[i],-1*vel[i]); 
    }
    else
    {
     digitalWrite(dir[i],dirs[i]);
     analogWrite(pwm[i],  vel[i]); 
    }
  }
}
void function_vel()
{
  if(functions[0] == 0)
  {shooting_func(70);}
  else if(functions[7] == 0)
  {shooting_func(128);}
  else
  {shooting_func(0);}
  if(functions[1] > 0)
  {servo1.write(functions[1]);
    Serial.println("Servo is in");}
  else
  {servo1.write(65);
  Serial.println("Servo is OFF");}
  if(functions[2] == 0)
  {leadscrew_up();}
  if(functions[3] == 0)
  {if(digitalRead(lim) == 0){
    leadscrew_down();}
  }
//  if(functions[4] == 0)
//  {conveyor_on();}
//  else 
//  {conveyor_off();}
  if(functions[5] == 0)
  {}
  if(functions[6] == 0)
  {}

}
void shooting_func(int16_t x)
{for(i=0;i<2;i++){
  analogWrite(shooting_pwm[i],x);
  digitalWrite(shooting_dir[i],HIGH);
}
}
void leadscrew_up()
{for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],100);
  digitalWrite(leadscrew_dir[i],LOW);
}
delay(20);
for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],0);
  digitalWrite(leadscrew_dir[i],LOW);
}
}

void leadscrew_down()
{
  for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],100);
  digitalWrite(leadscrew_dir[i],HIGH);
}
delay(20);
for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],0);
  digitalWrite(leadscrew_dir[i],HIGH);
}
}

void conveyor_on()
{
  digitalWrite(conveyor,HIGH);
}
void conveyor_off()
{
  digitalWrite(conveyor,LOW);
}
void leadscrew_up_IR(){
  while(digitalRead(sense) != 1)
  {  for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],50);
  digitalWrite(leadscrew_dir[i],LOW);
}
 }
 for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],50);
  digitalWrite(leadscrew_dir[i],LOW);
}
}
void leadscrew_down_IR(){
  while(digitalRead(sense) != 1)
  {  for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],50);
  digitalWrite(leadscrew_dir[i],LOW);
}
 }
 for(i=0;i<2;i++){
  analogWrite(leadscrew_pwm[i],50);
  digitalWrite(leadscrew_dir[i],LOW);
}
}
