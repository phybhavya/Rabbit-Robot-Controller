// including libraries
#include <esp_now.h>
#include <WiFi.h>
#include<math.h>
//Include all the definations
#define con 21
#define joystick_x 35
#define joystick_y 33
#define offset_x -15.5
#define offset_y 14.5
#define forward 5
#define backward 17
#define left 18
#define right 2
#define L1 26
#define R1 22
#define min_vel -255
#define max_vel 255
#define Rotation_vel 100

//Recievers MAC Address
uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x59, 0xDF, 0x84};

/* Here your structure will be added where you will initialize all the data you want to send*/
typedef struct message
{
  uint8_t con_mode;
  int16_t motor[4];
  uint8_t custom_function[8];
} message;
message phase;

/*
   Here front_motor integer will give send the velocity of front two motors in 0,255,255 form
   Here front_motor integer will give send the velocity of front two motors in 0,255,255 form
   con mode will give which mode is controller in
   con_mode = 0 : static mode
   con_mode = 1 : dynamic mode
   custom function represent 7 fucntions each position in the integer will give either a one or zero
   one stands for on
   zero stands for off

   For Static  Mode
   1: Flybelt velocity 1 
   2: Servo in/out
   3: Lead Screw Up 
   4: Lead Screw Down
   5: Conveyor Belt
   6: Single Ring Shoot
   7: Multiple ring shot
   8: FLybelt velocity 2

*/
/*Structutre end */

/*Initialization of Structure,buttons, variables*/
int i;
int x=0,y=0;
int func_buttons[8] = {16,4,15,19,23,2,18,0};
uint8_t val[8] = {1};
int conveyor_f = 0;
int wheel_angles[4] = {135,225,315,45};
int16_t vel[4] ={0,0,0,0};
int8_t servo_angle =65;
/*Initialization ends */

esp_now_peer_info_t peerInfo; //Enum of espnow peer's info

/*This function is made for sending the data and getting a call back if the data has been transfered successfully
  The status variable stores that if the packet is sent or not, a if statement is used
*/
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n Last Packet Send Status update:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Zomato Success" : "Delivery boy didnt came");
}

/*fucntion ends */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);//Serial Baudrate

  /*Intialize all the pins  */
for (i=0;i<8;i++)
{
  pinMode(func_buttons[i],INPUT_PULLUP);
}
pinMode(con,INPUT_PULLUP);
pinMode(forward,INPUT_PULLUP);
pinMode(backward,INPUT_PULLUP);
pinMode(left,INPUT_PULLUP);
pinMode(right,INPUT_PULLUP);
pinMode(joystick_x,INPUT);
pinMode(joystick_y,INPUT);
pinMode(L1,INPUT_PULLUP);
pinMode(R1,INPUT_PULLUP);

  /*Initialization end */

  WiFi.mode(WIFI_STA);//Setting Device as Wi-Fi station

  /* Initializing ESP-NOW THe main thing, this has to be initialized no matter what */
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
    /* Initialization end */

    /* Registering the peer */
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.channel = false;
    /* End */

    esp_now_register_send_cb(OnDataSent);// Call back function register for Send CB to get the status of Transmitted Pack
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(con) == 0)
  {
    Serial.println("static");
    first_phase();
  }
  else
  {
    Serial.println("dynamic");
    second_phase();
  }
}
void first_phase()
{
    for (i =0; i<8; i++)
    {
      val[i] = digitalRead(func_buttons[i]);
     // Serial.println(digitalRead(func_buttons[i]));
      //Serial.print("\t");
    }
    if(val[1] == 0)
    {
      //Serial.println("Inside the loop");
      val[1] = servo_angle;
    }
   val[5] = conveyor_check(val[5]);
   int joy_x = analogRead(joystick_x); 
   int joy_y = analogRead(joystick_y); 
   //Serial.println("Joystick coordinates");

   x = map(joy_x,0,4095, 100, -100) - offset_x*(100-abs(x))/100;
   y = map(joy_y,0,4095, 100, -100) - offset_y*(100-abs(y))/100;
   joystick(x,y);
   if (digitalRead(R1) == 0)
   { right_rotate();}
   else if (digitalRead(L1) == 0)
   {left_rotate();}
 //  Serial.println(val[1]);
 for(i =0;i<8;i++)
   {
    phase.custom_function[i] = val[i];
   Serial.println(phase.custom_function[i]);
   }
    for(i = 0;i<4;i++)
    {
      phase.motor[i] = vel[i];
      Serial.print(vel[i]);
      Serial.print("\t");
    }
    Serial.println(" ");
   esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&phase, sizeof(phase));
  Serial.println(sizeof(phase));
  if (result == ESP_OK) {
     Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  }
void second_phase()
{
   for (i =0; i<5; i++)
    {
      val[i] = digitalRead(func_buttons[i]);
    }

    if(val[1] == 0)
    {
      val[1] = servo_angle;
    }
   val[5] = conveyor_check(val[5]); 
   int joy_x = analogRead(joystick_x); 
   int joy_y = analogRead(joystick_y); 
   x = map(joy_x,0,4095, 100, -100) - offset_x*(100-abs(x))/100;
   y = map(joy_y,0,4095, 100, -100) - offset_y*(100-abs(y))/100;
   joystick(x,y);
   movement();
   if (digitalRead(R1) == 0)
   { right_rotate();}
   else if (digitalRead(L1) == 0)
   {left_rotate();}
   for(i =0;i<5;i++)
   {
    phase.custom_function[i] = val[i];
   }
    for(i = 0;i<4;i++)
    {
      phase.motor[i] = vel[i];
      Serial.print(vel[i]);
      Serial.print("\t");
    }
    
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&phase, sizeof(phase));

  if (result == ESP_OK) {
     Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}
int conveyor_check(int check)
{
  if(check == 1)
  {
    conveyor_f != conveyor_f;
  }
  return check;
}
double tan_func(int a, int b)
{
  if (b == 0)
  { if (a > 0) return M_PI / 2;
    else if (a < 0) return 3 * M_PI / 2;
    else return 0;
  }
  else if (a >= 0 && b > 0)
    return atan(a / b);
  else if (b < 0)
    return M_PI + atan(a / b);
  else
    return 2 * M_PI + atan(a / b);
}
void joystick(int sx, int sy)
{
    int dist = sqrt(sx * sx + sy * sy);
    int  speedd = map(dist, 0, 141.4, 0, 100);
    double ang = tan_func(sy, sx) ;
    ang  = (ang * 180) / M_PI;
    int vel_x = speedd * cos((ang * M_PI) / 180);
    int vel_y = speedd * sin((ang * M_PI) / 180);
    for (int i = 0; i < 4; i++)
    {
      vel[i] = vel_x * cos((wheel_angles[i] *M_PI)/180) + vel_y * sin((wheel_angles[i] *M_PI)/180);
//      Serial.print(vel[i]);
//      Serial.print("\t");

      vel[i] = constrain(vel[i], min_vel, max_vel);// constraining the velcoity 
      if (vel[i] <= 20 && vel[i] >= -20) 
        {
          vel[i] = 0;
        }
}Serial.println(" ");


}
void left_rotate()
{
  vel[0] = 100;
  vel[1] = 100;  
  vel[2] = -100;
  vel[3] = -100;
}
void right_rotate()
{
  vel[0] = -90;
  vel[1] = -90;  
  vel[2] = 90;
  vel[3] = 90;
 
}


void movement()
{
  if(digitalRead(forward) == 0)
    {joystick(0,100);}
  else if(digitalRead(backward) == 0)
    {joystick(0,-100);}
  else if(digitalRead(left) == 0)
    {joystick(100,0);}
  else if(digitalRead(right) == 0)
    {joystick(-100,0);}
}
