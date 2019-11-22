/**************/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <U8g2lib.h>
#include <Wire.h>

#include <DeadReckoner.h>

#include "ArduinoJson.h"

const char* ssid = "TP-LINK_70CC"; //Enter your wifi network SSID
const char* password = "15112550230"; //Enter your wifi network password


const int FORWARDS_PRESSED = 1;
const int FORWARDS_RELEASED = 2;
const int BACKWARDS_PRESSED = 3;
const int BACKWARDS_RELEASED = 4;
const int RIGHT_PRESSED = 5;
const int RIGHT_RELEASED = 6;
const int LEFT_PRESSED = 7;
const int LEFT_RELEASED = 8;
const int L_PRESSED = 9;
const int L_RELEASED = 10;
const int M_PRESSED = 11;
const int M_RELEASED = 12;


/** ------------------------- Actuators ------------------------ **/
#define PIN_IN1 13            // Motor Control for L298N
#define PIN_IN2 12
#define PIN_IN3 16
#define PIN_IN4 14

enum enCommandType{           // Command type - send from pc or remote controller or mobile phone
  enConfig = 0,               // rc car configuration command
  enControl = 1               // rc car movement control command
};

struct cmdConfig              // rc car configuration command
{
    bool done;
    char  c;
    int16_t  i;
    float   f;
}cmdConfig;                   // rc car movement control command

struct cmdControl
{
    int  x;
    int  y;
    bool done;
    unsigned long last_update_time;             // When did the last update
}cmdControl;                                    // Command for RC car control


int motor_pwm_threshold = 0;                    // RC car motor movement PWM threashold, the motor will not move under this value, the value will be automatically tested
int motor_pwm_max = 1023;                       // ESP8266 default pwm maximum number

/** ------------------- Encoders and coordinates ------------------ **/
#define PIN_ENCODER_LEFT 4                      // ENCODER PINS - left
#define PIN_ENCODER_RIGHT 5                     // ENCODER PINS - right

#define RADIUS 35                               // wheel radius in mm // MEASUREMENTS The units for all measurements must be consistent.  You can use any length unit as desired.
#define LENGTH 130                              // wheel base length in mm
#define TICKS_PER_REV 20

#define POSITION_COMPUTE_INTERVAL 50            // milliseconds // TIME INTERVALS
#define SEND_INTERVAL 100                       // milliseconds

unsigned int prevPositionComputeTime = 0, prevSendTime = 0;   // Previous times for computing elapsed time.
double prevX = 0, prevY = 0;                                  // Previous x and y coordinate.

volatile unsigned int leftEncoderTicks, rightEncoderTicks;    // Number of left and right tick counts on the encoder.
void ICACHE_RAM_ATTR  pulseLeft() { leftEncoderTicks++; }
void ICACHE_RAM_ATTR  pulseRight() { rightEncoderTicks++; }

DeadReckoner deadReckoner(&leftEncoderTicks, &rightEncoderTicks, TICKS_PER_REV, RADIUS, LENGTH);

/** ------------------- Network and Screen ------------------ **/
#define UDP_SERVER_PORT 1111
#define SERIAL_BAUD_RATE 9600
#define PIN_OLED_CLK 2
#define PIN_OLED_SDA 0
#define OVERDUE_COMMAND_TIME 1000 //ms

char packetBuffer[512];

WiFiUDP Udp;
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ PIN_OLED_CLK, /* data=*/ PIN_OLED_SDA);   // ESP32 Thing, HW I2C with pin remapping



/** -------------------- declaration ------------------- **/
void setupOutputs();    // Setup screen or serial port
void setupSensors();    // setup sensors - coders, radars etc.
void setupActuators();  // setup motors
void setupServer();     // setup network and connections

void printStatus(char*);

/** -------------------- Main Coding Area ------------------- **/
void setup() {  
  setupOutputs();
  setupSensors();
  setupActuators();
  Serial.println();
  Serial.printf("Motor PWM threshold: %d",motor_pwm_threshold);
  Serial.println(); 
  setupServer();
}

void setupOutputs(){
  u8g2.begin();
  //u8g2.setDisplayRotation(U8G2_R1);
  u8g2.enableUTF8Print();    // enable UTF8 support for the Arduino print() function  
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  // use chinese2 for all the glyphs of "�������"
  //u8g2.setFont(u8g2_font_b10_t_japanese1);  // all the glyphs of "����ˤ�������" are already included in japanese1: Lerning Level 1-6
  u8g2.setFontDirection(0);
  u8g2.firstPage();  
  do {
    u8g2.setCursor(0, 15);
    u8g2.printf("Initializing...");
  } while ( u8g2.nextPage());

  Serial.begin(SERIAL_BAUD_RATE);
  delay(10);  
}

/**Attaches interrupt and disables all serial communications.
This is necessary because when interrupts are active, incoming serial communication can be lost.*/
void setupSensors(){
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
  pinMode(PIN_ENCODER_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENCODER_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT), pulseLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), pulseRight, RISING);
}

/** depends on setupSensors, so must be called after setupSensors()**/
void setupActuators(){  
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  int threshold_max=512;
  int threshold =0;
  do{
    threshold+=10;
    analogWrite(PIN_IN1,threshold);
    analogWrite(PIN_IN2,0);
    analogWrite(PIN_IN3,threshold);
    analogWrite(PIN_IN4,0);    
    
    if(threshold > threshold_max) break;
    
    delay(10);    
  }while(leftEncoderTicks<1 && rightEncoderTicks<1);
  
  analogWrite(PIN_IN1,0);
  analogWrite(PIN_IN2,threshold);
  analogWrite(PIN_IN3,0);
  analogWrite(PIN_IN4,threshold);
  delay(80);
    
  motor_pwm_threshold = threshold/2;            // too big because of the delay
  analogWrite(PIN_IN1,0);
  analogWrite(PIN_IN2,0);
  analogWrite(PIN_IN3,0);
  analogWrite(PIN_IN4,0);
}

void setupServer(){
  Serial.println();
  Serial.println();
  Serial.print("Connecting to WIFI network");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Udp.begin(UDP_SERVER_PORT);
}


int speed_avg_max = 1023;
int speed_avg_step = 10;
int speed_avg = 0;

int speed_delta_max = 511;
int speed_delta_step = 5;
int speed_delta = 0;


int left = 0;
int right =0;

unsigned long curr_loop_mills = 0;
  
void loop() {
  curr_loop_mills = millis();

  detectSensors();                                  // detect sensors
  detectCommands();                                 //  Serial.println("Starting detect commands...");
  handleCommands();                                 //  Serial.println("Starting handleCommands...");
  showStatus();
  
  delay(50); // wait
}

void detectSensors(){
  if (millis() - prevPositionComputeTime > POSITION_COMPUTE_INTERVAL) {
    // Computes the new angular velocities and uses that to compute the new position.
    // The accuracy of the position estimate will increase with smaller time interval until a certain point.
    deadReckoner.computePosition();
    prevPositionComputeTime = millis();
  }
  if (millis() - prevSendTime > SEND_INTERVAL) {
    // Cartesian coordinate of latest location estimate.
    // Length unit correspond to the one specified under MEASUREMENTS.
    double x = deadReckoner.getX();
    double y = deadReckoner.getY();

    // Left and right angular velocities.
    double wl = deadReckoner.getWl();
    double wr = deadReckoner.getWr();

    // getTheta method returns the robot position angle in radians measured from the x axis to the center of the robot.
    // This angle is set initially at zero before the robot starts moving.
    double theta = deadReckoner.getTheta();

    // Total distance robot has troubled.
    double distance = sqrt(x * x + y * y);

//    Serial.print("x: "); Serial.print(x);
//    Serial.print("\ty: "); Serial.print(y);
//    Serial.print("\twl: "); Serial.print(wl);
//    Serial.print("\twr: "); Serial.print(wr);
//    Serial.print("\ttheta: "); Serial.print(theta*RAD_TO_DEG); // theta converted to degrees.
//    Serial.print("\tdist: "); Serial.println(distance);
    
    prevSendTime = millis();
  }
  
}

void detectCommands(){
 int noBytes = Udp.parsePacket();
  String received_command = "";

  if ( noBytes ) {      
    Udp.read(packetBuffer,noBytes);
    packetBuffer[noBytes]='\0';

    StaticJsonDocument<200> doc;                                                  // json document object
    DeserializationError error = deserializeJson(doc, packetBuffer);              // Read Json to doc
    if (!error) {
        int cmdType = doc[0];                                                     // eques const char *myC = doc["myChar"].as<char*>();
        switch(cmdType){
          case enConfig:                                                          // config command, used for parameter settings for rc car
            // TODO:
            Serial.printf("Config Package");
            Serial.println();
            break;
          case enControl:                                                         // control command, used for motor control eg. forward, backward, rotate...                                                
            cmdControl.x  = doc[1]; 
            cmdControl.y  = doc[2]; 
            cmdControl.done = false;
            cmdControl.last_update_time = curr_loop_mills;
            break;
          default:
            Serial.printf("Packge unknown");
            Serial.println();
            break;            
        }
    }
  }
}

void handleCommands(){
  if(!cmdConfig.done){
    ;
  }
  if(!cmdControl.done){
    int x = cmdControl.x;
    int y = cmdControl.y;
    if(y>0){                         // moving forwards
      int delta = motor_pwm_max-motor_pwm_threshold;
      int left  = max(abs(x),y)*delta/100+motor_pwm_threshold;                  // lefft = max(x,y)
      int right = (max(abs(x),y)*y/(abs(x)+y))*delta/100+motor_pwm_threshold;        // right = max(x,y)*y/(x+y)
      if(x<0){
        int temp = left;
        left = right;
        right = temp;
      }
      analogWrite(PIN_IN1,left);
      analogWrite(PIN_IN2,0);
      analogWrite(PIN_IN3,right);
      analogWrite(PIN_IN4,0);   
      
      //Serial.printf("x,y=%d,%d\tForward:%d-%d",x,y,left,right);
      //Serial.println(); 
    }else if(y<0){                    // Moving backwards
      int delta = motor_pwm_max-motor_pwm_threshold;
      int left  = max(abs(x),abs(y))*delta/100+motor_pwm_threshold;                           // lefft = max(x,y)
      int right = (max(abs(x),abs(y))*abs(y)/(abs(x)+abs(y)))*delta/100+motor_pwm_threshold;        // right = max(x,y)*y/(x+y)
      if(x<0){
        int temp = left;
        left = right;
        right = temp;
      }
      analogWrite(PIN_IN1,0);
      analogWrite(PIN_IN2,left);
      analogWrite(PIN_IN3,0);
      analogWrite(PIN_IN4,right); 
    }else{                                      // y=0 standby or rotate
      if(x>0){                                  // y=0 && x>0 rotate clockwise
        int delta = motor_pwm_max-motor_pwm_threshold;
        int left = abs(x*delta/100/2)+motor_pwm_threshold;
        int right = abs(x*delta/100/2)+motor_pwm_threshold;
        analogWrite(PIN_IN1,left);
        analogWrite(PIN_IN2,0);
        analogWrite(PIN_IN3,0);
        analogWrite(PIN_IN4,right); 
        
      }
      else if(x<0){                             // y=0 && x>0 rotate anti-clockwise
        int delta = motor_pwm_max-motor_pwm_threshold;
        int left = abs(x*delta/100/2)+motor_pwm_threshold;
        int right = abs(x*delta/100/2)+motor_pwm_threshold;
        analogWrite(PIN_IN1,0);
        analogWrite(PIN_IN2,left);
        analogWrite(PIN_IN3,right);
        analogWrite(PIN_IN4,0); 
      } 
      else{                                     // y=0 && x=0 standby
        analogWrite(PIN_IN1,0);
        analogWrite(PIN_IN2,0);
        analogWrite(PIN_IN3,0);
        analogWrite(PIN_IN4,0); 
      }                                         
    }
    cmdControl.done = true;
  }

  if(curr_loop_mills - cmdControl.last_update_time > OVERDUE_COMMAND_TIME){
    analogWrite(PIN_IN1,0);
    analogWrite(PIN_IN2,0);
    analogWrite(PIN_IN3,0);
    analogWrite(PIN_IN4,0); 
  }
}


char text_status[64];

/**
* update the status to screen or serial port.
*/
void showStatus(){
  u8g2.setFont(u8g2_font_unifont_t_chinese2);  // use chinese2 for all the glyphs of "�������"
  //u8g2.setFont(u8g2_font_b10_t_japanese1);  // all the glyphs of "����ˤ�������" are already included in japanese1: Lerning Level 1-6
  u8g2.setFontDirection(0);
  u8g2.firstPage();  
  do {
    u8g2.setCursor(0, 15);
    u8g2.printf("Ticks=%d/%d",leftEncoderTicks,rightEncoderTicks);
    u8g2.setCursor(0, 40);
    u8g2.printf("PWM=%d/%d",left,right);
  } while ( u8g2.nextPage());

    // calculate the new offset for the scrolling    
//  status_scroll_offset-=1;              // scroll by one pixel
//  if ( (u8g2_uint_t)status_scroll_offset < (u8g2_uint_t)-status_scroll_text_width )  
//    status_scroll_offset = 0;             // start over again

}
