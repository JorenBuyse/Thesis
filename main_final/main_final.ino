 
//#include <Wire.h>
#include <Servo.h>
//#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include "ssid_pass.h" 

//LOW LEVEL 
int command;  //not used
int bitrate = 1500000;  //bitrate of serial communication between end effector and baxter
byte applicationNumber = 1; //not used
byte deviceNumber = 1;  //used in hardware response
int packetLength = 13;
int responsePacketLength = 13;
int encodedPacketLength = 17; 

//SENSOR
const int numberOfSensorPoints = 2;
int data[numberOfSensorPoints];   //contains the data from all points, will be sent over udp


//NETWORK
WiFiUDP Udp;
IPAddress baxterIP = (192,168,0,58);
unsigned int baxterPort = 2390;
unsigned int localPort = 2391;      // local port om te luisteren
int status = WL_IDLE_STATUS;
char ssid[] = SECRET_SSID;        // network SSID
char pass[] = SECRET_PASS;    //network password: wpa (of key voor wep)
int keyIndex = 0;            // key index number (enkel voor wep)
char packetBuffer[255]; //buffer to hold incoming packet

//SERVO
Servo myservo1;
Servo myservo2;
byte aantalServos = 1; //change to number of servos in end effector
int currentSetpoint1 =180;
int currentSetpoint2 = 180;
int pos = 180;    // variable to store the servo position
const int maxs = 180;
const int mins = 120;

void setup() {
  init_serial();
  init_wifi();  
  init_sensor();
  your_init();
}

void loop() {  
  byte* packet = readLowLevel();
  handlePacket(packet);
  handleWiFi();
  handleSensor();
  handleUDP();  
  setServo();  
  }

void setServo() {         //sets the servo the value in pos
  myservo1.write(pos);
}

void your_init(){         //can be customized for other initialisations
  init_servo();
}

void init_sensor() {      //initialises pins for sensors
  pinMode(A1,INPUT);
  for (int i=2;i<4;i++) {
    pinMode(i, INPUT);
  }  
}
void init_wifi() {      //makes connection with access point created by workstation
  if (WiFi.status() == WL_NO_SHIELD) {
    //Serial.println("WiFi shield not present");
    while (true);
  }
  
  WiFi.config(IPAddress(192, 168, 0, 59));  //needs to be the same as referred to in the ROS-node
  
  WiFi.begin(ssid);
  delay(2000); //2 second delay
  Udp.begin(localPort); //start socket on localport
}

void init_serial() {  
   Serial1.begin(bitrate, SERIAL_8N1);     //initialises serial communicaton with baxter
}


void init_servo() { //can be deleted
  myservo1.attach(5);    //servo on pin 5
  
}

void dummyData(){   //can be deleted
    for(int i =0; i<numberOfSensorPoints; i++) {
      data[i]=i;
    }
}
void sendSensorData(){      //create a UDP packet and send sensordata and current setpoint of servo to baxter
  Udp.beginPacket(baxterIP,baxterPort); 
  Udp.print("D:,");
  Udp.print(data[0]); 
  for (int i = 1; i<numberOfSensorPoints; i++) {    
    Udp.print(",");  
    Udp.print(data[i]);     
  }   
  Udp.print(",");
  Udp.print(pos);
 
   Udp.endPacket();  
   }

byte* readLowLevel() {      //read packets from baxter coming in on the serial port
  while(!Serial1);  
  if (Serial1.available() >= 34) {
    
    while(Serial1.read() != 0){ //wachten opolgende 0 (COBS encodering zet frame delimiters op 0)
    }
      
    byte dataLength;
    while (true){
       dataLength = Serial1.read();
       if (dataLength != 0){
          break;
       }
    }
    byte recievedPacket[dataLength];      
    for (int i=0;i<dataLength;i++){
       recievedPacket[i] = Serial1.read();
    }
    
    byte decodedPacket[dataLength-1]; //eerste nul zit niet meer in gedecodeerd packet
    int length = cobs_decode(recievedPacket,dataLength, decodedPacket);
   }
}
void handleSensor() {     //read sensor values and store them in data
  for(int i =2; i<4;i++) {
    pinMode(i,OUTPUT);
    digitalWrite(i, HIGH);
    data[i-2]=analogRead(A1);
    pinMode(i, INPUT);
  }
}

void handleWiFi(){
  
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

   if(status == WL_DISCONNECTED) {  //if disconnected try again
      WiFi.begin(ssid);
 
    }
  }
}

void handleUDP() {      //reads udp packet and calls sendSensorData
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    IPAddress remoteIp = Udp.remoteIP();
    baxterIP = remoteIp;
    baxterPort = Udp.remotePort();
    
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
           
    pos = atoi(packetBuffer); //needs changes for multiple servos: use handlebuffer, checkbuffer and mapservo
    
    sendSensorData();    
  }
}

byte* handleBuffer() { //can be changed for different functionality 
                      //here it is used to isolate different servo values from the packet and put them in an array
                      
    char *ptr = NULL;
    char *servostr[aantalServos*3];
    ptr = strtok(packetBuffer, ",");
    servostr[0]= ptr;  
    ptr = strtok(NULL, ",");
    servostr[2]=ptr;

    byte servoValue[aantalServos];
    for(int i =0; i<aantalServos; i++) {
      servoValue[i] = atoi(servostr[i]);
    }
    return servoValue;  
}

bool checkBuffer(byte servoValue[]){ //returns false when value is out of bounds
  for(int i = 0; i<aantalServos; i++) {   //checken dat waardes tussen 0 en 100 liggen
        if(servoValue[i]>100 || servoValue[i]<0) {
          return false;
          break;
        }
    }
}
  
void mapServo(byte servoAngles[]) {  //this function maps the servo values in % recieved from baxter [0,100] to the proper angles for your servos
   
  //number of servos can be changed up top in aantalServos
  int minAngles[aantalServos];
  int maxAngles[aantalServos];
  
  //change min and max angles here, if more than 2 servos add more lines
  
  minAngles[0]=25;  //minimum angle of first servo
  minAngles[1]=25;  //minimum angle of second servo
  //minAngles[2] = ?;
  
  maxAngles[0]=256; //maximum angle of first servo
  maxAngles[1]=256; //maximum angle of second servo
  //maxAngles[2] = ?;
  
  for(int i=0; i<aantalServos; i++) {
    servoAngles[i] = map(servoAngles[i],0,100,minAngles[i],maxAngles[i]);
  }
  
}

size_t cobs_decode(const uint8_t * input, size_t length, uint8_t * output)
{   //decodes cobs encoded packet input with specified length and stores them in output
    size_t read_index = 0;
    size_t write_index = 0;
    uint8_t code;
    uint8_t i;

    while(read_index < length)
    {
        code = input[read_index];
        
        if(read_index + code > length && code != 1)
        {
            return 0;
        }

        read_index++;

        for(i = 1; i < code; i++)
        {
            output[write_index++] = input[read_index++];
        }
        if(code != 0xFF && read_index != length)
        {
            output[write_index++] = '\0';
        }
    }

    return write_index;
}


size_t cobs_encode(const uint8_t * input, size_t length,uint8_t * output)
{   //encodes packet input with length and stores it in output
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while(read_index < length)
    {
        if(input[read_index] == 0)
        {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        }
        else
        {
            output[write_index++] = input[read_index++];
            code++;
            if(code == 0xFF)
            {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }

    output[code_index] = code;

    return write_index;
}

void buildFrame(const byte * responsePacket, byte * output) { //generates a frame to be sent to baxter
    
    byte *p = output; *p++; *p++;
    int lengte = cobs_encode(responsePacket, responsePacketLength, p);
    output[0]=0; //eerste bit op 0 als delimiter
    output[1]=responsePacketLength+1;  //totale data= responsePacket + 1 bit overhead van COBS
    output[encodedPacketLength-1]=0; //laatste bit op 0 als delimiter
}

void addChecksum (byte * packet, int lengte) {    //adds a checksum to the frame
    uint8_t checksum =0;
    for(int i =0; i<lengte-1; i++) {
       checksum = checksum + packet[i];
    }
        packet[lengte-1] = checksum; 
}

void handlePacket(byte * packet) {  //calls the correct method corresponding with the respons packet according to the protocol
  command = packet[0];
  switch (command) {
    case 0:
      identificationResponse();
      
      break;
    case 1:
      calibrateResponse();
      break;
    case 2:
      //startDownloadResponse();
      break;
    case 3: 
      statusResponse();      
      break;
    case 4:
      idleResponse();       
      break;
    case 5:
      statusResponse();
      break;
    case 6:
      hardwareResponse();
      
      break;
    case 7:
      if(packet[1]==0x34){  //start app
        identificationResponse2();
      }
      else if (packet[1] == 0) {
        softwareResponse();
      }     
      break;
  }
 }

int bytesToInt(byte b1, byte b2){ //calculates the value of 2 consecutive bytes
  return (b1*256+b2); 
}

void identificationResponse2() { //byte6 cleared id response frame
    byte responsePacket[responsePacketLength];
    responsePacket[0] = B10111110;   
    responsePacket[1] = 0x01; 
    responsePacket[2] = 0x00; //manufacturer id 0xFFFF voor development end effectors
    responsePacket[3] = deviceNumber; 
    for(int i = 4; i<12; i++) {
      responsePacket[i]=0;
    }
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];
    
    buildFrame(responsePacket, encodedPacket);
    
    sendResponse(encodedPacket);
}

void identificationResponse() { //byte6 not cleared
    byte responsePacket[responsePacketLength];
    responsePacket[0] = B11000000 + applicationNumber;   //byte 0 begint met 11 en dan application number
    responsePacket[1] = 0x01; 
    responsePacket[2] = 0x00; //manufacturer id 0xFFFF voor development end effectors
    responsePacket[3] = deviceNumber; 
    for(int i = 4; i<12; i++) {
      responsePacket[i]=0;
    }
    
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];
    
    buildFrame(responsePacket, encodedPacket);
        
    sendResponse(encodedPacket);
}

void protocolResponse() {   //not used
    byte responsePacket[responsePacketLength];
    responsePacket[0] = 128;   //byte 0 begint met 11 en dan application number
    responsePacket[1] = 0x00; 
    responsePacket[2] = 0x02; 
    responsePacket[3] = 0x01; 
    for(int i = 4; i<12; i++) {
      responsePacket[i]=0;
    }
    
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];
    
    buildFrame(responsePacket,encodedPacket);
    
    sendResponse(encodedPacket);
}

void idleResponse() {   //not used
    byte responsePacket[responsePacketLength];
    responsePacket[0] = 1;
    for(int i = 1; i<12; i++) {
      responsePacket[i]=0;
    }
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];    
    buildFrame(responsePacket, encodedPacket);
    sendResponse(encodedPacket);
}

void statusResponse() { //builds status response
    byte responsePacket[responsePacketLength];
    responsePacket[0] = 3;
    responsePacket[1]= 4;
    responsePacket[2]= 0;
    for(int i = 3; i<12; i++) {
      responsePacket[i]=0;
    }
    responsePacket[6]=0;
    responsePacket[7]=24;
    //addData(responsePacket);
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];    
    buildFrame(responsePacket, encodedPacket);    
   
    sendResponse(encodedPacket);
}

void calibrateResponse() {  //not used
    byte responsePacket[responsePacketLength];
    responsePacket[0] = 3;
    responsePacket[1]= 4;
    responsePacket[2]= 0x80;
    //responsePacket[3]= B00000000;
    //responsePacket[4]= 0;
    for(int i = 3; i<12; i++) {
      responsePacket[i]=0;
    }
    responsePacket[6]=0xFF;
    responsePacket[7]=0xFF;
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];    
    buildFrame(responsePacket, encodedPacket);
   
    sendResponse(encodedPacket);
}

void softwareResponse(){  //builds software response
  byte responsePacket[responsePacketLength];
    responsePacket[0] = 0x02;   //byte 0 begint met 11 en dan application number
    responsePacket[1] = 0x03; 
    responsePacket[2] = 0x03; //manufacturer id 0xFFFF voor development end effectors
    responsePacket[3] = 0x01; 
    for(int i = 4; i<12; i++) {
      responsePacket[i]=0;
    }
    
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];
    
    buildFrame(responsePacket,encodedPacket);

    sendResponse(encodedPacket);
}

void hardwareResponse(){  
  byte responsePacket[responsePacketLength];
    responsePacket[0] = 0x01;   
    responsePacket[1] = 0xE8; 
    responsePacket[2] = 0xE3; 
    responsePacket[3] = 0x93;
    responsePacket[4] = 0x4c; 
    responsePacket[5] = 0x02;
    responsePacket[6]=  0x00;
    for(int i = 7; i<12; i++) {
      responsePacket[i]=0;
    }
    addChecksum(responsePacket, responsePacketLength);
    
    byte encodedPacket[encodedPacketLength];
    
    buildFrame(responsePacket,encodedPacket);

    sendResponse(encodedPacket);
}

void sendResponse(byte * response) {
    //writes the packet response to baxter 
    Serial1.write(response, encodedPacketLength);
    
}

void printWiFiStatus() {  //used for debugging
  // print the SSID of the network you're attached to:
  /*Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  Serial.print(mac[5],HEX);
  Serial.print(":");
  Serial.print(mac[4],HEX);
  Serial.print(":");
  Serial.print(mac[3],HEX);
  Serial.print(":");
  Serial.print(mac[2],HEX);
  Serial.print(":");
  Serial.print(mac[1],HEX);
  Serial.print(":");
  Serial.println(mac[0],HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");*/
}
void printMacAddress(byte mac[]) {
  /*for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();*/
}
