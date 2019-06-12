// Written by Chris Giverin, Julian Wang, and Ryan Tennant
// March 2, 2017: Re-enabled microstepping feature.
// June 08, 2017: Converted travel to count in leadscrew steps for precision
// July 07, 2017: Changed Home Position. Now Z axis travels to its highest position (UP), and sets this location to zero to prevent the hydrophone from crashing into the skull/skull holder

// STEPS TO CONTROLLING THE POSITIONER
// 1. Open Matlab and run FOH_Pressure_Measurement_r2.m
// 2. Ensure the board is powered, the ethernet cable is connected, and the Arduino is connected by USB to your computer
// 3. Determine the IP Address of the Arduino Ethernet by opening the serial monitor (should only be necessary if IP Address Changes)
// 4. Click "Connect" in the Matlab GUI until the box turns green
// 5. Click "Update Status" to initialize the current location of the positioner (wait until the "updated position" box is all zeros)
// 6. Click "Home" to return the positioner to its 0, 0, 0 location if it is not currently there
// 7. Click "Zero" to ensure the homed coordinates are set as the 0, 0, 0 position
// 8. Click "Update Status" to ensure the positioner has been homed to zero, and click "Zero" again
// 8. Operate the positioner by inputting distances in mm and click "Send" (10000 = 100mm, 1000 = 10mm, 100 = 1mm, 10 = 0.1mm, 1 = 0.01mm)
// 9. To return to the home position, set the distances for the x, y and z axes to 0
// 10.If using the Scan function, always ensure that you click "Update Status" before clicking "Scan", ensuring that all three coordinate boxes display the same values

// Z Motor moves first to ensure that the hydrophone is raised before moving in the xy-plane so that it does not crash into the skull/skull holder
// X Motor moves second
// Y Motor moves third

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Arduino.h>
byte mac[] = { 0x90, 0xA2, 0xDA, 0x10, 0x41, 0x7C };
IPAddress ip(142, 20, 168, 51);

//port 80 is default for HTTP:
//EthernetServer server(80);

//max length of messages sent to arduino:
#define message_length 60
//TL
unsigned int localPort = 8888;      // local port to listen on

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  //buffer to hold incoming packet,
char  ReplyBuffer[] = "acknowledged";       // a string to send back
//TL
///////////////////////////
//-----PIN SETTINGS------//
///////////////////////////
#define stepX 9
#define directionX 7
#define stepY 6
#define directionY 4
#define stepZ 3
#define directionZ 12


////////////////////////////////
//-----Global Variables------//
////////////////////////////////
String message;
boolean Microstep = LOW;
boolean debug = true;
float Command[4] = {};

int homeCount = 0;

unsigned long currentTime = 0; //unused
unsigned long previousTime = 0; //unused

float xyz[3] = {0, 0, 0}; // index 0 is x position, 1 is y position, 2 is z position

int Speed = 1;
int Speed1 = 1; // This is the delay speed, which is either 1 for normal or 16 for microstepping

//Added to global variables
//Add to global variables
int stepxyz[3] = {9, 6, 3};
int dirxyz[3] = {7, 4, 12};
bool highlow[2] = {LOW, HIGH};
int lswitches[6] = {0, 1, 2, 3, 4, 5};

int travelDir[3] = {0, 0, 0}; //direction of travel, 0 = neg, 1 = pos;

String stringx;
String stringy;
String stringz;

void setup()
{
  pinMode(stepX, OUTPUT);
  pinMode(stepY, OUTPUT);
  pinMode(stepZ, OUTPUT);
  pinMode(directionX, OUTPUT);
  pinMode(directionY, OUTPUT);  //Set Stepper Motor Pins
  pinMode(directionZ, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);

  digitalWrite(2, LOW);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);   //Set Limit Switch Pins
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  //Begin a connection with computer
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);              //Start UDP connection

  Serial.begin(9600);

  Serial.println(Ethernet.localIP());
}

void loop()
{
 
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) 
  {
    Serial.print("############################Received packet of size ##########################");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++) 
    {
      Serial.print(remote[i], DEC);
      if (i < 3) 
      {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    //Read and parse the UDP packet - should be only NUMXNUMYNUMZ
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    Serial.println("Contents:");
    Serial.println(packetBuffer);

    int ifsuccess = readMessage();
    if (ifsuccess) 
    {
      if(Microstep == LOW)
      {
        Speed1 = Speed;
      }
      
      else if (Microstep == HIGH) 
      {
        Speed = 1;
      }
      
      getQuickstat();
      Serial.println("Finished with packet.");
      Serial.print(Speed1);
      Serial.print(" ,");
      Serial.println(Speed);
    } 
    
    else 
    {
      Serial.println("Error: message not recognized. Please enter the following commands:");
      Serial.println("home");
      Serial.println("zero");
      Serial.println("status");
      Serial.println("@#####X#####Y#####Z");
      Serial.println("$#####X#####Y#####Z");
    }
  }//if packet size
}//void loop();

int readMessage() 
{
  Serial.println("Executing readMessage()");
  message = String(packetBuffer);
  if (message.substring(0, 1) == "*"){
    return 1;
  } else if (message.length() == 19 && (packetBuffer[0] == '@' || packetBuffer[0] == '$')){
    convertCommand();
    return 1;
  } else if (message.substring(0,4) == "mson"){
    Microstep = HIGH;
    Serial.println("microstepping on");
    digitalWrite(2,Microstep);
    Speed1 = 16;
    return 1;
  } else if (message.substring(0,5) == "msoff"){
    Microstep = LOW;
    Serial.println("microstepping off");
    digitalWrite(2,Microstep);
    Speed1 = Speed;
    return 1;
  } else if (message.substring(0, 4) == "home") {
    goHome();
    return 1;
  } else if (message.substring(0, 4) == "zero") {
    goZero();
    return 1;
  } else if (message.substring(0, 6) == "status") {
    getStatus();
    return 1;
  }else if (message.substring(0,7) == "debugon"){
    debug = true;
    Serial.println("Debugging on");
    return 1;
  } else if (message.substring(0,8) == "debugoff"){
    debug = false;
    Serial.println("Debugging off");
    return 1;
  } else if (message.substring(0,5) == "speed") {
    String tempval = message.substring(5,6);
    Speed = 10-tempval.toInt();
    Serial.print("\n Speed set to: " );
    Serial.println(Speed);
    return 1;
  } else {
    return 0;
  }
} //readMessage();

void convertCommand() 
{
  Serial.println("Executing convertCommand()");
  Command[0] = message.substring(0, 1).toInt();
  
  if (packetBuffer[0] == '@') 
  {
    // multiply by 2 to get # of steps (e.g. input of 00100->100->200 = 1 rotation = 1mm)
    Command[1] = message.substring(1, 6).toInt() * 2; 
    Command[2] = message.substring(7, 12).toInt() * 2;
    Command[3] = message.substring(13, 18).toInt() * 2;
  } 
  
  else 
  {
    if (packetBuffer[6] == 'X') 
    {
      Command[1] = xyz[0] +  message.substring(1, 6).toInt() * 2; 
    } 
    else 
    {
      Command[1] = xyz[0] -  message.substring(1, 6).toInt() * 2;
    }

    if (packetBuffer[12] == 'Y' ) {
      Command[2] = xyz[1] +  message.substring(7, 12).toInt() * 2;
    } 
    else 
    {
      Command[2] = xyz[1] -  message.substring(7, 12).toInt() * 2;
    }

    if (packetBuffer[18] == 'Z') {
      Command[3] = xyz[2] +  message.substring(13, 18).toInt() * 2;
    } 
    else 
    {
      Command[3] = xyz[2] -  message.substring(13, 18).toInt() * 2;
    }

  } //if else $ or @
  controlMotors();
} //void convertCommand();

void controlMotors() 
{
  // initialize the travel direction to 0
  int dirX = 0;
  int dirY = 0;
  int dirZ = 1;

  // determine the travel distance by subtracting the initial position from the command
  int stepsX = Command[1] - xyz[0];
  int stepsY = Command[2] - xyz[1];
  int stepsZ = Command[3] - xyz[2];

  // determine the travel direction and set it to 1 if positive (HIGH) or leave at 0 if negative (LOW)
  if(stepsX > 0)
  {
    dirX = 1;
  }

  if(stepsY > 0)
  {
    dirY = 1;
  }

  if(stepsZ > 0)
  {
    dirZ = 0;
  }

  // initilize limit switch cache
  int lswitchcache[6] = {0, 0, 0, 0, 0, 0};

  // initialize a counter to stop the motors running once they have moved
  int stopMotors = 0;

  // run through the motor on each axis one time, then terminate travel
  while(stopMotors < 3)
  {
    /////////////////////
    // CONTROL Z MOTOR //
    /////////////////////
    if(stepsZ > 0)
    {
      digitalWrite(directionZ, LOW);
      for(int i = 0; i < stepsZ; i++)
      {
        if(analogRead(5) > 90)
        {
          lswitchcache[4]++;
        }
        else 
        {
          lswitchcache[4] = 0;
        }
          
        if (lswitchcache[4] > 9) 
        {
          stopMotors++;
          digitalWrite(stepZ, LOW);    
        }
        else
        {
          digitalWrite(stepZ, HIGH);
          delay(1);
          digitalWrite(stepZ, LOW);
          delay(1);
        }
      }
      digitalWrite(stepZ, LOW);
      stopMotors++;
      xyz[2] = Command[3];
    }

    else
    {
      digitalWrite(directionZ, HIGH);
      for(int i = 0; i < abs(stepsZ); i++)
      {
        if(analogRead(4) > 90)
        {
          lswitchcache[5]++;
        }
        else 
        {
          lswitchcache[5] = 0;
        }
          
        if (lswitchcache[5] > 9) 
        {
          stopMotors++;
          digitalWrite(stepZ, LOW);    
        }
        else
        {
          digitalWrite(stepZ, HIGH);
          delay(1);
          digitalWrite(stepZ, LOW);
          delay(1);
        }
      }
      digitalWrite(stepZ, LOW);
      stopMotors++;
      xyz[2] = Command[3];
    }
    
    /////////////////////
    // CONTROL X MOTOR //
    /////////////////////
    if(stepsX > 0)
    {
      digitalWrite(directionX, HIGH);
      for(int i = 0; i < stepsX; i++)
      {
        // reading the limit switch value
        // these statements were taken from the previous code version
        // once the limit switch is activated 9 times above 90, then it is considered to be triggered
        if(analogRead(1) > 90)
        {
          lswitchcache[0]++;
        }
        else 
        {
          lswitchcache[0] = 0;
        }
          
        if (lswitchcache[0] > 9) 
        {
          stopMotors++;
          digitalWrite(stepX, LOW);    
        }
        else
        {
          digitalWrite(stepX, HIGH);
          delay(1);
          digitalWrite(stepX, LOW);
          delay(1);
        }
      }
      digitalWrite(stepX, LOW);
      stopMotors++;
      xyz[0] = Command[1];
    }
    
    else
    {
      digitalWrite(directionX, LOW);
      for(int i = 0; i < abs(stepsX); i++)
      {
        if(analogRead(0) > 90)
        {
          lswitchcache[1]++;
        }
        else 
        {
          lswitchcache[1] = 0;
        }
          
        if (lswitchcache[1] > 9) 
        {
          stopMotors++;
          digitalWrite(stepX, LOW);    
        }
        else
        {
          digitalWrite(stepX, HIGH);
          delay(1);
          digitalWrite(stepX, LOW);
          delay(1);
        }
      }
       digitalWrite(stepX, LOW);
       stopMotors++;
       xyz[0] = Command[1];
    }

    /////////////////////
    // CONTROL Y MOTOR //
    /////////////////////
    if(stepsY > 0)
    {
      digitalWrite(directionY, HIGH);
      for(int i = 0; i < stepsY; i++)
      {
        if(analogRead(3) > 90)
        {
          lswitchcache[2]++;
        }
        else 
        {
          lswitchcache[2] = 0;
        }
          
        if (lswitchcache[2] > 9) 
        {
          stopMotors++;
          digitalWrite(stepY, LOW);    
        }
        else
        {
          digitalWrite(stepY, HIGH);
          delay(1);
          digitalWrite(stepY, LOW);
          delay(1);
        }
      }
      digitalWrite(stepY, LOW);
      stopMotors++;
      xyz[1] = Command[2];
    }
    
    else
    {
      digitalWrite(directionY, LOW);
      for(int i = 0; i < abs(stepsY); i++)
      {
        if(analogRead(2) > 90)
        {
          lswitchcache[3]++;
        }
        else 
        {
          lswitchcache[3] = 0;
        }
          
        if (lswitchcache[3] > 9) 
        {
          stopMotors++;
          digitalWrite(stepY, LOW);    
        }
        else
        {
          digitalWrite(stepY, HIGH);
          delay(1);
          digitalWrite(stepY, LOW);
          delay(1);
        }
      }
      digitalWrite(stepY, LOW);
      stopMotors++;
      xyz[1] = Command[2];
    } 
  } 
}

void goZero() 
{
  Serial.print("Executing goZero()");
  xyz[0] = 0;
  xyz[1] = 0;
  xyz[2] = 0;
  Serial.print("\nAll axis set to zero.");
}

void goHome() 
{
  digitalWrite(2,LOW);
  
  if (homeCount == 0)
  {
    Serial.println("\nHoming...");
    Command[0] = 28;
    Command[1] = -999999; 
    Command[2] = -999999;
    Command[3] = -999999;

    controlMotors();
    goZero();
    homeCount = 1;
  
    Command[1] = 100;
    Command[2] = 100;
    Command[3] = 100;

    controlMotors();
  
    goZero();
    homeCount = 1;
  }

  else
  {
    Serial.println("\nHoming...");
    Command[0] = 28;
    Command[1] = 0; 
    Command[2] = 0;
    Command[3] = 0;

    controlMotors();
    goZero();
  }
}

void getStatus() 
{
  // for debugging purposes
  Serial.println("\n\n\n***********  STATUS  ***********");
}

void getQuickstat() 
{
  // convert back to what was inputted through Matlab
  stringx = String((int)(xyz[0] / 2));
  stringy = String((int)(xyz[1] / 2));
  stringz = String((int)(xyz[2] / 2));

  int len = 5 - stringx.length();
  for (int i = 0; i < len; i++) 
  {
    stringx = 0 + stringx;
  }

  len = 5 - stringy.length();
  for (int i = 0; i < len; i++) 
  {
    stringy = 0 + stringy;
  }

  len = 5 - stringz.length();
  for (int i = 0; i < len; i++) 
  {
    stringz = 0 + stringz;
  }

  String packetresponse = stringx + stringy + stringz;
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(packetresponse);
  Udp.endPacket();
  Serial.println("\nExecuted getQuickstat()");
  Serial.println(packetresponse);
}



