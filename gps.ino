
#include <TinyGPS.h>
#include <Wire.h>


#define address 0x1E //0011110b, I2C 7bit address of HMC5883
TinyGPS gps;
#define rxPin 0 
#define txPin 1

bool feedgps();

int pingPin = 16; // pingpin to Digital 8

int SAFE_ZONE = 10;

//Motor direction control pin def
#define right1 4
#define right2 5
#define left1 6
#define left2 7
#define pwm_left 9
#define pwm_right 10



void setup()

{




  Serial.begin(9600);
  delay(100);                   

  Serial.println("PANIMALAR INSTITUTE OF TECHNOLOGY ");
  Serial.println("====================================================");
  Serial.println("            -by S.Varun Kumaran                ");


  Wire.begin();

  Serial.print("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");
  Serial.print("$PMTK220,200*2C\r\n");

  // Set operating mode to continuous in compass
  Wire.beginTransmission(address); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();

  pinMode(left1, OUTPUT);//LEFT  
  pinMode(left2, OUTPUT); //LEFT
  pinMode(right1, OUTPUT); //R
  pinMode(right1, OUTPUT); //R
  pinMode(pwm_left, OUTPUT); //LEFT
  pinMode(pwm_right, OUTPUT); //R
  pinMode(pingPin, OUTPUT);
  pinMode(pingPin, INPUT);

}

inline void setDir(bool L1, bool L2, bool R1, bool R2){
  digitalWrite(left1,L1);//L
  digitalWrite(left2,L2);//L
  digitalWrite(right1,R1);//R
  digitalWrite(right2,R2);//R
}

void forward()
{
  setDir(0,1,0,1);
  analogWrite(pwm_left,200);//L
  analogWrite(pwm_right,255);//R
}

void Backward()
{
  setDir(1,0,1,0);
  analogWrite(pwm_left,200);//L
  analogWrite(pwm_right,255);//R
}


void search()
{
  setDir(0,1,1,0);
  analogWrite(pwm_left,255);//R
  analogWrite(pwm_right,255);//L

}

void Stop()
{
  setDir(0,0,0,0);
  analogWrite(12,0);//L
  analogWrite(13,0);//R
}

void right()
{
  setDir(1,0,0,1);
  analogWrite(pwm_left,255);//L
  analogWrite(pwm_right,255);//R
}

void left()
{
  setDir(0,1,1,0);
  analogWrite(pwm_left,255);//L
  analogWrite(pwm_right,255);//R
}
long getDistance(){
  long duration;
  long cm;
   pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH); // Sending a High Pulse on the Trig Pin of the Sensor
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  return duration / 29 / 2;
   Serial.print(cm);
  Serial.println("cm"); 
}

void loop(){

  long cm=getDistance();
  feedgps();
  getDistance();


  unsigned char cc = Serial.read();
  float flat, flon,x2lat,x2lon;
  unsigned long age;
  gps.f_get_position(&flat, &flon, &age);

  feedgps();
  getDistance();

  float flat1=flat;     // flat1 = our current latitude.  
  float flon1=flon;  // flon1 = our current longitude. 
  float dist_calc=0;
  float angle_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;
  x2lat=  12.997987      ;  //enter lat your waypoint
  x2lon= 80.168012     ;  // enter a long your waypoint
  //Calculates distance from current location to waypoint

  dist_calc=sqrt((((flon1)-(x2lon))*((flon1)-(x2lon)))+(((x2lat-flat1)*(x2lat-flat1))));
  dist_calc*=110567 ; //Converting to meters
 
  angle_calc=atan2((x2lon-flon1),(x2lat-flat1));

  float declinationAngle2 = 57.29577951;
  angle_calc*= declinationAngle2;
  feedgps();
  getDistance();

  if(angle_calc < 0){
    angle_calc = 360 + angle_calc;
    feedgps();
    getDistance();
  }
  
  if(angle_calc >0){
    angle_calc= angle_calc;
    feedgps();
    getDistance();
  }

  float angleDegrees = angle_calc;
  feedgps();
  // compass code==================================      
  int x, y, z;
  feedgps();
  getDistance();

  // Initiate communications with compass
  Wire.beginTransmission(address);
  Wire.write(byte(0x03));       
  Wire.endTransmission();

  Wire.requestFrom(address, 6);    // Request 6 bytes; 2 bytes per axis
  if(6<=Wire.available()) {    // If 6 bytes available
    x = Wire.read()<<8; 
    x |= Wire.read(); 
    z = Wire.read()<<8; 
    z |= Wire.read(); 
    y = Wire.read()<<8; 
    y |= Wire.read(); 
    feedgps();
    getDistance();
  }

  float heading = atan2(y,x);
  float declinationAngle = 0.0457;
  heading += declinationAngle;
  feedgps();
    getDistance();

  
  if(heading < 0){
    heading += 2*PI;
    feedgps();
    getDistance();
  }

  
  if(heading > 2*PI){
    heading -= 2*PI;
    feedgps();
    getDistance();
  }

  float headingDegrees = heading * 180/M_PI;
  feedgps();
  getDistance();

  

  Serial.print("E: ");
  
  Serial.print(flat);
Serial.println("   ");

  Serial.print("N: ");
  
  Serial.println(flon);


  Serial.print("d:");
  Serial.println(dist_calc,2);




  Serial.print("H:");
  Serial.println(headingDegrees,2);


  Serial.print("A:");
  Serial.println(angleDegrees,2);


  



  feedgps();
  getDistance();
  Serial.print("Lat: ");
  Serial.println(flat,8);
  Serial.print("Long: ");
  Serial.println(flon,8);
  Serial.print("distance:");
  Serial.println(dist_calc,8);    
  //compass========================================   
  Serial.println("Heading:\t");
  Serial.print(headingDegrees);
  Serial.println(" Degrees   \t");
  Serial.print(angleDegrees);
  Serial.println(" Degrees   \t");
  Serial.print(cm);
  Serial.println("cm"); 
  Serial.println("                 ");
  delay(500);


      
  bool condition = ((angleDegrees -10) < headingDegrees) && ((angleDegrees +10) > headingDegrees);  
  if(condition){    
    Serial.println("MOVE");

    Serial.println("   MOVE   ");
    feedgps();
    getDistance();
    forward();
    long stTime = millis();
    while(millis()-stTime < 4000){
      getDistance();
      feedgps();
      if(getDistance() <= 10)
      {
        feedgps();
        getDistance();
        Serial.println("OBS FORWARD");
        Serial.println("                 ");

    Serial.println("OBS FORWARD");
        feedgps();
        getDistance();
        Stop();
         myDelay(1000);    
        feedgps();
        getDistance();
        Backward();
         myDelay(500);
        feedgps();
        getDistance();
        right();
         myDelay(800);
        feedgps();
        getDistance();
        forward();
         myDelay(2000);
        feedgps();
        getDistance();
        left();
         myDelay(800);
        feedgps();
        getDistance();
        forward();
       // break;              

      }
      else{
      forward();
      }
      feedgps();
      getDistance();
      
    }
  }

  if(SAFE_ZONE>cm){    
    
   Serial.println("OBS");
Serial.println("                 ");


    feedgps();
        getDistance();
        Stop();
         myDelay(1000);    
        feedgps();
        getDistance();
        Backward();
         myDelay(500);
        feedgps();
        getDistance();
        right();
         myDelay(800);
        feedgps();
        getDistance();
        forward();
         myDelay(2000);
        feedgps();
        getDistance();
        left();
         myDelay(800);
        feedgps();
        getDistance();
        forward();
       
  }

  if(dist_calc<4)
  {
    feedgps();
    getDistance();
    Stop();

   
    Serial.println("      stop    ");
  }
  
  if(flon==1000 && flat==1000 )
  {
    feedgps();
    Stop();

    Serial.print("  NO SAT  ");
     Serial.println("NO SAT");
  }

  else {

    Serial.println("SEARCHING");
    search();
    feedgps();
    getDistance();
    delay(150);
    Stop();
    feedgps();
    getDistance();
    delay(70);
  }  

}


void myDelay(long duration){
  long stTime = millis();
  while(millis() - stTime <duration){
    feedgps();
    delay(1);
  }
}


 bool feedgps()        
{
  while (Serial.available())
  {
    if (gps.encode(Serial.read()))
      return true;
  }
  return false;
}

















