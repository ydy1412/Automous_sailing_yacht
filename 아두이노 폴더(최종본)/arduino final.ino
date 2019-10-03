#include <SoftwareSerial.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Wire.h>
#include <EEPROM.h>
                      
SoftwareSerial mySerial(10,16); // (rx,tx) bluetooth
Adafruit_BNO055 bno = Adafruit_BNO055(55);         /* SC(red) =>SDL , SD (blown) => SC */
int PWM_PIN = 9;
int Sailservo_PIN = 5;
int Rudderservo_PIN = 11;
Servo Rudservo;
Servo Sailservo;
String readString;
static bool valchange = true;
TinyGPSPlus gps;                
static int Sailval = 740;
static int Rudval = 90;
imu::Vector<3> euler;



struct Rudderdata  {
  int  index[9]={0,0,0,0,0,0,0,0,0};                    /*number of data*/
  int  datastack[9] = {0,0,0,0,0,0,0,0,0};
  bool isvalchange[9] = {false,false,false,false,false,false,false,false,false};
  int  oldindex=0;
  float oldangle=0.0;
  float Rudval[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  float data[10][9] = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0},
                       {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
                                       /*collecting data*/
  };  

/* at start, this function extract original data from EEPROM*/                                            
void Eepromextractor(struct Rudderdata *ptr) {
  for(int i=0; i<9;i++) {
    (*ptr).Rudval[i] = EEPROM.read(i);
  }    
  Serial.println("Eeprome end");
}

/* This function extract rudval from rudderdata*/
int Rudderfinder(float angle, struct Rudderdata rudderdata) {
  int iP;      /*i index, j index*/
  float Idif = abs(angle-rudderdata.Rudval[0]);
  for (int i = 1; i<9; i=i+1) {
    float Fdif = abs(angle-rudderdata.Rudval[i]);
    if(Fdif < Idif) {
      Idif = Fdif;
      iP = i;
    }
  }
 return  iP;
}

int Sailfinder(float WindAngle) {
   double L = 90.00; /* from mast to loop; */
   double R = 82.85;
   WindAngle = WindAngle * 2*PI/360.0;
   double A = 90-82.85*cos(WindAngle);
   double B = 82.85*sin(WindAngle);
   double C = sqrt(sq(A)+sq(B));    /* Dragonforce65 servo....................................
                                     * if Sailangle = 0 => Sailservo val = 1500; basevalue; 
                                     * Diameter of wheel = 20mm;
                                     * working frequency = 510μs, degree = 180; 
                                     * frequency per degree = 510/180;
                                     * length change per degree  = PI * Diameter/ 360 ;
                                     * So length change per frequency =  PI*Diameter/1020;
                                     * C-7.15 = length change;
                                     * frequency =1500 + (C-7.15) *1020/(PI*Diameter);
                                     * !!!!!!!!!!caution frequnecy can be 1500-(C-7.15) *620/(PI*Diameter);!!!!!!!!!
                                     * sign must be determined after real test.
                                     */
int Sailval = 740 + (C-7.15)*(51.0)/PI;
return Sailval;   
}

/*This function extracts direction of yacht*/

/* This function calculates optimizing data from collection data and input to EEPROM */
void machinelearning (float courseN, struct Rudderdata *ptr){  
 if ((*ptr).oldangle > 0 ) {
  float sum = 0.0;
  int i;
  bool rearrangement_up=false;
  bool rearrangement_down = false;
  float RealrotateAngle =  abs(courseN - (*ptr).oldangle);
  int m =  (*ptr).oldindex;
  int n =  (*ptr).index[m];
  (*ptr).data[n][m] =  RealrotateAngle;
  (*ptr).index[m] += 1;
  if ((*ptr).index[m] == 10) {
    for(i =0; i < 10; i++) {
      sum = sum +(*ptr).data[i][m];
    }
    if ((*ptr).isvalchange[m] ==  true){
      (*ptr).isvalchange[m] = false;
      (*ptr).Rudval[m] = sum/(10.0);
    }
    if ((*ptr).isvalchange[m] ==false) {
    (*ptr).Rudval[m] = ((*ptr).Rudval[m]*(*ptr).datastack[m]*(10.0)+sum)/((*ptr).datastack[m]*10.0+10.0);
    }
    (*ptr).index[m] = 0;
    (*ptr).datastack[m] +=1;
    for ( i = 0; i < 10; i++) {
     (*ptr).data[i][m] = 0.0;
    }
    for (i = m+1; i < 9; i++ ) {
      if ((*ptr).Rudval[i] < (*ptr).Rudval[m]) {
        rearrangement_up =true;
      }
   }
   if (rearrangement_up == true ) {
    for(i = m+1;i < 9;i++) {
      (*ptr).Rudval[i] = (*ptr).Rudval[m]+(i-m)*5;
      }
    }
    for ( i = 0 ; i < m; i++) {
      if ((*ptr).Rudval[i] > (*ptr).Rudval[m]) {
        rearrangement_down = true;
        }
    }
    
    if (rearrangement_down == true) {
     for (i = 0; i < m; i++ ) {
      (*ptr).Rudval[i] = (*ptr).Rudval[m] + (i-m)*5;
      }
      }
        
if (rearrangement_up == true && rearrangement_down ==false) {
  for (i = m; i < 9; i++ ) {
      EEPROM.write(i,(*ptr).Rudval[i]); 
  }    
}
if (rearrangement_up ==false && rearrangement_down == true) {
  for (i = 0; i < m; i++ ) {
    EEPROM.write(i,(*ptr).Rudval[i]);
  }
}
if(rearrangement_up == true && rearrangement_down ==true) {
  for (i = 0; i < 9; i++) {
    EEPROM.write(i,(*ptr).Rudval[i]);
  }
   }
  }
 }
}

/* Rotary function caculate Wind direction angle */
int Rotary() {
  double pwm_value = 0.0;
  pwm_value = pulseIn(PWM_PIN, HIGH);
  int windD = abs(pwm_value-925.0)*(360.0)/(925.0)+175.0;
  if(windD >= 360.0) {
    windD -= 360.0;
  }
  return windD;
}

void setup() {   
 Serial.begin(9600);
 pinMode(PWM_PIN, INPUT);
 mySerial.begin(9600);
 Serial1.begin(9600);
 Sailservo.attach(Sailservo_PIN,740,2600);
 Rudservo.attach(Rudderservo_PIN);
 Sailservo.writeMicroseconds(740);
 Rudservo.write(90);
 Sailservo.detach();
 Rudservo.detach();
   if(!bno.begin())
  {
    while(1);
  }
  bno.setExtCrystalUse(true);
}

void loop() {
 static struct Rudderdata rudderdata;
 struct Rudderdata *ptr;
 ptr = &rudderdata;
 int Rudderfinder(float angle, struct Rudderdata rudderdata);
 void machinelearning (float courseN, struct Rudderdata *ptr);
 int Sailfinder(float WindAngle);
 void Eepromextractor(struct Rudderdata *ptr);
 int Rotary();
 double atan2 (double __y, double __x);
 char a = {};
 static char mode = 'H'; 
 static int windD;                  /* Wind Direction data */
 String lat = "";              
 String longi = "";
 String Azimuth = "";
 long int ISpo[3] = {0,0,0};   /* setting Position   {latitute[degree], Longitute[degree],magnetic calibration data} */
 double Spo[3] = {0,0,0};
 static double Gpsdata [2] = {0,0};    /* Gps data {latitute[degree], Longitute[degree], Velocity[m]}  */
 static bool Start = false;        /* if Start == true, yacht can move along setting data */
 static bool setting_complete = false;
 static double courseTo;       /* Course Data to go [degree] */
 static float RotateA;
 static float courseN;


 
/* "mode H => bluetooth connection mod."*/

if ( mode == 'H' ) {
    delay(1000);
    if (mySerial.available()) {
    a = mySerial.read();
    switch (a) {
      case 'C' ://
      mode = 'C';
      // 2 = controlmod.
      break;
      case 'G' :
      mode = 'G';
      // 1= Gps mod.
      break;
  }   
  }
}   
/* "mode 2 => Direct control mod. "*/

while (mode == 'C') {
  Serial.println(mode);
  Sailservo.attach(Sailservo_PIN,740,2600); 
  Sailservo.writeMicroseconds(2500);
if (mySerial.available()) {
  a = mySerial.read();
  if (a == 'r') {    
     delayMicroseconds(200);
     while (mySerial.available()) {
      char c= mySerial.read();
      readString +=c;
      }
  }
   if (a == 's') {    
     delayMicroseconds(200);
     while (mySerial.available()) {
      char c= mySerial.read();
      readString +=c;
      }
    if(readString.length()>0) {
     float fSailval = readString.toInt() + 0.0;
     readString = "";
     fSailval = Sailfinder(fSailval);
     Sailservo.writeMicroseconds(fSailval);
     Serial.println(fSailval);
    }
  
  
  
  }

  
  
  if( a == 'H'){
  mode = 'H';
  Sailservo.detach();
  break;
 }
if(readString.length()>0){
float fRudval = readString.toInt()+0.0;
readString = "";
Sailservo.writeMicroseconds(2500);
Rudservo.attach(Rudderservo_PIN); 
Rudservo.write(fRudval);
delay(4000);
Rudservo.write(90);
delay(1500);
Rudservo.detach();
}
}
}


 
/* "mode G => GPS control mod."*/
while (mode == 'G') {
  Serial.println(mode);
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.println(euler.x());
  int Senddata = euler.x();
  Serial.println(Senddata);
  mySerial.write(Senddata);
  delay(1000);
if (mySerial.available()) {
  a = mySerial.read();
  if (a == 'a') {
    delayMicroseconds(1000);
     while (mySerial.available()) {
      char c= mySerial.read();
      readString +=c;
      }
  }
  if (a == 's' ) {
    Start = true;
   }
  }
  if (a == 'H') {
    mode = 'H';
    break;
  }
int i = 0 ;
if(readString.length()>0){
while (readString [i] != 'o') {
  lat += readString[i];
  i = i+1;
}
i=i+1;
while (readString[i] != 'c'){
  longi += readString[i];
  i=i+1;
}
i=i+1;
while (readString[i] != 'e'){
  Azimuth += readString[i];
  i=i+1;
}
ISpo[0] = lat.toInt();
ISpo[1] = longi.toInt();
ISpo[2] = Azimuth.toInt();
readString = "";
lat = "";
longi = "";
Azimuth = "";
}

if (ISpo[0] > 0 && ISpo[1] > 0 && ISpo[2] > 0 ) {
  Spo[0] = ISpo[0] / (1000000.00);             /*latitute data setting completed*/
  Spo[1] = ISpo[1] / (1000000.00);             /*longitute data setting completed*/
  Spo[2] = ISpo[2];                            /* callibration data setting completed */
  setting_complete = true;
}

while(Start == true && setting_complete == true) { 
  static bool machinelearningRecord = false;
  static bool isextract = true;
  if(isextract == true){
  Eepromextractor(ptr);
  isextract =false;
  } 
  if (mySerial.available()) {
  a = mySerial.read();
   if (a == 'H' ) {
   mode = 'H';
   Start = false;
   setting_complete = false;
   double Spo[3] = {0.0,0.0,0.0};
   }
   if (a == 'C' ) {
   mode = 'C';
   Start = false;
   setting_complete = false;
   double Spo[3] = {0.0,0.0,0.0};
   }
  } 
  Serial.println("rotary start");
  windD = Rotary(); 
bool GPS = false;
Serial.println("GPS  start");
while(GPS==false){
  while (Serial1.available()){
    if (gps.encode(Serial1.read())){
      if(gps.location.isValid()){
        Gpsdata[0] = gps.location.lat();
        Gpsdata[1] = gps.location.lng();
        GPS = true;
        break;
        }
        }
      }
  }
 euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
 courseN = euler.x()+Spo[2];
 if (courseN > 360.0) {
  courseN -= 360.0;
 }
 if(machinelearningRecord == true) {
 machinelearning (courseN, ptr);
 machinelearningRecord = false;
 }
 courseTo = gps.courseTo(Gpsdata[0], Gpsdata[1], Spo[0], Spo[1]);    /*  double TinyGPSPlus::courseTo(double lat1, double long1, double lat2, double long2) returns course angle in degrees (North=0, West=270) from position 1 to position 2,*/ 
 float courseD = abs(courseTo-courseN);  /* courseD =  course Difference (angle to rotate)*/
/* start moving yacht based on data */
 (*ptr).oldangle =  courseD;

/* In case of little difference of angle */
 
 if (courseD < 20) {
  Sailservo.attach(Sailservo_PIN,640,2600); 
  if( (windD >= 90 && windD <= 270)) {  /* in case of reverse direction wind */
    Rudservo.attach(Rudderservo_PIN);  
    Rudval =80-(Rudderfinder(45.0,rudderdata))*(10.0);  /* value of RudAngle right side*/
    Sailservo.writeMicroseconds(1500);
    Rudservo.write(Rudval);
    delay(1500);
    Rudservo.write(90);
    delay(1500);
    Rudval = 100.0 + (Rudderfinder(45.0,rudderdata))*(10.0);  /* value of RudAngle left side*/
    Rudservo.write(Rudval); 
    delay(1500);
    Rudservo.write(90);
    delay(1500);
    Rudservo.detach();
    }
 else {
    float Sailangle  = abs(180-windD)/(2.0);  
    if(Sailangle > 85) {
      Sailangle = 85;
    }
    Sailval = Sailfinder(Sailangle);
    Sailservo.writeMicroseconds(Sailval);
    delay(4000);
 }
 Sailservo.detach();
 }
 
/* In case of need for rotating yacht */
 if ( courseD >= 20 && courseD <= 90 ) {
    machinelearningRecord = true;
    Rudservo.attach(Rudderservo_PIN);
    RotateA = courseTo-courseN;
    if (RotateA > 0) {
    (*ptr).oldindex = Rudderfinder(courseD,rudderdata);
    Rudval = 80-((*ptr).oldindex)*(10.0);         /* Rudval = 0 일시 러더 우현으로 꺽임, 러더가 우현으로 꺽일시 요트 시계 방향으로 회전 */
    }
    if (RotateA < 0) {
    (*ptr).oldindex = Rudderfinder(courseD,rudderdata);  
    Rudval = 100 + ((*ptr).oldindex)*(10.0); 
    }
    Rudservo.write(Rudval);
    delay(1500); 
    Rudservo.write(90);
    delay(1500);
    Rudservo.detach();  
  } 

if ( courseD > 90 ) { 
  Rudservo.attach(Rudderservo_PIN);
  Sailservo.attach(Sailservo_PIN,640,2600); 
  Sailservo.writeMicroseconds(1500);
  RotateA = courseTo-courseN;
    if (RotateA > 0) {
      Rudval = 180;     
    }
    if (RotateA < 0) {
     Rudval = 0;       
    }
   Rudservo.write(Rudval);
   delay(1500);
   Rudservo.write(90);
   delay(1000);
   Rudservo.detach();
   Sailservo.detach();
    }
   } /*while(mode=!H)*/
  }
 } 
