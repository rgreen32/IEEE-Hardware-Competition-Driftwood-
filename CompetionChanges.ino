#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 Display(OLED_RESET);

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();



#define rm1 36
#define rm2 37
#define rsSR04 22
#define rfSR04 22
#define calibrate1 40
#define calibrate2 41
//1#define SCL 21
//#define SDA 20
//#define Battery A6
//#define Button A7
//#define Rx 0
//#define Tx 1
#define capSwitch 7
#define lsSR04 2
#define lmPWM 12
#define lfSR04 2
#define capm1 51     //Rey: captain's wheel 
#define capm2 50
#define capPWM 13
//#define irInput 5
#define fmPWM 13
#define fm1 50
#define fm2 51
#define lm2 26
#define lm1 27
#define liftServo 33
#define switchServo 47
#define rmPWM 11
#define lsPulse 3
#define rsPulse 23
#define MidPulse 44   // Rey
#define MidSR04 45   //Rey
#define GROSS 30

//  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
Servo myservo;  // create servo object to control a servo
Servo myservoB;  // create servo object to control a servo

// Global Variables
double x = 0;
int Y=0;
double y = 0;
double heading = 0.00;
double error;
int mSpeed = 0;
double Pk = 6;
double Pd = -60000;
double Pi = 0;
int rmSpeed = 0;
int lmSpeed = 0;
double eSum = 0;
double ePre = 0;
double pe = 0;
double ce = 0;
long t = 0;
long tp=0;
int pt = 0;
int ct = 0;
int dt = 0;
double dk = 0;
int adjust = 0;
int MAX = 100;
long dist=0;
int switchcount = 0;

void setup() {
pinMode( rm1,OUTPUT);
pinMode( rm2 ,OUTPUT);
pinMode( rsSR04 ,INPUT);
pinMode( rfSR04 ,INPUT);
//pinMode( SCL ,OUTPUT);
//pinMode( SDA ,OUTPUT);
//pinMode( Battery A6
//pinMode( Button A7
//pinMode( Rx ,INPUT);
//pinMode( Tx,OUTPUT);
pinMode( lsSR04 ,INPUT);
pinMode( lmPWM ,OUTPUT);
pinMode( lfSR04 ,INPUT);
//pinMode( irInput ,INPUT);
pinMode( fmPWM,OUTPUT);
pinMode( fm2,OUTPUT);
pinMode( fm1 ,OUTPUT);
pinMode( lm2,OUTPUT);
pinMode( lm1 ,OUTPUT);
pinMode( liftServo ,OUTPUT);
pinMode( switchServo,OUTPUT);
pinMode( rmPWM ,OUTPUT);
pinMode( lsPulse ,OUTPUT);
pinMode( rsPulse ,OUTPUT);
pinMode( capSwitch , INPUT_PULLUP);
pinMode( calibrate1, INPUT_PULLUP);
pinMode( calibrate2, OUTPUT);

digitalWrite(rm1, LOW);
digitalWrite(rm2, HIGH);
analogWrite(rmPWM,50);
digitalWrite(lm1, LOW);
digitalWrite(lm2, HIGH);
analogWrite(lmPWM, 25);
delay(1600);
brake();

 myservo.attach(liftServo);  // attaches the servo on pin defined as liftServo to the servo object
 myservoB.attach(switchServo);  // attaches the servo on pin defined as liftServo to the servo object
   myservo.write(37);
   delay(1000);
   myservo.detach();
    myservoB.write(10); // 10 = virtical  155 = lowest
    delay(1000);
    myservoB.detach();
  Serial.begin(9600);
  Display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  Display.clearDisplay();
  Display.display();
  Display.setTextSize(1);
  Display.setTextColor(WHITE, BLACK);
  Display.setCursor(40, 0);
  Display.println("- 1 0 1 -");
  Display.setTextSize(3);
  Display.setCursor(60, 12 );
  Display.println(99);
  //Display.display();
  //delay(10000);
 // Display.clearDisplay();
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while (1);
  }
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
 
  delay(1000);
  bno.setExtCrystalUse(true);
  
}
void displayCalStatus()
{
    uint8_t system, gyro, accel, mag;

  Serial.print("system");
  Serial.print(system, DEC);

  Serial.print("gyro");
  Serial.print(gyro, DEC);

  Serial.print("accel");
  Serial.print(accel, DEC);
  
  Serial.print("mag");
  Serial.println(mag, DEC);
}

void rm(int s) { // right motor control
  if (s > 255) {
    s = 255;
  }
  if (s < -255) {
    s = -255 ;
  }
  if (s > .5)
  {
    digitalWrite(rm1, LOW);
    digitalWrite(rm2, HIGH);
    analogWrite(rmPWM, abs(s));
  }
  else if (s < -.5)
  {
    digitalWrite(rm1, HIGH);
    digitalWrite(rm2, LOW);
    analogWrite(rmPWM, abs(s));
  }
  else
  {
    analogWrite(rmPWM, 0);
    digitalWrite(rm1, HIGH);
    digitalWrite(rm2, HIGH);
  }
}

void lm(int s) { // left motor control
  if (s > 255) {
    s = 255;
  }
  if (s < -255) {
    s = -255 ;
  }
  if (s > .5)
  {
    digitalWrite(lm1, HIGH);
    digitalWrite(lm2, LOW);
    analogWrite(lmPWM, abs(s));
  }
  else if (s < -.5)
  {
    digitalWrite(lm1, LOW);
    digitalWrite(lm2, HIGH);
    analogWrite(lmPWM, abs(s));
  }
  else
  {
    analogWrite(lmPWM, 0);
    digitalWrite(lm1, HIGH);
    digitalWrite(lm2, HIGH);
  }
}
void IMU()
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  y=euler.y();
  x = euler.x();
  }

void motors() { // loop to drive motors
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  y=euler.y();
  x = euler.x();
  error = x - heading;
  if (error > 180) {
    error -= 360;
  }
  if (error < -180) {
    error += 360;
  }
  pt = ct;
  pe = ce;
  ct = micros();
  ce = error;
  dk = Pd * (ce - pe);
  dk /= ct - pt;
  adjust = error * Pk + dk;
  if (adjust <= (-1 * MAX)) {
    adjust = -1 * MAX;
  }
  if (adjust >= MAX) {
    adjust = MAX;
  }

//  Display.clearDisplay();
//  Display.setTextSize(1);
//  Display.setTextColor(WHITE, BLACK);
//  Display.setCursor(0, 0);
//  Display.println(error);
//  Display.setCursor(0, 12);
//  Display.println(error * Pk);
//  Display.setCursor(80, 12);
//  Display.println(dk);
//  Display.setCursor(0, 24);
//  Display.println(adjust);
//  Display.display();
  rmSpeed = mSpeed + adjust;
  lmSpeed = mSpeed - adjust;
  rm(rmSpeed);
  lm(lmSpeed);
  eSum = eSum + error;
  ePre = error;
  if (eSum > 4000) {
    eSum = 4000;
  }
  if (eSum < -4000) {
    eSum = -4000;
  }
  delay(1);
}

void calibration()
{
  digitalWrite(calibrate2, LOW); // Rey
  while (digitalRead(calibrate1) == HIGH)
  {
  }


  }


void LmotorTest()
{
   digitalWrite(lm1, LOW);
    digitalWrite(lm2, HIGH);
    analogWrite(lmPWM, 100);
      }
void RmotorTest()
{
   digitalWrite(rm1, LOW);
    digitalWrite(rm2, HIGH);
    analogWrite(rmPWM, 100);
      }      

void grossTurn()
{
  int power = 150;
  if (error > 0)
  {
    digitalWrite(lm1, LOW);
    digitalWrite(lm2, HIGH);
    analogWrite(lmPWM, power);
    digitalWrite(rm1, HIGH);
    digitalWrite(rm2, LOW);
    analogWrite(rmPWM, power);

  }
  else
  {
    digitalWrite(lm1, HIGH);
    digitalWrite(lm2, LOW);
    analogWrite(lmPWM, power);
    digitalWrite(rm1, LOW);
    digitalWrite(rm2, HIGH);
    analogWrite(rmPWM, power);
  }
  delay(abs(error) * 1);
  brake();
  delay(50);
}
void brake() {
 
  analogWrite(lmPWM, 0);
  digitalWrite(lm1, HIGH);
  digitalWrite(lm2, HIGH);
  analogWrite(rmPWM, 0);
  digitalWrite(rm1, HIGH);
  digitalWrite(rm2, HIGH);
  delay(100);
}

void MoveT(int h, int r, float sec)
{
  heading = h;
  mSpeed = r;
  t = millis();
  while (millis() < t + (sec * 1000)) {
    motors();
  }
  brake();
}
void Point(int h) {
  heading = h;
  mSpeed = 0;
  t = millis();
  while (millis() < t + (2000)) {
    motors();
  }
}
void MoveD(int h, int r,float sec, float dis,int Pulse)
{
  int Sense=0;
  heading = h;
  mSpeed = r;
  if (Pulse == lsPulse)
  {
    Pulse=lsPulse;
   Sense=lfSR04;
  }
  else if (Pulse ==  MidPulse)       //Rey
  {
    Pulse=MidPulse;
    Sense=MidSR04;
  }
  else
  {
  Pulse=rsPulse;  
   Sense = rfSR04;  
   
  }
}
void MoveB(int h, int r,float sec, float dis,int Pulse)
{
  int Sense = 0;
  heading = h;
  mSpeed = r;
  if (Pulse == lsPulse)
  {
    Pulse=lsPulse;
   Sense=lfSR04;
  }
  else if (Pulse ==  MidPulse)       //Rey
  {
    Pulse=MidPulse;
    Sense=MidSR04;
  }
  else
  {
  Pulse=rsPulse;  
   Sense = rfSR04;  
   
  }
while (SR04(Pulse,Sense) < dis)
{
  Serial.println(Sense);
  motors();
  }
  mSpeed = r/2;
  while (millis() < t + (sec * 1000)) {
    motors();
  }
  brake();
}

void Ramp(int h, int r,float sec, float dis,int Pulse)
{
 // imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  int Sense=0;
  heading = h;
  mSpeed = r;
  y=0;
  while(abs(y) < 14){
    
    text(y);
    motors();
  }
  while(abs(y) > 6){
    motors();
  }
  if (Pulse == lsPulse)
  {
    Pulse=lsPulse;
   Sense=lfSR04;
  }
  else
  {
  Pulse=rsPulse;  
   Sense = rfSR04;  
  }

while (SR04(Pulse,Sense) > dis)
{
  motors();
  }
  mSpeed = r/2;
  while (millis() < t + (sec * 1000)) {
    motors();
  }
  brake();
}

void lift()
{
   myservo.attach(liftServo);
   delay(100);
  myservo.write(37);              // tell servo to go to up position 
  delay(100);                       // waits for the servo to reach the position
  myservo.write(120);
  delay(500);
  myservo.write(42); 
  delay(500);
  }
void Key()
{
  myservoB.attach(switchServo);
  myservoB.write(10);              // tell servo to go to up position
  delay(1300);                       // waits for the servo to reach the position
  myservoB.write(155);
  delay(1000);
  myservoB.write(10);
  delay(1300);
  myservoB.detach();
  

  }

long SR04(int Pulse, int Sense)
{
  pinMode(Pulse,OUTPUT);
  pinMode(Sense,INPUT);
//while(millis() < tp + 10){}
digitalWrite(Pulse,LOW);
delayMicroseconds(4); 
digitalWrite(Pulse,HIGH);
delayMicroseconds(10000); 
digitalWrite(Pulse,LOW); 
dist = pulseIn(Sense,HIGH)/58.2;
//tp=millis();
Serial.println(dist);
text(dist);
return dist;
}
void wheel()
{
  digitalWrite(fm1, HIGH);
  digitalWrite(fm2, LOW);
  analogWrite(fmPWM, 100);
  while ( switchcount < 5) {


    while ( digitalRead(capSwitch) == HIGH) {

      //Serial.println(switchcount);
      //delay(2500);
      //
    }
    delay(150);
    switchcount++;
    Serial.println(switchcount);
  }
  analogWrite(fmPWM, 0);
}
void text(int s)
{ //Serial.println(s);
  Display.clearDisplay();
 Display.setTextSize(1);
 Display.setTextColor(WHITE, BLACK);
 Display.setCursor(0, 0);
 Display.println(s);
 Display.display();
  }
void loop() {
  //void MoveD(int h, int r, ,float sec, float dis,int Pulse)
  
 //  Key();
  displayCalStatus();
  calibration();
  text(56);
   
  SR04(rsPulse,rfSR04);
 Serial.println(dist); 
 //RmotorTest();
     
    //  }
delay(2000);
//lift();
//while(1==1){
  //SR04(lsPulse,lfSR04);
  //IMU();
  
// Serial.println(x); 
//  }
Point(0);
delay(1000);
 Point(270);  // Point at west A Button
  MoveT(270,150,2.4);
    MoveT(270,75,.75);             //Rey
    MoveT(270,-150,2.2);
  Point(0); // point down ramp
 //Ramp(0,180,0,90,rsSR04);
 MoveT(0,150,5);
 //MoveD(0,150,0,200,lsSR04);     //Davis  Rey
 Point(270);  // Point at west B wall
 MoveT(270,150,1.3);          //now
//  Point(0);   // Drive by Dest B switch
 //MoveT(0,150,1.6); 
 brake();
 Key();
 //while(1==1){}
 Point(0);
  MoveT(0,150,1.25);
  Point(45); // Point at Treasure
  MoveT(45,150,3);
 lift();  // Pick up Treasure
 MoveT(45,150,.75);
 Point(270);  // Point at west wall
//MoveT(270,-150,.6);  // Back up before centering
//MoveD(270,150,0,39.32,rsSR04);//  Move to center
MoveT(270,150,1.45);         //Rey
 Point(0); // Point at ship's wheel
  //MoveD(0,150,0,6,lsSR04);      //mOE
   MoveT(0,75,3);     //Rey
   MoveT(0,-50,.5);
  delay(300); 
  wheel();
  //delay(3000); // This is where motor will turn
   MoveT(0,-150,1); // back away from wheel
 Point(180); // Point at ramp
 Ramp(180,255,0,20,rsSR04);
 Point(270); // Point at west dest C button
 //MoveD(270,150,2.0,22,rsSR04);
  MoveT(270,75,7);
 while (1==1){}  // All done!
 Point(270);
  MoveT(270,150,1.87);
    text(270);
    
    MoveT(270,-150,1.75);
    Point(0);
    MoveT(0,100,4.7);
    Point(90);
    MoveT(90,150,1.41); 
     text(90);
    Point(0);
    MoveT(0,150,2.8);
    text(0);
    Point(270);
    MoveT(270,150,1.6);
    text(270);
    Point(180);
    MoveT(180,150,6.8);
    text(180);
    Point(90);
    text(90);
    MoveT(90,150,2.1);
      text(90);
  
 brake();
 while(1==1){}
}
