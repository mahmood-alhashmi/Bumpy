#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include <MPU6050.h>

const int receivePin = 4;  // Change this to your chosen pin



// for measuring angles
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;



//create objects for the servos
Servo servo_ul;
Servo servo_ur;


//set the pin values for each servo 
int servo_up_left = 3; 
int servo_up_right = 2;


// set variables for the angle of the servo
int servo_up_left_angle = 70;
int servo_up_right_angle = 80;


// initiallise the gyroscope 



//Create the gyro object

MPU6050 mpu;
// class default I2C address is 0x68

// create variables for each offset so we can zerro error the reading
int16_t gyroX_offset = 0;
int16_t gyroY_offset = 0;
int16_t gyroZ_offset = 0;

//declare variables for left and right motor
const int enL = 11 ;
const int in1 = 10 ;
const int in2 = 9 ;
const int enR = 5 ;
const int in3 = 8;
const int in4 = 7;

float set_angle = -30;

float Kp = 6;
float Ki = 1;
float Kd = 0.01;

float previous_error = 0;
float integral = 0;

char receiveByte() {
  char receivedByte = 0;

  for (int i = 7; i >= 0; i--) {
    bool receivedBit = digitalRead(receivePin);
    receivedByte |= (receivedBit << i);

    delayMicroseconds(500);  // Adjust this delay according to your requirements
  }

  return receivedByte;
}


void measure_angle_setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

}



void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}


void measure_angle() {
  gyro_signals();
  Serial.print(" AngleRoll= ");
  Serial.print(AngleRoll);
  Serial.print(" AnglePitch= ");
  Serial.print(AnglePitch);


  Serial.print(" AccX= ");
  Serial.print(AccX);
  Serial.print(" AccY= ");
  Serial.print(AccY);
  Serial.print(" AccZ= ");
  Serial.println(AccZ);


  delay(50);
}


void setup() {

  pinMode(receivePin, INPUT);

    //set the pins as outputs


    measure_angle_setup();

    //Attach the servo to the pins 
  servo_ul.attach(servo_up_left);
  servo_ur.attach(servo_up_right);


    //write the angle for each servo
  servo_ul.write(servo_up_left_angle);
  servo_ur.write(servo_up_right_angle);

  delay(30);




}

void loop() {
  


  servo_ul.write(servo_up_left_angle);
  servo_ur.write(servo_up_right_angle);



  measure_angle();



  // create the PID controller
  // Compute PID error
  float error = abs(-30 - AnglePitch);

  // Update integral term
  integral += error;

  // Limit the integral term to prevent windup
  integral = constrain(integral,-50, 50);
  // Compute derivative term
  float derivative = error - previous_error;

  // Compute PID output
  float motor_speed = Kp * error + Ki * integral + Kd * derivative;

  // Update previous error
  previous_error = error;



  motor_speed = constrain(motor_speed,-230,230);
  motor_speed = map(motor_speed,0,200,0,200);


  


 
    //control direction based on where it is falling 
  if (AnglePitch >= -30) { 
    analogWrite(enR, motor_speed);
    analogWrite(enL, motor_speed);
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    Serial.print("motor speed= ");
    Serial.print(motor_speed);


  } 
  else if (AnglePitch < -30) {
    analogWrite(enR, motor_speed);
    analogWrite(enL, motor_speed);
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Serial.print("motor speed= ");
    Serial.print(motor_speed);

  }

  char receivedMessage[50];
  int messageIndex = 0;

  while (true) {
    char receivedChar = receiveByte();

    if (receivedChar == '\0') {
      receivedMessage[messageIndex] = '\0';
      Serial.print("Received: ");
      Serial.println(receivedMessage);
      break;
    }

    receivedMessage[messageIndex] = receivedChar;
    messageIndex++;
  }
  delay(30);
}











