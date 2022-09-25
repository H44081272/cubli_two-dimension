#include <Wire.h>
#include <analogWrite.h>
#include <MPU6050_tockn.h>
#include <Ticker.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define BUZZER  23

#define A_X 35
#define B_X 34
#define FR_X 33
#define OnOff_X 25
#define PWM_X 32
float offset_X = -2.51; //小车的机械中值-2.65
float offset1=  -33.57 ;
int Balance_Pwm_X;
int Velocity_Pwm_X;
//PD参数与PI参数
float Balance_Kp_X = 30  , Balance_Kd_X = 10  ;//50//20//0.05     //30/10/0.1   //200/40/0.05   //1200/300/0.05
float Velocity_Kp_X = 0.1, Velocity_Ki_X = (1 / 200) * Velocity_Kp_X ;
//--------------------------------------------------------------------------
#define A_Y 5
#define B_Y 18
#define FR_Y 4
#define OnOff_Y 16
#define PWM_Y 17
float offset_Y = 25.57; //小车的机械中值
float offset2=  13.67;
int Balance_Pwm_Y;
int Velocity_Pwm_Y;
//PD参数与PI参数
float Balance_Kp_Y = -60 , Balance_Kd_Y = -20 ;//20//8
float Velocity_Kp_Y = 0.1, Velocity_Ki_Y = (1 / 200) * Velocity_Kp_Y ;
//--------------------------------------------------------------------------
#define A_Z 27
#define B_Z 26
#define FR_Z 14
#define OnOff_Z 12
#define PWM_Z 13
float offset_Z = 29.8; //小车的机械中值-2.65
float offset3= 11.62  ;
int Balance_Pwm_Z;
int Velocity_Pwm_Z;
//PD参数与PI参数
float Balance_Kp_Z = 50 , Balance_Kd_Z = 20 ;//20//8
float Velocity_Kp_Z = 0.05, Velocity_Ki_Z = (1 / 200) * Velocity_Kp_Z ;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
float Angle;//角度显示
float angleY;
float Gyro;//角度显示
bool vertical = false;
int balancing_point = 0;


int cc_X;
int cc_Y;
int cc_Z;
volatile long pulseCount_X = 0;
volatile long pulseCount_Y = 0;
volatile long pulseCount_Z = 0;
int encoder_X = 0;
int encoder_Y = 0;
int encoder_Z = 0;

MPU6050 mpu6050(Wire);

//定时中断
Ticker ticker1;


void beep() {
  if (!vertical) {
    digitalWrite(BUZZER, HIGH);
    delay(70);
    digitalWrite(BUZZER, LOW);
  }
}

void countpulse_X() {
  if (digitalRead(A_X) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(B_X) == LOW) pulseCount_X--;//根据另外一相电平判定方向
    else pulseCount_X++;
  }
  else { //如果是上升沿触发的中断
    if (digitalRead(B_X) == LOW) pulseCount_X++; //根据另外一相电平判定方向
    else pulseCount_X--;
  }
}

void countpulse_Y() {
  if (digitalRead(A_Y) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(B_Y) == LOW) pulseCount_Y--;//根据另外一相电平判定方向
    else pulseCount_Y++;
  }
  else { //如果是上升沿触发的中断
    if (digitalRead(B_Y) == LOW) pulseCount_Y++; //根据另外一相电平判定方向
    else pulseCount_Y--;
  }
}

void countpulse_Z() {
  if (digitalRead(A_Z) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(B_Z) == LOW) pulseCount_Z--;//根据另外一相电平判定方向
    else pulseCount_Z++;
  }
  else { //如果是上升沿触发的中断
    if (digitalRead(B_Z) == LOW) pulseCount_Z++; //根据另外一相电平判定方向
    else pulseCount_Z--;
  }
}

void control() {
  sei();//全局中断开启
  mpu6050.update();
  Angle = mpu6050.getAngleX();
  angleY = mpu6050.getAngleY();
  Gyro = mpu6050.getGyroX();
  //Serial.print("Angle");
  Serial.print(Angle);
  //Serial.print("angleY");
  //Serial.println(angleY);

  if (abs(Angle - offset_X) < 0.6 && abs(angleY - offset1) < 0.6) {
    balancing_point = 1;
    beep();
    vertical = true;
  } else if (abs(Angle - offset_Y) < 0.6 && abs(angleY - offset2) < 0.6) {
    balancing_point = 2;
    beep();
    vertical = true;
  } else if (abs(Angle - offset_Z) < 0.6 && abs(angleY - offset3) < 0.6) {
    balancing_point = 3;
    beep();
    vertical = true;
  }

  if (balancing_point == 1) {
    if (abs(Angle - offset_X) > 5)
      vertical = false;
  } else if (balancing_point == 2) {
    if (abs(Angle - offset_Y) > 5)
      vertical = false;
  } else if (balancing_point == 3) {
    if (abs(Angle - offset_Z) > 5)
      vertical = false;
  }

  if (vertical) {
    if (balancing_point == 1) {
      MotorX_control();
    } else if (balancing_point == 2) {
      MotorY_control();
    } else if (balancing_point == 3) {
      MotorZ_control();
    }
  } else {
    set_motor_speed(0, FR_X, OnOff_X, PWM_X);
    set_motor_speed(0, FR_Y, OnOff_Y, PWM_Y);
    set_motor_speed(0, FR_Z, OnOff_Z, PWM_Z);
  }
}

//中断函数
void MotorX_control() {
  Balance_Pwm_X = balance(Angle - Velocity_Pwm_X, offset_X, Gyro, Balance_Kp_X, Balance_Kd_X);
  set_motor_speed(Balance_Pwm_X, FR_X, OnOff_X, PWM_X);
  //Serial.print(" , ");
  //Serial.println(Balance_Pwm_X);
  //Serial.print("Balance_Pwm_X");
  //Serial.println(Balance_Pwm_X);
  //Serial.print(" ");
  cc_X++;
  if (cc_X >= 2)  //5*2=10，enter PI algorithm of speed per 40ms
  {
    encoder_X = pulseCount_X;
    pulseCount_X = 0;
    Velocity_Pwm_X = velocity(encoder_X, Velocity_Kp_X, Velocity_Ki_X);
    cc_X = 0; //Clear
  }
}
void MotorY_control() {
  Balance_Pwm_Y = balance(Angle + Velocity_Pwm_Y, offset_Y, Gyro, Balance_Kp_Y, Balance_Kd_Y);
  set_motor_speed(Balance_Pwm_Y, FR_Y, OnOff_Y, PWM_Y);
  //Serial.print("Balance_Pwm_Y");
  //Serial.println(Balance_Pwm_Y);
  cc_Y++;
  if (cc_Y >= 2)  //5*2=10，enter PI algorithm of speed per 40ms
  {
    encoder_Y = pulseCount_Y;
    pulseCount_Y = 0;
    Velocity_Pwm_Y = velocity(encoder_Y, Velocity_Kp_Y, Velocity_Ki_Y);
    cc_Y = 0; //Clear
  }
}
void MotorZ_control() {
  Balance_Pwm_Z = balance(Angle + Velocity_Pwm_Z, offset_Z, Gyro, Balance_Kp_Z, Balance_Kd_Z);
  set_motor_speed(Balance_Pwm_Z, FR_Z, OnOff_Z, PWM_Z);
  //Serial.print("Balance_Pwm_Z");
  //Serial.println(Balance_Pwm_Z);
  cc_Z++;
  if (cc_Z >= 2)  //5*2=10，enter PI algorithm of speed per 40ms
  {
    encoder_Z = pulseCount_Z;
    pulseCount_Z = 0;
    Velocity_Pwm_Z = velocity(encoder_Z, Velocity_Kp_Z, Velocity_Ki_Z);
    cc_Z = 0; //Clear
  }
}

void setup() {

  pinMode(BUZZER, OUTPUT);

  pinMode(FR_X, OUTPUT);
  pinMode(OnOff_X, OUTPUT);
  digitalWrite(FR_X, LOW); //TB6612控制引脚拉低
  digitalWrite(OnOff_X, LOW); //TB6612控制引脚拉低
  pinMode(A_X, INPUT_PULLUP);//编码器引脚A
  pinMode(B_X, INPUT); //编码器引脚B
  //----------------------------------------------
  pinMode(FR_Y, OUTPUT);
  pinMode(OnOff_Y, OUTPUT);
  digitalWrite(FR_Y, LOW); //TB6612控制引脚拉低
  digitalWrite(OnOff_Y, LOW); //TB6612控制引脚拉低
  pinMode(A_Y, INPUT_PULLUP);//编码器引脚A
  pinMode(B_Y, INPUT); //编码器引脚B
  //----------------------------------------------
  pinMode(FR_Z, OUTPUT);
  pinMode(OnOff_Z, OUTPUT);
  digitalWrite(FR_Z, LOW); //TB6612控制引脚拉低
  digitalWrite(OnOff_Z, LOW); //TB6612控制引脚拉低
  pinMode(A_Z, INPUT_PULLUP);//编码器引脚A
  pinMode(B_Z, INPUT); //编码器引脚B

  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    //Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("offset_X = -2.49");
  display.setCursor(0, 10);
  display.println("offset_Y = 25.57");
  display.setCursor(0, 20);
  display.println("offset_Z = 29.8");
  display.display(); 

  //MPU
  Wire.begin();
  mpu6050.begin();
  mpu6050.setGyroOffsets(0, 0, 0);

  delay(20);


  //中断
  ticker1.attach(0.005, control);//5ms
  attachInterrupt(digitalPinToInterrupt(A_X), countpulse_X , CHANGE);
  attachInterrupt(digitalPinToInterrupt(A_Y), countpulse_Y , CHANGE);
  attachInterrupt(digitalPinToInterrupt(A_Z), countpulse_Z , CHANGE);
}

//PI函数
int velocity(int encoder, float Velocity_Kp, float Velocity_Ki)
{

  // 定义成静态变量，保存在静态存储器，使得变量不丢掉
  static int PWM_out, Encoder_Err, Encoder_S, EnC_Err_Lowout, EnC_Err_Lowout_last;
  float a = 0.7;
  int velocity;

  // 1.计算速度偏差
  //舍去误差--我的理解：能够让速度为"0"的角度，就是机械中值。
  Encoder_Err = encoder;
  EnC_Err_Lowout = (1 - a) * Encoder_Err + a * EnC_Err_Lowout_last; // 使得波形更加平滑，滤除高频干扰，放置速度突变
  EnC_Err_Lowout_last = EnC_Err_Lowout;   // 防止速度过大影响直立环的正常工作
  // 3.对速度偏差积分出位移
  Encoder_S += EnC_Err_Lowout;
  // 4.积分限幅
  Encoder_S = Encoder_S > 10000 ? 10000 : (Encoder_S < (-10000) ? (-10000) : Encoder_S);

  // 5.速度环控制输出
  velocity = Velocity_Kp * EnC_Err_Lowout + Velocity_Ki * Encoder_S;

  return velocity;
}



void set_motor_speed(int spd, int pinFR, int pinOnOff, int pinPWM) {

  int power;

  if (spd > 255)
    spd = 255;
  else if (spd < -255)
    spd = -255;

  //Serial.print("spd");
  Serial.print(" , ");
  Serial.println(spd);

  if (spd == 0)
  {
    analogWrite(pinPWM, 255);
    digitalWrite(pinOnOff, LOW);
  }
  else if (spd > 0)
  {
    power = map(spd, 0, 255, 255, 0);
    analogWrite(pinPWM, power);
    digitalWrite(pinFR, LOW);
    digitalWrite(pinOnOff, HIGH);
  }
  else if (spd < 0)
  {
    power = map(spd, -255, 0, 0, 255 );
    analogWrite(pinPWM, power);
    digitalWrite(pinFR, HIGH);
    digitalWrite(pinOnOff, HIGH);
  }

}

//PD函数
int balance(float Angle, float Mechanical_balance, float Gyro, float Balance_Kp, float Balance_Kd)
{
  float Bias;//角度误差
  int balance;//直立环计算出来的电机控制pwm
  Bias = Angle - Mechanical_balance; //===求出平衡的角度中值 和机械相关
  Serial.print(" , ");
  Serial.print(Bias);
  balance = Balance_Kp * Bias + Balance_Kd * Gyro ; //===计算平衡控制的电机PWM PD控制 kp是P系数 kd是D系数
  return balance;
}


void loop() {

}
