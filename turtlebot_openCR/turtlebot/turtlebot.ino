#include <ros.h>

#include <std_msgs/String.h>

#include <std_msgs/Int32.h>

#include <RC100.h>

#include <DynamixelSDK.h>
 
#define LEFT_DXL  1

#define RIGHT_DXL 2

#define BAUDRATE 1000000

#define DEVICENAME ""

#define PROTOCOL_VERSION 2.0

#define ADDR_TORQUE_ENABLE 64

#define ADDR_GOAL_VELOCITY 104

#define LEN_TORQUE_ENABLE 1

#define LEN_GOAL_VELOCITY 4

#define ON  1

#define OFF 0

#define VELOCITY 10

dynamixel::PortHandler *portHandler;

dynamixel::PacketHandler *packetHandler;

dynamixel::GroupSyncWrite *groupSyncWrite;

bool dxl_addparam_result = false;

int8_t dxl_comm_result = COMM_TX_FAIL;

uint8_t dxl_error = 0;

int vel[2] = {0, 0};

int const_vel = 200;

RC100 Controller;

int RcvData = 0;

ros::NodeHandle  nh;

int Intercept = 0; // 차선 직선의 y절편

int barAndLight = 0;   //영상으로 전달받는 신호등과 차단바 값 저장 변수

int pre_Intercept=0;  // 이전 차선 직선의 y절편

int parkSignal = 0; 

/*

 pid 제어를 위한 각종 변수 선언.

 */

double Kp = 1;

double Ki = 2.5;

double Kd = 3.2;

double error;

double error_previous;

double desired_value = 120;

double current_value = Intercept;

double Time = 0.004;

double P_control, I_control, D_control;

double PID_control;

 
/*

 초음파 센서 pin 지정 및 초음파가 되돌아온 시간, 거리 변수 선언

 */

//right

const int trigPin1 = 7;

const int echoPin1 = 6; 

//mid

const int trigPin2 = 13;

const int echoPin2 = 12;

//left

const int trigPin3 = 9;

const int echoPin3 = 8;

long duration1;

int  mid_distance;

long duration2;

int right_distance;

long duration3;

int left_distance;

// 조도센서는 빛의 밝기에 대하여 전기저인 성질로 변환시켜주는 센서입니다.

// 조도센서를 아날로그 A1핀으로 설정합니다.
int cds = A1;

// LED를 A0핀으로 설정합니다.
int led = A0;


String remoteControl;  //원격 컴퓨터에서 "go"일경우 코드를 실행하고, "stop" 일경우 정지

//y절편의 데이터를 Intercept 변수에 저장
void messageIntercept( const std_msgs::Int32& msg){

  Intercept = msg.data;

}
 
//신호등과 차단바감지 상태를 barAndLight 변수에 저장
void messageBarAndLight( const std_msgs::Int32& msg){

  barAndLight = msg.data;

}

//원격제어를 위한 데이터를 remoteControl 변수에 저장
void messageControl(const std_msgs::String& msg){
  
  remoteControl = msg.data;
  
}

//parkSignal을 받아와 parkSignal 변수에 저장.
void messagePark( const std_msgs::Int32& msg){
  parkSignal = msg.data;
}

ros::Subscriber<std_msgs::Int32> interceptsub("ros_Intercept_msg", messageIntercept); //ros_Intercept_msg 구독

ros::Subscriber<std_msgs::Int32> barlightsub("ros_bar_light_msg", messageBarAndLight); //ros_bar_light_msg 구독

ros::Subscriber<std_msgs::String> remoteControlsub("remoteControl_msg", messageControl); // remoteControl_msg 구독

ros::Subscriber<std_msgs::Int32> parksub("ros_parkzone_msg", messagePark); //ros_bar_light_msg 구독
 

void setup() 

{

  Serial.begin(9600);

  Controller.begin(1);

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);

  portHandler -> openPort();

  portHandler->setBaudRate(BAUDRATE);

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, LEFT_DXL, ADDR_TORQUE_ENABLE, ON, &dxl_error);

  packetHandler->printTxRxResult(dxl_comm_result);

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, RIGHT_DXL, ADDR_TORQUE_ENABLE, ON, &dxl_error);

  packetHandler->printTxRxResult(dxl_comm_result);

  pinMode(led, OUTPUT);

  controlMotor(0, 0);

  delay(30);

  pinMode(trigPin1, OUTPUT);

  pinMode(echoPin1, INPUT);

  pinMode(trigPin2, OUTPUT);

  pinMode(echoPin2, INPUT);

  pinMode(trigPin3, OUTPUT);

  pinMode(echoPin3, INPUT);

  nh.initNode();

  nh.subscribe(interceptsub);

  nh.subscribe(barlightsub);

  nh.subscribe(remoteControlsub);

  nh.subscribe(parksub);

}

void loop() 

{
  nh.spinOnce();

  delay(100);

  /* 메시지가 go일경우 주어진 함수 실행*/

  if(remoteControl == "go"){
    pid();
    park();
    barAndLightAction();

    //조도 센서의 값이 낮아지면 터널이라고 인식
    int cdsValue = analogRead(cds);
    if(cdsValue < 900){
      for( ;cdsValue < 900; ){
        digitalWrite(led, HIGH); //터널안에서는 LED를 킨다.
        ultra_distance();
        maze(); //터널을 빠져나올때까지 우수법 알고리즘 사용
      }
    }
    //터널 안이 아닐 경우 LED 끈다.
    digitalWrite(led, LOW);
  }

  else if(remoteControl == "stop"){
    controlMotor(0,0);
  }
}

 

/*

 오른쪽, 왼쪽, 중앙 초음파 센서의 거리값을 읽어 와 주는 함수

 */

void ultra_distance(){  

  //초음파 초기화

  digitalWrite(trigPin1, LOW);

  delayMicroseconds(2);

  //초음파 송신

  digitalWrite(trigPin1, HIGH);

  delayMicroseconds(10);

  //10ms 후 정지

  digitalWrite(trigPin1, LOW);

  //리턴된 시간 데이터 읽기

  duration1 = pulseIn(echoPin1, HIGH);

  //시간 데이터를 거리데이터로 환산 식

  right_distance = duration1*0.034/2; //Cm

  //if Inch, duration *0.0133/2

  //화면에 출력

  Serial.print("right Distance : ");

  Serial.print(right_distance);

  Serial.println("CM");

  digitalWrite(trigPin2, LOW);

  delayMicroseconds(2);

  //초음파 송신

  digitalWrite(trigPin2, HIGH);

  delayMicroseconds(10);

  //10ms 후 정지

  digitalWrite(trigPin2, LOW);

  //리턴된 시간 데이터 읽기

  duration2 = pulseIn(echoPin2, HIGH);

  //시간 데이터를 거리데이터로 환산 식

  mid_distance = duration2*0.034/2; //Cm

  //if Inch, duration *0.0133/2

  //화면에 출력

  Serial.print("Mid Distance : ");

  Serial.print(mid_distance);

  Serial.println("CM");

  digitalWrite(trigPin3, LOW);

  delayMicroseconds(2);

  //초음파 송신

  digitalWrite(trigPin3, HIGH);

  delayMicroseconds(10);

  //10ms 후 정지

  digitalWrite(trigPin3, LOW);

  //리턴된 시간 데이터 읽기

  duration3 = pulseIn(echoPin3, HIGH);

  //시간 데이터를 거리데이터로 환산 식

  left_distance = duration3*0.034/2; //Cm

  //if Inch, duration *0.0133/2

  //화면에 출력

  Serial.print("left Distance : ");

  Serial.print(left_distance);

  Serial.println("CM");

  delay(10);

}

/*

 우수법 알고리즘을 이용하여 터널을 탈출하게 해주는 함수

 */

void maze(){

  if(right_distance >13){

    forward(90,90);

    delay(800);

    right90();

    forward(140,140);

    delay(2000);

  }

  else if(right_distance < 13 && mid_distance < 13 && left_distance > 13){

    forward(90,90);

    delay(800);

    left90();

  }

  else if(right_distance < 13 && mid_distance < 13 && left_distance < 13){

    right180();

    forward(140,140);

    delay(500);

  }
}

 

/*
3일때 빨간불 감지

5일때 차단바 감지
*/
void barAndLightAction(){

  //빨간불일 경우 1초간 정지 한다.
  if(barAndLight == 3){
    controlMotor(0,0);
    delay(1000);
  }

  //차단바가 있을 경우 1초간 정지 한다.
  else if(barAndLight == 5){
    controlMotor(0,0);
    delay(1000);
  }

}

 

//왼쪽,오른쪽 모터값을 받아서 제어하는 함수

void forward(int left_speed, int right_speed){

    controlMotor(left_speed, right_speed);

}

 

//멈출 시간을 받아서, 터틀봇을 멈추게 하는 함수

void stop(int delay){

  controlMotor(0,0);

  delay(delay);

}

 

//왼쪽으로 90도 꺽는 함수

void left90(){

  controlMotor(-150, 150);

  delay(1075);

  controlMotor(0,0);

  delay(2000);

}

 

//왼쪽으로 180도 꺽는 함수

void left180(){

    controlMotor(-150, 150);

  delay(2150);

  controlMotor(0,0);

  delay(2000);

}

 

//오른쪽으로 90도 꺽는 함수

void right90(){

  controlMotor(150, -150);

  delay(1075);

  controlMotor(0,0);

  delay(2000);

}

 

//오른쪽으로 180도 꺽는 함수

void right180(){

  controlMotor(150, -150);

  delay(2150);

  controlMotor(0,0);

  delay(2000);

}

 

//터널안에서 90도 꺽는 함수

void maze_right90(){
  
  controlMotor(170, 0);

  delay(2000);

  controlMotor(0,0);

  delay(2000);
}

 

//터널안에서 왼쪽으로 90도 꺽는 함수

void maze_left90(){

  controlMotor(0, 170);

  delay(2000);

  controlMotor(0,0);

  delay(2000);
}

 

//왼쪽 모터와, 오른쪽 모터의 속력을 지정해 주는 함수.

void controlMotor( int64_t left_wheel_value, int64_t right_wheel_value){
  
  bool dxl_addparam_result;

  int8_t dxl_comm_result;

  dxl_addparam_result = groupSyncWrite->addParam(LEFT_DXL, (uint8_t*)&left_wheel_value);

  dxl_addparam_result = groupSyncWrite->addParam(RIGHT_DXL, (uint8_t*)&right_wheel_value);

  dxl_comm_result = groupSyncWrite->txPacket();

  groupSyncWrite->clearParam();

}

 

/*

 모터를 차선의 y절편의 값에 따라, pid제어를 통해서 도로를 주행하게 해주는 함수

 */

void pid(){

  current_value = Intercept;

  error = desired_value - current_value;

  P_control = Kp * error;

  I_control = I_control + Ki * error * Time;

  D_control = Kd * (error - error_previous) / Time;

  PID_control = P_control + I_control + D_control;

  PID_control = constrain(PID_control, 0, 250);

  controlMotor(130+PID_control,130-PID_control);

  error_previous = error;
}

//주차장임을 감지했을때 주차를 해주는 함수
void park(){ 
  if(parkSignal == 2){
   //터틀봇이 없을경우 바로 주차를 한다.
   if(right_distance > 13){
    
    right90();
    stop(1000);
    controlMotor(120, 120);
    delay(2500);
    stop(1000);
    left180();
    stop(1000);
    controlMotor(100, 100);
    delay(3000);
    stop(1000);
    right90();
    
    }
    //터틀봇이 주차되있을 경우 앞에 구역에다가 주차를 한다.
    else if(right_distance <= 13){
      controlMotor(100, 100);
      delay(4000);
      right90();
      stop(1000);
      controlMotor(120, 120);
      delay(2500);
      stop(1000);
      left180();
      stop(1000);
      controlMotor(100, 100);
      delay(3000);
      stop(1000);
      right90();
      }
  }
}
